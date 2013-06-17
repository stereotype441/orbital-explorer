/*
 * This file is part of the Electron Orbital Explorer. The Electron
 * Orbital Explorer is distributed under the Simplified BSD License
 * (also called the "BSD 2-Clause License"), in hopes that these
 * rendering techniques might be used by other programmers in
 * applications such as scientific visualization, video gaming, and so
 * on. If you find value in this software and use its technologies for
 * another purpose, I would love to hear back from you at bjthinks (at)
 * gmail (dot) com. If you improve this software and agree to release
 * your modifications under the below license, I encourage you to fork
 * the development tree on github and push your modifications. The
 * Electron Orbital Explorer's development URL is:
 * https://github.com/bjthinks/orbital-explorer
 * (This paragraph is not part of the software license and may be
 * removed.)
 *
 * Copyright (c) 2013, Brian W. Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * + Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * + Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdexcept>
#include <vector>
#include <set>
#include <map>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex>
#include <sys/time.h>
#include <GL/glew.h>
#include <SDL.h>

#include "callbacks.hh"
#include "shaders.hh"
#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "quaternion.hh"
#include "transform.hh"
#include "util.hh"
#include "delaunay.hh"
#include "function.hh"
#include "polynomial.hh"
#include "wavefunction.hh"
#include "tetrahedralize.hh"
#include "oopengl.hh"
#include "mouseevents.hh"
#include "controls.hh"

using namespace std;

#define GetGLError() GetGLError_(__LINE__)

static void GetGLError_(int line)
{
  GLenum err;
  if ((err = glGetError()) != GL_NO_ERROR) {
    fprintf(stderr, "Detected OpenGL error at line %d\n", line);
    do
      fprintf(stderr, "Error code %d (0x%x)\n", err, err);
    while ((err = glGetError()) != GL_NO_ERROR);
    exit(1);
  }
}

// GPU timers
static GLuint timer_query[4];

// GLSL programs
static Program *solidProg, *cloudProg, *overlayProg;

// Vertex array objects
static VertexArrayObject *solid, *cloud, *rect;

// Framebuffer names
static GLuint solidFBO, cloudFBO;

// Textures
static Texture *solidRGBTex, *solidDepthTex, *cloudDensityTex;

// This records the number of primitives, not the number of indices
static unsigned num_tetrahedra;

// The function to visualize
static Orbital *orbital = NULL;

// Subdivision of space into tetrahedra
static TetrahedralSubdivision *ts = NULL;

Texture *attachNewTexture(GLint internalformat, GLenum format,
                          GLenum attachment)
{
  Texture *tex = new Texture();
  tex->bind(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, internalformat, 1, 1, 0,
               format, GL_BYTE, NULL);
  glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, attachment, GL_TEXTURE_2D,
                         *tex, 0);
  return tex;
}

void resizeTexture(Texture *name, GLint internalformat, GLenum format,
                   GLuint width, GLuint height)
{
  name->bind(GL_TEXTURE_2D);
  glTexImage2D(GL_TEXTURE_2D, 0, internalformat, width, height, 0,
               format, GL_BYTE, NULL);
}

static void checkFramebufferCompleteness()
{
  GLenum isComplete = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
  if (isComplete != GL_FRAMEBUFFER_COMPLETE) {
    printf("Framebuffer not complete!\n");
    printf("glCheckFramebufferStatus returned %x\n", isComplete);
    GetGLError();
    exit(1);
  }
}

void initialize()
{
  glGenQueries(4, timer_query);
  for (int i = 0; i < 4; ++i)
    glQueryCounter(timer_query[i], GL_TIMESTAMP);

  solidProg = new Program();
  solidProg->vertexShader(solidVertexShaderSource);
  solidProg->fragmentShader(solidFragmentShaderSource);
  glBindAttribLocation(*solidProg, 0, "inPosition");
  glBindAttribLocation(*solidProg, 1, "inColor");
  glBindFragDataLocation(*solidProg, 0, "fragColor");
  solidProg->link();

  GetGLError();

  cloudProg = new Program();
  cloudProg->vertexShader(cloudVertexShaderSource);
  cloudProg->geometryShader(cloudGeometryShaderSource);
  cloudProg->fragmentShader(cloudFragmentShaderSource);
  glBindAttribLocation(*cloudProg, 0, "position");
  glBindAttribLocation(*cloudProg, 1, "uvY");
  glBindFragDataLocation(*cloudProg, 0, "integratedValue");
  cloudProg->link();

  GetGLError();

  overlayProg = new Program();
  overlayProg->vertexShader(overlayVertexShaderSource);
  overlayProg->fragmentShader(overlayFragmentShaderSource);
  glBindAttribLocation(*overlayProg, 0, "inPosition");
  glBindFragDataLocation(*overlayProg, 0, "RGB");
  overlayProg->link();

  GetGLError();

  // Solid objects
  solid = new VertexArrayObject();
  solid->bind();

  // Positions
  GLfloat solid_data[] = {
    // X axis
    0.0, 0.0, 0.0, 0.6400, 0.3300, 0.2126,
    1.0, 0.0, 0.0, 0.6400, 0.3300, 0.2126,

    // Y axis
    0.0, 0.0, 0.0, 0.3000, 0.6000, 0.3290,
    0.0, 1.0, 0.0, 0.3000, 0.6000, 0.3290,

    // Z axis
    0.0, 0.0, 0.0, 0.1500, 0.0600, 0.0721,
    0.0, 0.0, 1.0, 0.1500, 0.0600, 0.0721
  };
  solid->buffer(GL_ARRAY_BUFFER, solid_data, sizeof(solid_data));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat),
                        (void *)(3 * sizeof(GLfloat)));

  GetGLError();

  // Clouds
  cloud = new VertexArrayObject();

  GetGLError();

  // Big rectangle
  rect = new VertexArrayObject();
  rect->bind();

  // Positions
  GLfloat rect_data[] = {
    -1.0, -1.0, 0.0,
    -1.0,  1.0, 0.0,
    +1.0,  1.0, 0.0,
    +1.0, -1.0, 0.0,
  };
  rect->buffer(GL_ARRAY_BUFFER, rect_data, sizeof(rect_data));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);

  GetGLError();

  ////////////////////////////////////////////////
  // Set up OpenGL state that will never change //
  ////////////////////////////////////////////////

  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_ONE, GL_ONE);

  glClearColor(0., 0., 0., 1.);

  GetGLError();

  /////////////////////////////////////////////////////////////////
  // Set up auxiliary FBOs and textures for multistage rendering //
  /////////////////////////////////////////////////////////////////

  // Solid objects
  glGenFramebuffers(1, &solidFBO);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, solidFBO);
  solidRGBTex = attachNewTexture(GL_RGB8, GL_RGB, GL_COLOR_ATTACHMENT0);
  solidDepthTex = attachNewTexture(GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                                   GL_DEPTH_ATTACHMENT);
  checkFramebufferCompleteness();

  // Clouds
  glGenFramebuffers(1, &cloudFBO);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cloudFBO);
  cloudDensityTex = attachNewTexture(GL_RGB16F, GL_RGB, GL_COLOR_ATTACHMENT0);
  checkFramebufferCompleteness();

  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  GetGLError();
}

void resizeTextures()
{
  int width = getWidth();
  int height = getHeight();

  // Resize statically-sized textures
  resizeTexture(solidRGBTex, GL_RGB8, GL_RGB, width, height);
  resizeTexture(solidDepthTex, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                width, height);

  GetGLError();
}

void display()
{
  int width = getWidth();
  int height = getHeight();

  static int num_points = 0;
  bool just_started = false;

  // Are we just starting up, or did the wave function change?
  Orbital newOrbital = getOrbital();
  if (!orbital || *orbital != newOrbital) {
    // Stop any running thread
    if (ts)
      ts->kill();
    // delete checks for NULL, so we don't have to
    delete orbital;
    orbital = new Orbital(newOrbital);
    delete ts;
    ts = new TetrahedralSubdivision(*orbital, 10.0);
    num_points = 0;

    // Golden ratio
    const double phi = (1.0 + sqrt(5.0)) / 2.0;
    // 500, 800, 1300, 2100, 3400, 5500, 8900, 14400, 23300, 37700
    int v = 100 * int(pow(phi, getDetail() + 4.0) / sqrt(5.0) + 0.5);
    ts->runUntil(v);
    just_started = true;
  }

  // Only suck down new vertices and tetrahedra if at least 100 more
  // have been calculated -- because locking the mutex for the time it
  // takes to suck down primitives slows down subdivision substantially
  if ((ts->isRunning() && ts->numVertices() > num_points + 100) ||
      ts->isFinished() || just_started) {
    cloud->bind();

    // Tetrahedron indices
    std::vector<unsigned> indices = ts->tetrahedronVertexIndices();
    num_tetrahedra = indices.size() / 4;
    cloud->buffer(GL_ELEMENT_ARRAY_BUFFER, indices);

    GetGLError();

    // Vertex positions
    std::vector<float> positions = ts->vertexPositions();
    num_points = positions.size() / 3;

    // Vertex varying data
    std::vector<float> varyings(positions.size() * 2);
    Vector<3> x;
    for (int p = 0; p < num_points; ++p) {
      x[0] = varyings[6 * p]     = positions[3 * p];
      x[1] = varyings[6 * p + 1] = positions[3 * p + 1];
      x[2] = varyings[6 * p + 2] = positions[3 * p + 2];
      complex<double> density = (*orbital)(x);
      if (orbital->phase) {
        double a = arg(density);
        double r = 0.06;
        varyings[6 * p + 3] = r * cos(a);
        varyings[6 * p + 4] = r * sin(a);
      }
      varyings[6 * p + 5] = abs(density);
    }
    cloud->buffer(GL_ARRAY_BUFFER, varyings);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));

    GetGLError();

    setVerticesTetrahedra(int(num_points), int(num_tetrahedra));
  }

  bool detail_reduction = getReduction();

  GetGLError();

  static int next_set_of_timers = 0;
  static int shrink_times_two = 2;

  if (detail_reduction) {
    GLuint64 start, stop;
    GLint timer_good;
    long render_time;
    int old_shrink_times_two;

    glGetQueryObjectiv(timer_query[2 * next_set_of_timers],
                       GL_QUERY_RESULT_AVAILABLE, &timer_good);
    if (!timer_good)
      goto bailout;

    glGetQueryObjectiv(timer_query[2 * next_set_of_timers + 1],
                       GL_QUERY_RESULT_AVAILABLE, &timer_good);
    if (!timer_good)
      goto bailout;

    glGetQueryObjectui64v(timer_query[2 * next_set_of_timers],
                          GL_QUERY_RESULT, &start);
    glGetQueryObjectui64v(timer_query[2 * next_set_of_timers + 1],
                          GL_QUERY_RESULT, &stop);
    render_time = stop - start;

    old_shrink_times_two = shrink_times_two;
    if (render_time > 30 * 1000 * 1000)
      ++shrink_times_two;

    static int calls_since_shrink_reduction = 0;
    if (++calls_since_shrink_reduction >= 30) {
      --shrink_times_two;
      calls_since_shrink_reduction = 0;
    }

    clamp(shrink_times_two, 2, 20);

    if (shrink_times_two != old_shrink_times_two)
      setShrinkage(shrink_times_two / 2);

  bailout:
    GetGLError();
  }

  int shrink = shrink_times_two / 2;

  if (detail_reduction)
    glQueryCounter(timer_query[2 * next_set_of_timers], GL_TIMESTAMP);

  GetGLError();

  if (detail_reduction) {
    resizeTexture(cloudDensityTex, GL_RGB16F, GL_RGB,
                  width / shrink, height / shrink);
  } else {
    resizeTexture(cloudDensityTex, GL_RGB16F, GL_RGB, width, height);
  }

  GetGLError();

  struct timeval now_tv;
  gettimeofday(&now_tv, NULL);
  double now_sec = (now_tv.tv_sec % 6) + double(now_tv.tv_usec) / 1e6;
  now_sec *= 2.0 * pi / 6.0;

  // Generate the so-called model-view-projection matrix

  double rectangular_imbalance = sqrt(double(width) / double(height));
  double L = -rectangular_imbalance;
  double R =  rectangular_imbalance;
  double B = -1.0 / rectangular_imbalance;
  double T =  1.0 / rectangular_imbalance;
  double N =  1.0;
  double F =  getCameraRadius() + 100.0;
  Matrix<4,4> frustum = transformFrustum(L, R, B, T, N, F);

  Matrix<4,4> translation =
    transformTranslation(-getCameraRadius() * basisVector<3>(2));

  Matrix<4,4> rotation = transformRotation(getCameraRotation());

  Matrix<4,4> mvpm = frustum * translation * rotation;

  GetGLError();

  solidProg->use();
  solidProg->uniform<Matrix<4,4> >("modelViewProjMatrix") = mvpm;
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, solidFBO);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glLineWidth(2);
  solid->bind();
  glViewport(0, 0, width, height);
  glDrawArrays(GL_LINES, 0, 6);

  GetGLError();

  cloudProg->use();
  cloudProg->uniform<Matrix<4,4> >("modelViewProjMatrix") = mvpm;
  cloudProg->uniform<Vector<2> >("nearfar") = Vector2(N, F);
  cloudProg->uniform<int>("solidDepth") = 0;
  double b = pow(1.618, getBrightness());
  if (orbital->square)
    b *= b;
  cloudProg->uniform<float>("brightness") = b;
  glActiveTexture(GL_TEXTURE0);
  solidDepthTex->bind(GL_TEXTURE_2D);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cloudFBO);
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  cloud->bind();
  if (detail_reduction)
    glViewport(0, 0, width / shrink, height / shrink);
  else
    glViewport(0, 0, width, height);
  glDrawElements(GL_LINES_ADJACENCY, 4 * num_tetrahedra, GL_UNSIGNED_INT, 0);

  GetGLError();

  overlayProg->use();
  overlayProg->uniform<int>("solidData") = 0;
  overlayProg->uniform<int>("cloudData") = 1;
  Matrix<3,2> ct;
  ct(0,0) =  cos(now_sec);  ct(0,1) = sin(now_sec);
  ct(1,0) = -sin(now_sec);  ct(1,1) = cos(now_sec);
  ct(2,0) = 0.19784;        ct(2,1) = 0.46832;
  overlayProg->uniform<Matrix<3,2> >("color_trans") = ct;
  glActiveTexture(GL_TEXTURE0);
  solidRGBTex->bind(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE1);
  cloudDensityTex->bind(GL_TEXTURE_2D);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  rect->bind();
  glViewport(0, 0, width, height);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

  GetGLError();

  if (detail_reduction) {
    glQueryCounter(timer_query[2 * next_set_of_timers + 1], GL_TIMESTAMP);
    next_set_of_timers = 1 - next_set_of_timers;
  }

  GetGLError();
}

void cleanup()
{}
