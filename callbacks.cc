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

#include "glprocs.hh"
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
#include "viewport.hh"
#include "camera.hh"
#include "controls.hh"
#include "solid.hh"
#include "cloud.hh"
#include "final.hh"

using namespace std;

// Vertex array objects
static VertexArrayObject *cloud;

// Textures
static Texture *solidRGBTex, *solidDepthTex, *cloudDensityTex;

// This records the number of primitives, not the number of indices
static unsigned num_tetrahedra;

// The function to visualize
static Orbital *orbital = NULL;

// Subdivision of space into tetrahedra
static TetrahedralSubdivision *ts = NULL;

void resizeTexture(Texture *name, GLint internalformat, GLenum format,
                   GLuint width, GLuint height)
{
  glBindTexture(GL_TEXTURE_2D, *name);
  glTexImage2D(GL_TEXTURE_2D, 0, internalformat, width, height, 0,
               format, GL_BYTE, NULL);
}

void initialize()
{
  solidRGBTex = new Texture();
  solidDepthTex = new Texture();
  cloudDensityTex = new Texture();

  initSolids(solidRGBTex, solidDepthTex);
  initClouds(solidDepthTex, cloudDensityTex);
  initFinal(solidRGBTex, cloudDensityTex);

  // Clouds
  cloud = new VertexArrayObject();
  GetGLError();

  glClearColor(0., 0., 0., 0.);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  GetGLError();
}

void resizeTextures(Viewport &view)
{
  int width = view.getWidth();
  int height = view.getHeight();

  // Resize statically-sized textures
  resizeTexture(solidRGBTex, GL_RGB8, GL_RGB, width, height);
  resizeTexture(solidDepthTex, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                width, height);
  resizeTexture(cloudDensityTex, GL_RGBA16F, GL_RGB, width, height);

  GetGLError();
}

Matrix<4,4> generateMvpm(Camera &camera, int width, int height, double near, double far)
{
  // Generate the so-called model-view-projection matrix

  double rectangular_imbalance = sqrt(double(width) / double(height));
  double L = -rectangular_imbalance;
  double R =  rectangular_imbalance;
  double B = -1.0 / rectangular_imbalance;
  double T =  1.0 / rectangular_imbalance;
  Matrix<4,4> frustum = transformFrustum(L, R, B, T, near, far);

  Matrix<4,4> translation =
    transformTranslation(-camera.getRadius() * basisVector<3>(2));

  Matrix<4,4> rotation = transformRotation(camera.getRotation());

  return frustum * translation * rotation;
}

void display(Viewport &view, Camera &camera)
{
  static bool need_full_redraw = true;

  int width = view.getWidth();
  int height = view.getHeight();

  static int num_points = 0;
  bool just_started = false;

  // Are we just starting up, or did the wave function change?
  Orbital newOrbital = getOrbital();
  // Did the detail level change?
  static int saved_detail = 0;
  int detail = getDetail();
  if (!orbital || *orbital != newOrbital || saved_detail != detail) {
    saved_detail = detail;

    // Stop any running thread
    if (ts)
      ts->kill();
    // delete checks for NULL, so we don't have to
    delete orbital;
    delete ts;
    orbital = new Orbital(newOrbital);
    ts = new TetrahedralSubdivision(*orbital, orbital->radius());
    num_points = 0;

    // Golden ratio
    const double phi = (1.0 + sqrt(5.0)) / 2.0;
    // 500, 800, 1300, 2100, 3400, 5500, 8900, 14400, 23300, 37700
    int v = 100 * int(pow(phi, double(detail) + 4.0) / sqrt(5.0) + 0.5);
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
      double a = arg(density);
      double r = 0.06;
      varyings[6 * p + 3] = r * cos(a);
      varyings[6 * p + 4] = r * sin(a);
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

    need_full_redraw = true;
  }

  GetGLError();

  double near = 1.0;
  double far = camera.getRadius() + orbital->radius() * sqrt(2.0);
  Matrix<4,4> mvpm = generateMvpm(camera, width, height, near, far);

  static Matrix<4,4> old_mvpm;
  if (mvpm != old_mvpm)
    need_full_redraw = true;
  old_mvpm = mvpm;
  static int old_width = 0;
  static int old_height = 0;
  if (width != old_width || height != old_height)
    need_full_redraw = true;
  old_width = width;
  old_height = height;

  GetGLError();

  if (need_full_redraw) {
    drawSolids(mvpm, width, height);
    drawClouds(mvpm, width, height, near, far,
               cloud, num_tetrahedra);
    need_full_redraw = false;
  }
  double brightness = pow(1.618, getBrightness());
  if (orbital->square)
    brightness *= brightness;
  drawFinal(width, height, brightness);

  glFinish();

  GetGLError();
}

void cleanup()
{}
