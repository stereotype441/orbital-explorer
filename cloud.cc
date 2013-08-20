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

#include <complex>
#include <vector>
#include <algorithm>

#include "oopengl.hh"
#include "cloud.hh"
#include "shaders.hh"

Cloud::Cloud(Texture *solidDepthTex_, Texture *cloudDensityTex)
{
  solidDepthTex = solidDepthTex_;
  cloudProg = new Program();
  cloudProg->vertexShader(cloudVertexShaderSource);
  cloudProg->geometryShader(cloudGeometryShaderSource);
  cloudProg->fragmentShader(cloudFragmentShaderSource);
  glBindAttribLocation(*cloudProg, 0, "position");
  glBindAttribLocation(*cloudProg, 1, "uY_vY_Y");
  glBindFragDataLocation(*cloudProg, 0, "integratedValue");
  cloudProg->link();

  GetGLError();

  glGenFramebuffers(1, &cloudFBO);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cloudFBO);
  attachTexture(cloudDensityTex, GL_RGBA16F, GL_RGB, GL_COLOR_ATTACHMENT0);
  checkFramebufferCompleteness();

  GetGLError();

  cloudVAO = new VertexArrayObject();

  GetGLError();

  // Camera should not actually start at this location
  old_camera_position = Vector<4>(0.);

  primitives_changed = false;
}

Cloud::StrippedTetra Cloud::strip_sort_key(const Tetra &t)
{
  StrippedTetra s;

  for (int i = 0; i < 4; ++i)
    s.vertex[i] = t.vertex[i];

  return s;
}

void Cloud::setPrimitives(const std::vector<Vector<3> > &pos,
                          const std::vector<unsigned> &ind,
                          const Orbital *orb)
{
  positions = pos;

  int num_tetrahedra = ind.size() / 4;
  indices.resize(num_tetrahedra);
  for (int i = 0; i < num_tetrahedra; ++i) {
    indices[i].sort_key = 0.0;
    for (int j = 0; j < 4; ++j)
      indices[i].vertex[j] = ind[4 * i + j];
  }
  orbital = orb;
  primitives_changed = true;
}

void Cloud::depthSortClouds(const Vector<4> &camera_position)
{
  // Set up tetrahedra for depth sort
  for (int i = 0; i < int(indices.size()); ++i) {
    Matrix<4,4> vertexMatrix;
    for (int col = 0; col < 4; ++col) {
      Vector<3> vert = positions[indices[i].vertex[col]];
      vertexMatrix(0, col) = vert[0];
      vertexMatrix(1, col) = vert[1];
      vertexMatrix(2, col) = vert[2];
      vertexMatrix(3, col) = 1.0;
    }
    Vector<4> vert_norm_sqr;
    for (int col = 0; col < 4; ++col)
      vert_norm_sqr[col] = norm_squared(positions[indices[i].vertex[col]]);
    indices[i].sort_key =
      dot_product(vert_norm_sqr, inverse(vertexMatrix) * camera_position);
  }

  std::sort(indices.begin(), indices.end());
}

void Cloud::uploadVertices()
{
  // Vertex varying data
  int num_points = positions.size();
  std::vector<Varying> varyings(num_points);
  for (int p = 0; p < num_points; ++p) {
    varyings[p].pos = FVector<3>(positions[p]);
    std::complex<double> density = (*orbital)(positions[p]);
    double s = 0.06;
    varyings[p].uvY = FVector3(s * density.real(), s * density.imag(),
                               abs(density));
  }

  cloudVAO->bind();
  cloudVAO->buffer(GL_ARRAY_BUFFER, varyings);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(varyings[0]),
                        reinterpret_cast<void *>(offsetof(Varying, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(varyings[0]),
                        reinterpret_cast<void *>(offsetof(Varying, uvY)));
  GetGLError();
}

void Cloud::uploadPrimitives()
{
  int num_tetrahedra = indices.size();

  std::vector<StrippedTetra> upload_indices(num_tetrahedra);
  for (int i = 0; i < num_tetrahedra; ++i)
    upload_indices[i] = strip_sort_key(indices[i]);

  cloudVAO->bind();
  cloudVAO->buffer(GL_ELEMENT_ARRAY_BUFFER, upload_indices);
  GetGLError();
}

void Cloud::draw(const Matrix<4,4> &mvpm, int width, int height,
                 double near, double far,
                 const Vector<4> &camera_position)
{
  int num_tetrahedra = indices.size();

  if (primitives_changed) {
    uploadVertices();
  }

  if (primitives_changed || camera_position != old_camera_position) {
    depthSortClouds(camera_position);
    uploadPrimitives();
  }

  primitives_changed = false;
  old_camera_position = camera_position;

  cloudProg->use();
  cloudProg->uniform<Matrix<4,4> >("modelViewProjMatrix") = mvpm;
  cloudProg->uniform<Vector<2> >("nearfar") = Vector2(near, far);
  cloudProg->uniform<int>("solidDepth") = 0;
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, *solidDepthTex);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, cloudFBO);
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_ONE, GL_ONE);
  cloudVAO->bind();
  glViewport(0, 0, width, height);
  glDrawElements(GL_LINES_ADJACENCY, 4 * num_tetrahedra, GL_UNSIGNED_INT, 0);

  GetGLError();
}
