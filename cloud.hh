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

#ifndef CLOUD_HH
#define CLOUD_HH

#include "oopengl.hh"
#include "matrix.hh"
#include "wavefunction.hh"

class Cloud
{
public:
  Cloud(Texture *solidDepthTex, Texture *cloudDensityTex);
  void setPrimitives(const std::vector<Vector<3> > &positions,
                     const std::vector<unsigned> &indices,
                     const Orbital *orbital);
  void draw(const Matrix<4,4> &mvpm, int width, int height,
            double near, double far,
            const Vector<4> &camera_position);

private:
  struct Tetra
  {
    double sort_key;
    unsigned vertex[4];
    bool operator<(const struct Tetra &rhs) const
    {
      return sort_key < rhs.sort_key;
    }
  };

  struct StrippedTetra
  {
    unsigned vertex[4];
  };

  struct Varying
  {
    FVector<3> pos;
    FVector<3> rim;
  };

  void uploadVertices();
  void uploadPrimitives();
  StrippedTetra strip_sort_key(const Tetra &t);
  void depthSortClouds(const Vector<4> &camera_position);

  Program *cloudProg;
  Texture *solidDepthTex;
  GLuint cloudFBO;
  VertexArrayObject *cloudVAO;
  Vector<4> old_camera_position;
  std::vector<Vector<3> > positions;
  std::vector<Tetra> indices;
  const Orbital *orbital;
  bool primitives_changed;
};

#endif
