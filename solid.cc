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

#include "solid.hh"
#include "shaders.hh"

static Program *solidProg;
static VertexArrayObject *solid;
static GLuint solidFBO;

struct SolidVarying
{
  FVector<3> pos;
  FVector<3> xyY; // Emitted CIE 1931 color
};

void initSolids(Texture *solidRGBTex, Texture *solidDepthTex)
{
  // Solid program

  solidProg = new Program();
  solidProg->vertexShader(solidVertexShaderSource);
  solidProg->fragmentShader(solidFragmentShaderSource);
  glBindAttribLocation(*solidProg, 0, "inPosition");
  glBindAttribLocation(*solidProg, 1, "inColor");
  glBindFragDataLocation(*solidProg, 0, "fragColor");
  solidProg->link();

  GetGLError();

  // Solid objects

  solid = new VertexArrayObject();
  solid->bind();

  SolidVarying solid_data[6];
  FVector<3> origin(FVector3(0.0, 0.0, 0.0));
  FVector<3> red_xyY  (FVector3(0.6400, 0.3300, 0.2126));
  FVector<3> green_xyY(FVector3(0.3000, 0.6000, 0.3290));
  FVector<3> blue_xyY (FVector3(0.1500, 0.0600, 0.0721));

  // X axis
  solid_data[0].pos = origin;
  solid_data[0].xyY = red_xyY;
  solid_data[1].pos = FVector3(1.0, 0.0, 0.0);
  solid_data[1].xyY = red_xyY;

    // Y axis
  solid_data[2].pos = origin;
  solid_data[2].xyY = green_xyY;
  solid_data[3].pos = FVector3(0.0, 1.0, 0.0);
  solid_data[3].xyY = green_xyY;

    // Z axis
  solid_data[4].pos = origin;
  solid_data[4].xyY = blue_xyY;
  solid_data[5].pos = FVector3(0.0, 0.0, 1.0);
  solid_data[5].xyY = blue_xyY;

  solid->buffer(GL_ARRAY_BUFFER, solid_data, sizeof(solid_data));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SolidVarying),
                        reinterpret_cast<void *>(offsetof(SolidVarying, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SolidVarying),
                        reinterpret_cast<void *>(offsetof(SolidVarying, xyY)));

  GetGLError();

  // Solid Framebuffer Object

  glGenFramebuffers(1, &solidFBO);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, solidFBO);
  attachTexture(solidRGBTex, GL_RGB8, GL_RGB, GL_COLOR_ATTACHMENT0);
  attachTexture(solidDepthTex, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                GL_DEPTH_ATTACHMENT);
  checkFramebufferCompleteness();
}

void drawSolids(const Matrix<4,4> &mvpm, int width, int height)
{
  solidProg->use();
  solidProg->uniform<Matrix<4,4> >("modelViewProjMatrix") = mvpm;
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, solidFBO);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  // Note: glLineWidth must be no greater than 1.0 in OpenGL 3.2 Core.
  solid->bind();
  glViewport(0, 0, width, height);
  glDrawArrays(GL_LINES, 0, 6);

  GetGLError();
}
