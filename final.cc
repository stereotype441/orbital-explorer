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

#include "oopengl.hh"
#include "shaders.hh"
#include "util.hh"
#include "controls.hh"

static Program *finalProg;
static VertexArrayObject *rect;
static Texture *solidRGBTex, *cloudDensityTex;

void initFinal(Texture *solidRGBTex_, Texture *cloudDensityTex_)
{
  solidRGBTex = solidRGBTex_;
  cloudDensityTex = cloudDensityTex_;
  finalProg = new Program();
  finalProg->vertexShader(finalVertexShaderSource);
  finalProg->fragmentShader(finalFragmentShaderSource);
  glBindAttribLocation(*finalProg, 0, "inPosition");
  glBindFragDataLocation(*finalProg, 0, "RGB");
  finalProg->link();

  GetGLError();

  // Big rectangle
  rect = new VertexArrayObject();
  rect->bind();

  // Positions
  FVector<2> rect_data[4];
  rect_data[0] = FVector2(-1.0, -1.0);
  rect_data[1] = FVector2(-1.0,  1.0);
  rect_data[2] = FVector2( 1.0,  1.0);
  rect_data[3] = FVector2( 1.0, -1.0);
  rect->buffer(GL_ARRAY_BUFFER, rect_data, sizeof(rect_data));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(rect_data[0]), NULL);

  GetGLError();
}

void drawFinal(int width, int height, double brightness)
{
  double this_instant = now();
  static double last_instant = -1.;
  if (last_instant < 0)
    last_instant = this_instant;
  static double color_cycle = 0.;
  color_cycle += (this_instant - last_instant) * double(getCycleRate());
  last_instant = this_instant;

  finalProg->use();
  finalProg->uniform<int>("solidData") = 0;
  finalProg->uniform<int>("cloudData") = 1;
  Matrix<3,2> ct;
  ct(0,0) =  cos(color_cycle);  ct(0,1) = sin(color_cycle);
  ct(1,0) = -sin(color_cycle);  ct(1,1) = cos(color_cycle);
  ct(2,0) = 0.19784;        ct(2,1) = 0.46832;
  finalProg->uniform<Matrix<3,2> >("color_trans") = ct;
  finalProg->uniform<int>("use_color") = getColorPhase();
  finalProg->uniform<float>("brightness") = brightness;
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, *solidRGBTex);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, *cloudDensityTex);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  rect->bind();
  glViewport(0, 0, width, height);
  glEnable(GL_FRAMEBUFFER_SRGB);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glDisable(GL_FRAMEBUFFER_SRGB);

  GetGLError();
}
