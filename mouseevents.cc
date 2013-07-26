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
#include <map>
#include <complex>
#include <cmath>

#include "util.hh"
#include "genericops.hh"
#include "array.hh"
#include "vector.hh"
#include "matrix.hh"
#include "quaternion.hh"
#include "transform.hh"

using namespace std;

static int width = 0;
static int height = 0;

int getWidth()
{
  return width;
}

int getHeight()
{
  return height;
}

void resize(int w, int h)
{
  width = w;
  height = h;
}

static Quaternion cameraRotation(1.0);

Quaternion getCameraRotation()
{
  return cameraRotation;
}

static double cameraRadius = 32.;

double getCameraRadius()
{
  return cameraRadius;
}

void mouse_drag_left(int movex, int movey)
{
  double dx = double(movex) / double(width);
  double dy = double(movey) / double(height);

  Quaternion xz_rotation = quaternionRotation(dx * pi, basisVector<3>(1));
  Quaternion yz_rotation = quaternionRotation(dy * pi, basisVector<3>(0));

  cameraRotation = xz_rotation * yz_rotation * cameraRotation;
  cameraRotation /= norm(cameraRotation);
}

void mouse_drag_right(int movex, int movey)
{
  double dx = double(movex) / double(width);
  double dy = double(movey) / double(height);

  Quaternion xy_rotation = quaternionRotation(-dx * pi, basisVector<3>(2));

  cameraRotation = xy_rotation * cameraRotation;
  cameraRotation /= norm(cameraRotation);

  double factor = 1 + dy;
  clamp(factor, 0.5, 2.0);

  cameraRadius *= factor;
  clamp(cameraRadius, 1.0, 2048.0);
}

void mouse_wheel(int direction)
{
  // + = zoom in, - = zoom out
  cameraRadius *= pow(2.0, 0.0625 * double(-direction));
  clamp(cameraRadius, 1.0, 2048.0);
}
