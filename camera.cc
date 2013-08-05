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

#include "util.hh"
#include "vector.hh"
#include "matrix.hh"
#include "transform.hh"
#include "camera.hh"

Matrix<4,4> Camera::viewMatrix() const
{
  return transformTranslation(-radius * basisVector<3>(2)) *
    transformRotation(rotation);
}

// Rotate the camera around the origin
// 1.0 = 180 degrees of motion
void Camera::rotate(double x, double y)
{
  Quaternion xz_rotation = quaternionRotation(x * pi, basisVector<3>(1));
  Quaternion yz_rotation = quaternionRotation(y * pi, basisVector<3>(0));

  rotation = xz_rotation * yz_rotation * rotation;
  rotation /= norm(rotation);
}

// Spin the camera around the line of sight
// 1.0 = 180 degrees of rotation
void Camera::spin(double s)
{
  Quaternion xy_rotation = quaternionRotation(s * pi, basisVector<3>(2));

  rotation = xy_rotation * rotation;
  rotation /= norm(rotation);
}

// f < 0 --> zoom in
// f > 0 --> zoom out
void Camera::zoom(double f)
{
  radius *= pow(2.0, f);
  clamp(radius, 1.0, 2048.0);
}
