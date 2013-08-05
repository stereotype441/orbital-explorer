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

#include <cmath>

#include "vector.hh"
#include "matrix.hh"
#include "quaternion.hh"

Matrix<4,4> transformTranslation(const Vector<3> &v)
{
  Matrix<4,4> m(1.0);

  m(0,3) = v[0];
  m(1,3) = v[1];
  m(2,3) = v[2];

  return m;
}

Matrix<4,4> transformRotation(const Quaternion &q)
{
  double a = q.real();
  double b = q.imag();
  double c = q.jmag();
  double d = q.kmag();

  Matrix<4,4> m(1.0);

  m(0,0) = a*a + b*b - c*c - d*d;
  m(0,1) = -2*a*d + 2*b*c;
  m(0,2) = 2*a*c + 2*b*d;
  m(1,0) = 2*a*d + 2*b*c;
  m(1,1) = a*a - b*b + c*c - d*d;
  m(1,2) = -2*a*b + 2*c*d;
  m(2,0) = -2*a*c + 2*b*d;
  m(2,1) = 2*a*b + 2*c*d;
  m(2,2) = a*a - b*b - c*c + d*d;

  return m;
}

Quaternion quaternionRotation(double t, Vector<3> v)
{
  v /= norm(v);

  double s = sin(t / 2.0);
  double c = cos(t / 2.0);

  return Quaternion(c, s * v[0], s * v[1], s * v[2]);
}

Matrix<4,4> transformFrustum(double L, double R,
                             double B, double T,
                             double N, double F)
{
  Matrix<4,4> m(0.0);

  m(0,0) =  2.0 * N / (R - L);
  m(0,2) =  (R + L) / (R - L);
  m(1,1) =  2.0 * N / (T - B);
  m(1,2) =  (T + B) / (T - B);
  m(2,2) = -(F + N) / (F - N);
  m(2,3) = -2.0 * F * N / (F - N);
  m(3,2) = -1.0;

  return m;
}
