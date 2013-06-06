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
#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "function.hh"
#include "polynomial.hh"
#include "wavefunction.hh"

using namespace std;

Orbital::Orbital(int Z_, int N_, int L_, int M_,
                 bool real_, bool diff_, bool square_, bool phase_) :
  Z(Z_), N(N_), L(L_), M(M_),
  real(real_), diff(diff_), square(square_), phase(phase_)
{
  c = 1.0;
  c *= pow(2.0 * double(Z) / double(N), 1.5);
  c *= sqrt(factorial(N - L - 1) / (2.0 * double(N) * factorial(N + L)));
  c *= sqrt((2.0 * double(L) + 1.0) / (4.0 * pi));
  c *= sqrt(factorial(L - fabs(M)) / factorial(L + fabs(M)));
  c /= factorial(L);
  c *= pow(double(Z) / double(N), L);

  expc = -double(Z) / double(N);

  rp = 0.0;
  Polynomial r(1.0, 1);
  for (int k = 0; k < N - L; ++k) {
    Polynomial m(1.0);
    if (k & 1) m = -1.0;
    m /= factorial(k);
    m *= choose(N + L, N - L - 1 - k);
    m *= pow(2.0 * double(Z) * r / double(N), k);
    rp += m;
  }

  Polynomial x(1.0, 1);
  Polynomial temp = pow(x * x - 1.0, L);
  ctp = temp.derivative(L + abs(M));
}

std::complex<double> Orbital::operator()(const Vector<3> &x) const
{
  double x2y2 = x[0] * x[0] + x[1] * x[1];
  double r_xy = sqrt(x2y2);
  double r = sqrt(x2y2 + x[2] * x[2]);

  double sin_theta, cos_theta;
  if (r > 0.0) {
    sin_theta = r_xy / r;
    cos_theta = x[2] / r;
  } else {
    sin_theta = 0.0;
    cos_theta = 1.0;
  }

  double phi;
  if (r_xy > 0.0)
    phi = atan2(x[1], x[0]);
  else
    phi = 0.0;

  double val = c;
  val *= exp(expc * r);
  val *= ipow(r, L);
  val *= rp(r);
  val *= ipow(sin_theta, abs(M));
  val *= ctp(cos_theta);

  std::complex<double> angle;
  if (real && M != 0) {
    if (diff) {
      angle = std::complex<double>(0.0, 1.0);
      val *= sin(M * phi);
    } else {
      angle = std::complex<double>(1.0, 0.0);
      val *= cos(M * phi);
    }
    val *= sqrt(2);
  }
  else
    angle = std::complex<double>(cos(M * phi), sin(M * phi));

  if (val < 0.0) {
    angle = -angle;
    val = -val;
  }

  if (square)
    val *= val;

  if (phase)
    return val * angle;
  else
    return val;
}

bool Orbital::operator==(const Orbital &rhs)
{
  return Z == rhs.Z && N == rhs.N && L == rhs.L && M == rhs.M
    && real == rhs.real && diff == rhs.diff && square == rhs.square
    && phase == rhs.phase;
}
