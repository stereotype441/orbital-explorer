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
  // Set up radial part of wave function

  // Leading constant: (2Z/N)^1.5 sqrt((N-L-1)! / 2N(N+L)!)
  radial_constant = pow(2.0 * double(Z) / double(N), 1.5) *
    sqrt(factorial(N - L - 1) /
         (2.0 * double(N) * factorial(N + L)));
  // e^(-Zr/N)
  radial_exponential_constant = -double(Z) / double(N);
  // (2Zr/N)^L
  radial_constant *= pow(2.0 * double(Z) / double(N), L);
  // Associated Laguerre polynomial L_(N-L-1)^(2L+1) (2Zr/N)
  radial_polynomial = 0.0;
  Polynomial r(1.0, 1);
  for (int k = 0; k <= N - L - 1; ++k) {
    radial_polynomial += ((k & 1) ? -1.0 : 1.0) *
      choose(N + L, double(N - L - 1 - k)) *
      pow(2.0 * double(Z) * r / double(N), k) /
      factorial(k);
  }

  // Set up angular part of wave function

  // Leading constant: sqrt((2L+1)(L-|M|)! / 4pi(L+|M|)!)
  angular_constant = sqrt((2.0 * double(L) + 1.0) * factorial(L - fabs(M)) /
                          (4.0 * pi * factorial(L + fabs(M))));
  // Other constants: 1 / 2^L L!
  angular_constant /= pow(2.0, L) * factorial(L);
  // Polynomial in cos(theta) comes from associated Legendre polynomial
  Polynomial x(1.0, 1);
  cos_theta_polynomial = pow(x * x - 1.0, L).derivative(L + abs(M));
}

double Orbital::radial_part(double r) const
{
  return radial_constant *
    exp(radial_exponential_constant * r) *
    ipow(r, L) *
    radial_polynomial(r);
}

double Orbital::theta_part(double sin_theta, double cos_theta) const
{
  return angular_constant *
    cos_theta_polynomial(cos_theta) *
    ipow(sin_theta, abs(M));
}

std::complex<double> Orbital::operator()(const Vector<3> &x) const
{
  double x2y2 = x[0] * x[0] + x[1] * x[1];

  double r_xy = sqrt(x2y2);
  double phi;
  if (r_xy > 0.0)
    phi = atan2(x[1], x[0]);
  else
    phi = 0.0;

  double r = sqrt(x2y2 + x[2] * x[2]);

  double sin_theta, cos_theta;
  if (r > 0.0) {
    sin_theta = r_xy / r;
    cos_theta = x[2] / r;
  } else {
    sin_theta = 0.0;
    cos_theta = 1.0;
  }

  // val is independent of the sign of M
  double val = radial_part(r) * theta_part(sin_theta, cos_theta);

  // If the sign of M is flipped, angle is negated, so the result
  // is complex conjugation of the wave function's value.
  double angle = M * phi;

  // An additional sign change is present if M is positive and odd, which
  // makes for a fine mess.
  double sign = 1.0;
  if (M > 0 && (M & 1))
    sign = -1.0;

  double result_mag;
  complex<double> result_arg;
  // If real is set, we need to add or subtract what the result would
  // have been with M negated, and divide by sqrt(2) or i sqrt(2).
  // (Assumes M is positive or zero. If M is zero the result is already
  // real.)
  if (real && M != 0) {
    double factor = diff ? -1.0 : 1.0;
    // sign * val * exp(i * angle) + factor * val * exp(-i * angle)
    // = val * (sign * cos(angle) + sign * i * sin(angle)
    //          factor * cos(angle) - factor * i * sin(angle))
    // = val * ((sign + factor) cos(angle) + (sign - factor) i sin(angle))
    result_mag = val * ((sign + factor) * cos(angle) +
                        (sign - factor) * sin(angle)) / sqrt(2.0);
    result_arg = complex<double>(1.0, 0.0);
  } else {
    result_mag = sign * val;
    result_arg = complex<double>(cos(angle), sin(angle));
  }

  if (result_mag < 0.0) {
    result_mag = -result_mag;
    result_arg = -result_arg;
  }

  if (square)
    result_mag *= result_mag;

  if (phase)
    return result_mag * result_arg;
  else
    return result_mag;
}

bool Orbital::operator==(const Orbital &rhs)
{
  return Z == rhs.Z && N == rhs.N && L == rhs.L && M == rhs.M
    && real == rhs.real && diff == rhs.diff && square == rhs.square
    && phase == rhs.phase;
}

double Orbital::radius() const
{
  return 10.0;
}
