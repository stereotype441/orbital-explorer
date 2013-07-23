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

#ifndef QUATERNION_HH
#define QUATERNION_HH

#include <cmath>

#include "genericops.hh"

class Quaternion : public Algebra<double, Quaternion>
{
public:
  using Algebra<double, Quaternion>::operator=;
  Quaternion() {}
  explicit Quaternion(double x) : r(x), i(0.), j(0.), k(0.) {}
  Quaternion(double w, double x, double y, double z) : r(w), i(x), j(y), k(z) {}
  double &real() { return r; }
  double &imag() { return i; }
  double &jmag() { return j; }
  double &kmag() { return k; }
  const double &real() const { return r; }
  const double &imag() const { return i; }
  const double &jmag() const { return j; }
  const double &kmag() const { return k; }

private:
  double r, i, j, k;
};

inline const Quaternion &operator+=(Quaternion &x, const Quaternion &y)
{
  x.real() += y.real();
  x.imag() += y.imag();
  x.jmag() += y.jmag();
  x.kmag() += y.kmag();
  return x;
}

inline Quaternion operator-(const Quaternion &x)
{
  return Quaternion(-x.real(), -x.imag(), -x.jmag(), -x.kmag());
}

inline const Quaternion &operator*=(Quaternion &x, double y)
{
  x.real() *= y;
  x.imag() *= y;
  x.jmag() *= y;
  x.kmag() *= y;
  return x;
}

inline Quaternion operator*(const Quaternion &x, const Quaternion &y)
{
  return Quaternion(+ x.real() * y.real() - x.imag() * y.imag()
                    - x.jmag() * y.jmag() - x.kmag() * y.kmag(),
                    + x.real() * y.imag() + x.imag() * y.real()
                    + x.jmag() * y.kmag() - x.kmag() * y.jmag(),
                    + x.real() * y.jmag() - x.imag() * y.kmag()
                    + x.jmag() * y.real() + x.kmag() * y.imag(),
                    + x.real() * y.kmag() + x.imag() * y.jmag()
                    - x.jmag() * y.imag() + x.kmag() * y.real());
}

inline double norm_squared(const Quaternion &q)
{
  return q.real() * q.real() + q.imag() * q.imag() +
    q.jmag() * q.jmag() + q.kmag() * q.kmag();
}

inline double norm(const Quaternion &q)
{
  return std::sqrt(norm_squared(q));
}

#endif
