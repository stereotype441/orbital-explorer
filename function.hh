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

#ifndef FUNCTION_HH
#define FUNCTION_HH

#include "matrix.hh"

template <unsigned n, typename T>
class Function
{
public:
  virtual T operator()(const Vector<n> &) const = 0;
  virtual ~Function() {}
};


// There isn't any mathematical problem with computing numerical
// gradients and hessians of a T-valued function; the problem is
// that our Vector and Matrix classes are only defined for doubles.
// So, for now, this code will sit in the mothballs of an unused
// subclass.

template <unsigned n>
class RealFunction : public Function<n,double>
{
public:
  virtual Vector<n> gradient(const Vector<n> &) const;
  virtual Matrix<n,n> hessian(const Vector<n> &) const;
};

template <unsigned n>
Vector<n> RealFunction<n>::gradient(const Vector<n> &x) const
{
  const RealFunction<n> &f = *this;

  Vector<n> grad;
  const double h = 0.0001;
  Vector<n> dx(0);

  for (unsigned i=0; i<n; ++i) {
    dx[i] = h;
    grad[i] = (f(x+dx) - f(x-dx)) / (2.0*h);
    dx[i] = 0.0;
  }

  return grad;
}

template <unsigned n>
Matrix<n,n> RealFunction<n>::hessian(const Vector<n> &x) const
{
  const RealFunction<n> &f = *this;

  Matrix<n,n> hess;
  const double h = 0.0001;
  Vector<n> d1(0.0), d2(0.0);

  for (unsigned i=0; i<n; ++i) {
    d1[i] = h;
    for (unsigned j=i; j<n; ++j) {
      d2[j] = h;
      hess(i,j) = hess(j,i) =
        (f(x+d1+d2) - f(x+d1-d2) - f(x-d1+d2) + f(x-d1-d2)) / (4.0*h*h);
      d2[j] = 0.0;
    }
    d1[i] = 0.0;
  }

  return hess;
}

#endif
