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

#ifndef VECTOR_HH
#define VECTOR_HH

#include <complex>

#include "genericops.hh"
#include "array.hh"

template <unsigned n, typename T, class V>
class GenericVector : public Array<n,T>, public VectorSpace<T,V>
{
protected:
  GenericVector() {}
  explicit GenericVector(T x) : Array<n,T>(x) {}

public:
  // Allow all elements of a vector to be set with assignment
  const V &operator=(T x)
  {
    for (unsigned i=0; i<n; ++i)
      (*this)[i] = x;
    return *static_cast<V *>(this);
  }
};

template <unsigned n>
class Vector : public GenericVector<n, double, Vector<n> >
{
public:
  Vector() {}
  explicit Vector(double x) : GenericVector<n, double, Vector<n> >(x) {}

  using GenericVector<n, double, Vector<n> >::operator=;
};

template <unsigned n>
class CVector : public GenericVector<n, std::complex<double>, CVector<n> >
{
public:
  CVector() {}
  explicit CVector(std::complex<double> x) :
    GenericVector<n, std::complex<double>, CVector<n> >(x) {}

  using GenericVector<n, std::complex<double>, CVector<n> >::operator=;
};

template <class V>
inline V genericBasisVector(unsigned i)
{
  V z(0.0);

  z[i] = 1.0;

  return z;
}

template <unsigned n>
inline Vector<n> basisVector(unsigned i)
{
  return genericBasisVector<Vector<n> >(i);
}

template <unsigned n>
inline CVector<n> cBasisVector(unsigned i)
{
  return genericBasisVector<CVector<n> >(i);
}

template <unsigned n, typename T, class V>
inline const V &operator+=(GenericVector<n,T,V> &x,
                           const GenericVector<n,T,V> &y)
{
  for (unsigned i = 0; i < n; ++i)
    x[i] += y[i];

  return static_cast<V &>(x);
}

template <unsigned n, typename T, class V>
inline V operator-(const GenericVector<n,T,V> &x)
{
  V z;

  for (unsigned i=0; i<n; ++i)
    z[i] = -x[i];

  return z;
}

template <unsigned n, typename T, class V>
inline const V &operator*=(GenericVector<n,T,V> &x, T y)
{
  for (unsigned i = 0; i < n; ++i)
    x[i] *= y;

  return static_cast<V &>(x);
}

template <unsigned n>
inline double dot_product(const Vector<n> &x, const Vector<n> &y)
{
  double z = 0.0;

  for (unsigned i=0; i<n; ++i)
    z += x[i] * y[i];

  return z;
}

template <unsigned n>
inline std::complex<double> dot_product(const CVector<n> &x,
                                        const CVector<n> &y)
{
  std::complex<double> z = 0.0;

  for (unsigned i=0; i<n; ++i)
    z += x[i] * conj(y[i]);

  return z;
}

inline Vector<3> cross_product(const Vector<3> &x, const Vector<3> &y)
{
  Vector<3> z;
  z[0] = x[1] * y[2] - x[2] * y[1];
  z[1] = x[2] * y[0] - x[0] * y[2];
  z[2] = x[0] * y[1] - x[1] * y[0];
  return z;
}

template <unsigned n, typename T, class V>
inline T norm_squared(const GenericVector<n,T,V> &x)
{
  const V &vx = static_cast<const V &>(x);
  return dot_product(vx,vx);
}

template <unsigned n, typename T, class V>
inline T norm(const GenericVector<n,T,V> &x)
{
  return std::sqrt(norm_squared(x));
}

// Constructors for specific small sizes

inline Vector<2> Vector2(double x, double y)
{
  Vector<2> a;
  a[0] = x;
  a[1] = y;
  return a;
}

inline Vector<3> Vector3(double x, double y, double z)
{
  Vector<3> a;
  a[0] = x;
  a[1] = y;
  a[2] = z;
  return a;
}

#endif
