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

#ifndef GENERICOPS_HH
#define GENERICOPS_HH

// Template madness.

// If V is a vector space over F, and V inherits from VectorSpace<F,V>,
// and defines the operations:
//   V += V
//   -V
//   V *= F
// Then the base class VectorSpace<F,V> will also define the following
// operations on class V:
//   +V
//   V + V
//   V -= V
//   V - V
//   V * F
//   F * V
//   V *= F
//   V /= F

// If, in addition, there is a natural way to construct a class instance
// from a scalar (e.g. if elements form an algebra with unit over F),
// and the class (which we now call A) inherits from Algebra<F,A>, and,
// in addition to the above vector operations, also defines the constructor:
//   explicit A(F)
// Then the base class Algebra<F,A> will also define the following
// operations on class A:
//   A = S
//   A += S
//   A + S
//   S + A
//   A -= S
//   A - S
//   S - A
// NOTE: The class must have a "using Algebra<F,A>::operator=" declaration

// If, in addition, A is commutative, and A inherits from
// CommutativeAlgebra<F,A>, and defines the operation:
//   A * A
// Then the base class CommutativeAlgebra<F,A> will also define the
// following operation on class A:
//   A *= A

template <typename F, class V>
class VectorSpace
{};

template <typename F, class A>
class Algebra : public VectorSpace<F,A>
{
public:
  const A &operator=(F x);
};

template <typename F, class A>
class CommutativeAlgebra : public Algebra<F,A>
{};

template <typename F, class V>
inline const V &operator+(const VectorSpace<F,V> &x)
{
  return static_cast<const V &>(x);
}

template <typename F, class V>
inline V operator+(const VectorSpace<F,V> &x, const VectorSpace<F,V> &y)
{
  V z(static_cast<const V &>(x));
  z += static_cast<const V &>(y);
  return z;
}

template <typename F, class V>
inline const V &operator-=(VectorSpace<F,V> &x, const VectorSpace<F,V> &y)
{
  static_cast<V &>(x) += -static_cast<const V &>(y);
  return static_cast<V &>(x);
}

template <typename F, class V>
inline V operator-(const VectorSpace<F,V> &x, const VectorSpace<F,V> &y)
{
  V z(static_cast<const V &>(x));
  z -= static_cast<const V &>(y);
  return z;
}

template <typename F, class V>
inline V operator*(const VectorSpace<F,V> &x, F y)
{
  V z(static_cast<const V &>(x));
  z *= y;
  return z;
}

template <typename F, class V>
inline V operator*(F x, const VectorSpace<F,V> &y)
{
  V z(static_cast<const V &>(y));
  z *= x;
  return z;
}

template <typename F, class V>
inline const V &operator/=(VectorSpace<F,V> &x, F y)
{
  static_cast<V &>(x) *= F(1.0) / y;
  return static_cast<V &>(x);
}

template <typename F, class V>
inline V operator/(const VectorSpace<F,V> &x, F y)
{
  V z(static_cast<const V &>(x));
  z *= F(1.0) / y;
  return z;
}

template <typename F, class A>
inline const A &Algebra<F,A>::operator=(F x)
{
  *static_cast<A *>(this) = A(x);
  return *static_cast<A *>(this);
}

template <typename F, class A>
inline const A &operator+=(Algebra<F,A> &x, F y)
{
  static_cast<A &>(x) += A(y);
  return static_cast<A &>(x);
}

template <typename F, class A>
inline A operator+(const Algebra<F,A> &x, F y)
{
  A z(static_cast<const A &>(x));
  z += A(y);
  return z;
}

template <typename F, class A>
inline A operator+(F x, const Algebra<F,A> &y)
{
  A z(static_cast<const A &>(y));
  z += A(x);
  return z;
}

template <typename F, class A>
inline const A &operator-=(Algebra<F,A> &x, F y)
{
  static_cast<A &>(x) -= A(y);
  return static_cast<A &>(x);
}

template <typename F, class A>
inline A operator-(const Algebra<F,A> &x, F y)
{
  A z(static_cast<const A &>(x));
  z -= A(y);
  return z;
}

template <typename F, class A>
inline A operator-(F x, const Algebra<F,A> &y)
{
  A z(-static_cast<const A &>(y));
  z += A(x);
  return z;
}

template <typename F, class A>
inline const A &operator*=(CommutativeAlgebra<F,A> &x,
                           const CommutativeAlgebra<F,A> &y)
{
  static_cast<A &>(x) = static_cast<A &>(x) * static_cast<const A &>(y);
  return static_cast<A &>(x);
}

#endif
