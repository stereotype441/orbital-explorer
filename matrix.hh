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

template <unsigned p, unsigned q, typename T, class M>
class GenericMatrix : public Array<p*q,T>, public Algebra<T,M>
{
public:
  using Algebra<T,M>::operator=;
  GenericMatrix() {}
  // Note: scalar matrix constructor; off-diagonal elements are zero
  explicit GenericMatrix(T x) : Array<p*q,T>(0.0) { set_diag(x); }

  T &operator()(unsigned i, unsigned j)
  {
    check_indices(i, j);
    return this->unsafe_element(i*q+j);
  }
  const T &operator()(unsigned i, unsigned j) const
  {
    check_indices(i, j);
    return this->unsafe_element(i*q+j);
  }

private:
  // Hide Array's operator[] to avoid programming errors
  T &operator[](unsigned); // Do not define
  const T &operator[](unsigned) const; // Do not define

  void set_diag(T t)
  {
    for (unsigned i=0; i<std::min(p,q); ++i)
      (*this)(i,i) = t;
  }

  void check_indices(unsigned r, unsigned c) const
  {
    if (r >= p || c >= q)
      throw_matrix_range_exception();
  }
  void throw_matrix_range_exception() const;
};

template <unsigned p, unsigned q, typename T, class M>
void GenericMatrix<p,q,T,M>::throw_matrix_range_exception() const
{
  throw std::range_error("Matrix access out of range");
}

template <unsigned p, unsigned q>
class Matrix : public GenericMatrix<p, q, double, Matrix<p,q> >
{
public:
  Matrix() {}
  explicit Matrix(double x) : GenericMatrix<p, q, double, Matrix<p,q> >(x) {}

  using GenericMatrix<p, q, double, Matrix<p,q> >::operator=;
  typedef Matrix<q,p> TransposeType;
};

template <unsigned p, unsigned q>
class CMatrix : public GenericMatrix<p, q, std::complex<double>, CMatrix<p,q> >
{
public:
  CMatrix() {}
  explicit CMatrix(std::complex<double> x) :
    GenericMatrix<p, q, std::complex<double>, CMatrix<p,q> >(x) {}

  using GenericMatrix<p, q, std::complex<double>, CMatrix<p,q> >::operator=;
  typedef CMatrix<q,p> TransposeType;
};

template <unsigned p, unsigned q, typename T, class M>
inline const M &operator+=(GenericMatrix<p,q,T,M> &x,
                           const GenericMatrix<p,q,T,M> &y)
{
  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      x(i,j) += y(i,j);

  return static_cast<const M &>(x);
}

template <unsigned p, unsigned q, typename T, class M>
inline M operator-(const GenericMatrix<p,q,T,M> &x)
{
  M z;

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      z(i,j) = -x(i,j);

  return z;
}

template <unsigned p, unsigned q, typename T, class M>
inline const M &operator*=(GenericMatrix<p,q,T,M> &x, T y)
{
  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      x(i,j) *= y;

  return static_cast<const M &>(x);
}

// Sadly, there is no easy way to deduce the derived type of the
// result from the base and derived types of the operands. So we
// need to implement separate matrix-vector and matrix-matrix
// multiplication functions for the real and complex cases.
template <unsigned p, unsigned q>
inline Vector<p> operator*(const Matrix<p,q> &x, const Vector<q> &y)
{
  Vector<p> z(0.);

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      z[i] += x(i,j) * y[j];

  return z;
}

template <unsigned p, unsigned q>
inline CVector<p> operator*(const CMatrix<p,q> &x, const CVector<q> &y)
{
  CVector<p> z(0.);

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      z[i] += x(i,j) * y[j];

  return z;
}

template <unsigned p, unsigned q, unsigned r>
inline Matrix<p,r> operator*(const Matrix<p,q> &x, const Matrix<q,r> &y)
{
  Matrix<p,r> z(0.);

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      for (unsigned k=0; k<r; ++k)
        z(i,k) += x(i,j) * y(j,k);

  return z;
}

template <unsigned p, unsigned q, unsigned r>
inline CMatrix<p,r> operator*(const CMatrix<p,q> &x, const CMatrix<q,r> &y)
{
  CMatrix<p,r> z(0.);

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      for (unsigned k=0; k<r; ++k)
        z(i,k) += x(i,j) * y(j,k);

  return z;
}

template <unsigned p, unsigned q, typename T, class M>
typename M::TransposeType transpose(const GenericMatrix<p,q,T,M> &x)
{
  typename M::TransposeType t;

  for (unsigned i=0; i<p; ++i)
    for (unsigned j=0; j<q; ++j)
      t(j,i) = x(i,j);

  return t;
}

template <unsigned n, typename T, class M>
M inverse(const GenericMatrix<n,n,T,M> &x)
{
  M y(static_cast<const M &>(x));
  M z(T(1.0));

  // Do row operations simultaneously to y and z
  // When y is transformed into the identity, z will become x^-1
  for (unsigned i=0; i<n; ++i) {
    // Swap row i with the row >= i with the largest entry in column i
    unsigned biggest = i;
    double biggest_value = std::abs(y(i,i));
    for (unsigned j=i+1; j<n; ++j)
      if (std::abs(y(j,i)) > biggest_value) {
        biggest_value = std::abs(y(j,i));
        biggest = j;
      }
    if (biggest != i)
      for (unsigned j=0; j<n; ++j) {
        T t = y(i,j);
        y(i,j) = y(biggest,j);
        y(biggest,j) = t;
        t = z(i,j);
        z(i,j) = z(biggest,j);
        z(biggest,j) = t;
      }

    // Divide row i by entry (i,i)
    T u = y(i,i);
    if (std::abs(u) < 1e-9)
      throw std::logic_error("Ill-conditioned matrix in inverse");
    for (unsigned j=0; j<n; ++j) {
      y(i,j) /= u;
      z(i,j) /= u;
    }
    y(i,i) = T(1.0);

    // Subtract row i * entry (j,i) from row j, for all j != i
    for (unsigned j=0; j<n; ++j)
      if (j != i) {
        T v = y(j,i);
        for (unsigned k=0; k<n; ++k) {
          y(j,k) -= v * y(i,k);
          z(j,k) -= v * z(i,k);
        }
        y(j,i) = T(0.0);
      }
  }

  return z;
}
