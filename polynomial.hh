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

#ifndef POLYNOMIAL_HH
#define POLYNOMIAL_HH

inline static double ipow(double x, unsigned y)
{
  // At some point, calling C library pow() is faster
  if (y > 16)
    return pow(x, double(y));

  double r = 1.0;
  // Invariant: result = r * pow(x, y)
  while (y) {
    if (y & 1) {
      // r * pow(x, odd y) == (r * x) * pow(x, y - 1)
      r *= x;
      y -= 1;
    } else {
      // r * pow(x, even y) == r * pow(x * x, y / 2)
      x *= x;
      y >>= 1;
    }
  }
  return r;
}

class Polynomial : public CommutativeAlgebra<double, Polynomial>
{
public:
  using Algebra<double, Polynomial>::operator=;
  Polynomial() {}
  explicit Polynomial(double c) { if (c != 0.) coeffs[0] = c; }
  Polynomial(double c, unsigned n) { if (c != 0.) coeffs[n] = c; }
  double operator()(double x) const;
  const Polynomial &operator+=(const Polynomial &rhs);
  Polynomial operator-() const;
  const Polynomial &operator*=(double rhs);
  Polynomial operator*(const Polynomial &rhs) const;
  Polynomial derivative(unsigned n) const;

private:
  std::map<unsigned,double> coeffs;
};

inline double Polynomial::operator()(double x) const
{
  std::map<unsigned,double>::const_iterator i;
  double r = 0.;

  for (i = coeffs.begin(); i != coeffs.end(); ++i)
    r += i->second * ipow(x, i->first);

  return r;
}

inline const Polynomial &Polynomial::operator+=(const Polynomial &rhs)
{
  std::map<unsigned,double>::const_iterator i;

  for (i = rhs.coeffs.begin(); i != rhs.coeffs.end(); ++i) {
    unsigned d = i->first;
    this->coeffs[d] += i->second;
    if (this->coeffs[d] == 0.)
      this->coeffs.erase(d);
  }

  return *this;
}

inline Polynomial Polynomial::operator-() const
{
  std::map<unsigned,double>::const_iterator i;
  Polynomial f;

  for (i = this->coeffs.begin(); i != this->coeffs.end(); ++i)
    f.coeffs[i->first] = - i->second;

  return f;
}

inline const Polynomial &Polynomial::operator*=(double rhs)
{
  std::map<unsigned,double>::const_iterator i;

  for (i = coeffs.begin(); i != coeffs.end(); ++i) {
    unsigned d = i->first;
    coeffs[d] *= rhs;
    if (coeffs[d] == 0.)
      coeffs.erase(d);
  }

  return *this;
}

inline Polynomial Polynomial::operator*(const Polynomial &rhs) const
{
  std::map<unsigned,double>::const_iterator i, j;
  Polynomial f;

  for (i = this->coeffs.begin(); i != this->coeffs.end(); ++i)
    for (j = rhs.coeffs.begin(); j != rhs.coeffs.end(); ++j) {
      unsigned d = i->first + j->first;
      f.coeffs[d] += i->second * j->second;
      if (f.coeffs[d] == 0.)
        f.coeffs.erase(d);
    }

  return f;
}

inline Polynomial pow(const Polynomial &lhs, unsigned n)
{
  if (n == 0) return Polynomial(1.);
  if (n == 1) return lhs;
  if (n & 1) return lhs * pow(lhs, n - 1);
  return pow(lhs * lhs, n / 2);
}

inline Polynomial Polynomial::derivative(unsigned n = 1) const
{
  if (n == 0)
    return *this;

  std::map<unsigned, double>::const_iterator i;
  Polynomial df;

  for (i = coeffs.begin(); i != coeffs.end(); ++i)
    if (i->first > 0)
      df.coeffs[i->first - 1] = double(i->first) * i->second;

  return df.derivative(n - 1);
}

#endif
