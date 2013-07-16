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

inline double factorial(int n)
{
  if (n < 0)
    throw std::logic_error("Factorial of negative number");
  if (n == 0)
    return 1.;
  return double(n) * factorial(n - 1);
}

inline double choose(int n, int k)
{
  return factorial(n) / (factorial(k) * factorial(n - k));
}

class Orbital : public Function<3,std::complex<double> >
{
public:
  Orbital(int Z_, int N_, int L_, int M_,
          bool real_, bool diff_, bool square_);
  double radial_part(double r) const;
  double theta_part(double sin_theta, double cos_theta) const;
  std::complex<double> operator()(const Vector<3> &x) const;
  const int Z, N, L, M;
  const bool real; // If true, add values for +/-M and divide by root 2
  const bool diff; // If true, real wave function is a difference of
  //                  wave functions for +/-M. If false, sum.
  const bool square; // If true, square magnitude of the function
  bool operator==(const Orbital &rhs);
  bool operator!=(const Orbital &rhs) { return !(*this == rhs); }
  double radius() const;
  double radialIntegral() const;

private:
  double radial_constant;
  double radial_exponential_constant;
  Polynomial radial_polynomial;

  double angular_constant;
  Polynomial cos_theta_polynomial;

  double normalization_constant;
};
