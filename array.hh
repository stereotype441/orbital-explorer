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

#ifndef ARRAY_HH
#define ARRAY_HH

#include <vector>
#include <stdexcept>

#include "genericops.hh"

template <unsigned n, typename T>
class Array : public Equality<Array<n,T> >
{
public:
  Array() {}

  // Array can be explicitly constructed from a scalar, but no corresponding
  // assignment operator is defined -- because derived classes have different
  // behavior when assigned a scalar.
  explicit Array(const T &x)
  {
    for (int i = 0; i < n; ++i)
      data[i] = x;
  }

  // A "conversion constructor" for explicitly turning arrays of one type
  // into another as long as the sizes are the same and the element type
  // is (explicitly or implicitly) convertible.
  // This is more type-safe than a conversion operator, which can't be
  // made explicit.
  template <typename U>
  explicit Array(const Array<n,U> &xs)
  {
    for (unsigned i=0; i<n; ++i)
      data[i] = T(xs[i]);
  }

  // Similarly, a function for (explicitly) converting an array into an
  // STL vector
  std::vector<T> toVector() const
  {
    std::vector<T> r;
    for (int i = 0; i < n; ++i)
      r.push_back(data[i]);
    return r;
  }

  T &operator[](unsigned i)
  {
    check_index(i);
    return data[i];
  }
  const T &operator[](unsigned i) const
  {
    check_index(i);
    return data[i];
  }

  bool operator==(const Array<n,T> &rhs) const
  {
    for (int i = 0; i < n; ++i)
      if (data[i] != rhs.data[i])
        return false;
    return true;
  }

protected:
  T &unsafe_element(unsigned i)
  {
    return data[i];
  }
  const T &unsafe_element(unsigned i) const
  {
    return data[i];
  }

private:
  T data[n];

  void check_index(unsigned i) const
  {
    if (i >= n)
      throw_array_range_exception();
  }
  void throw_array_range_exception() const;
};

template <unsigned n, typename T>
void Array<n,T>::throw_array_range_exception() const
{
  throw std::range_error("Array<> index out of range");
}

#endif
