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
#include <set>
#include <algorithm>
#include <complex>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <iostream>

#include "util.hh"
#include "array.hh"
#include "genericops.hh"
#include "vector.hh"
#include "matrix.hh"
#include "quaternion.hh"
#include "delaunay.hh"
#include "function.hh"
#include "polynomial.hh"
#include "wavefunction.hh"

using namespace std;
#include "gtest/gtest.h"

TEST(ArrayTest, ConstructUninitialized) {
  Array<2,int> a;
}

TEST(ArrayTest, ConstructScalar) {
  Array<2,int> a(6);
  EXPECT_EQ(a[0], 6);
  EXPECT_EQ(a[1], 6);
}

TEST(ArrayTest, ConstructArray) {
  Array<2,int> a;
  a[0] = 11; a[1] = 29;
  Array<2,int> b(a);
  EXPECT_EQ(b[0], 11);
  EXPECT_EQ(b[1], 29);
  Array<2,int> c = a;
  EXPECT_EQ(c[0], 11);
  EXPECT_EQ(c[1], 29);
}

TEST(ArrayTest, ConstructDifferentArray) {
  Array<2,char> a;
  a[0] = 89; a[1] = 65;
  Array<2,int> b(a);
  EXPECT_EQ(b[0], 89);
  EXPECT_EQ(b[1], 65);
  Array<2,int> c = Array<2,int>(a);
  EXPECT_EQ(c[0], 89);
  EXPECT_EQ(c[1], 65);
}

TEST(ArrayTest, ConvertArray) {
  Array<2,char> a;
  a[0] = 1; a[1] = 127;
  Array<2,int> b;
  b = Array<2,int>(a);
  EXPECT_EQ(b[0], 1);
  EXPECT_EQ(b[1], 127);
}

TEST(ArrayTest, Access) {
  Array<2,int> a(45);
  EXPECT_EQ(a[0], 45);
  EXPECT_EQ(a[1], 45);
}

TEST(ArrayTest, AccessWrite) {
  Array<2,int> a(45);
  a[0] = 67; a[1] = 78;
  EXPECT_EQ(a[0], 67);
  EXPECT_EQ(a[1], 78);
}

TEST(ArrayTest, AccessConst) {
  const Array<2,int> a(45);
  EXPECT_EQ(a[0], 45);
  EXPECT_EQ(a[1], 45);
}

TEST(ArrayTest, AccessOutOfRange) {
  Array<2,int> a;
  EXPECT_ANY_THROW(a[2];);
  EXPECT_ANY_THROW(a[-1];);
}

TEST(ArrayTest, AccessWriteOutOfRange) {
  Array<2,int> a;
  EXPECT_ANY_THROW(a[2] = 5;);
  EXPECT_ANY_THROW(a[-1] = 6;);
}

TEST(ArrayTest, AccessConstOutOfRange) {
  const Array<2,int> a;
  EXPECT_ANY_THROW(a[2];);
  EXPECT_ANY_THROW(a[-1];);
}

// An exercise in correctness: verify that Array can be copy constructed
// and/or conversion constructed even when the base type has no default
// constructor.
class NoDefaultConstructor3 {
public:
  NoDefaultConstructor3(int) {}
};

class NoDefaultConstructor {
public:
  NoDefaultConstructor(int) {}
  operator NoDefaultConstructor3() const { return NoDefaultConstructor3(0); }
};

class NoDefaultConstructor2 {
public:
  NoDefaultConstructor2(int) {}
  explicit NoDefaultConstructor2(const NoDefaultConstructor &) {}
};

TEST(ArrayTest, ConstructWithoutDefaultConstructor) {
  Array<2,NoDefaultConstructor> a(NoDefaultConstructor(5));
  Array<2,NoDefaultConstructor> b(a);
  Array<2,NoDefaultConstructor2> c(a);
  Array<2,NoDefaultConstructor3> d(a);
  Array<2,NoDefaultConstructor> bb = a;
  Array<2,NoDefaultConstructor2> cc = Array<2,NoDefaultConstructor2>(a);
  Array<2,NoDefaultConstructor3> dd = Array<2,NoDefaultConstructor3>(a);
}

class VectorTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    x[0] = 1;
    x[1] = 2;
    x[2] = 3;
    y[0] = 10;
    y[1] = 20;
    y[2] = 30;
    z[0] = 0;
    z[1] = 0.5;
    z[2] = -1;
  }
  Vector<3> x, y, z;
};

TEST_F(VectorTest, ConstructUninitialized) {
  Vector<2> a;
}

TEST_F(VectorTest, ConstructScalar) {
  Vector<2> a(6.75);
  EXPECT_EQ(a[0], 6.75);
  EXPECT_EQ(a[1], 6.75);
}

TEST_F(VectorTest, ConstructVector) {
  Vector<3> a(x);
  EXPECT_EQ(a[0], 1.0) << "a[0] != 1";
  EXPECT_EQ(a[1], 2.0) << "a[1] != 2";
  EXPECT_EQ(a[2], 3.0) << "a[2] != 3";
  Vector<3> b = y;
  EXPECT_EQ(b[0], 10.0) << "b[0] != 10";
  EXPECT_EQ(b[1], 20.0) << "b[1] != 20";
  EXPECT_EQ(b[2], 30.0) << "b[2] != 30";
}

TEST_F(VectorTest, AssignScalar) {
  Vector<2> a;
  a = 6.0;
  EXPECT_EQ(6.0, a[0]);
  EXPECT_EQ(6.0, a[1]);
}

TEST_F(VectorTest, ConvertVectorToArray) {
  Array<3,double> a(x);
  EXPECT_EQ(a[0], 1.0) << "a[0] != 1";
  EXPECT_EQ(a[1], 2.0) << "a[1] != 2";
  EXPECT_EQ(a[2], 3.0) << "a[2] != 3";
  Array<3,double> b = y;
  EXPECT_EQ(b[0], 10.0) << "b[0] != 10";
  EXPECT_EQ(b[1], 20.0) << "b[1] != 20";
  EXPECT_EQ(b[2], 30.0) << "b[2] != 30";
}

TEST_F(VectorTest, ConvertVectorToDifferentArray) {
  Array<3,float> a(x);
  EXPECT_EQ(a[0], 1.0) << "a[0] != 1";
  EXPECT_EQ(a[1], 2.0) << "a[1] != 2";
  EXPECT_EQ(a[2], 3.0) << "a[2] != 3";
  Array<3,float> b = Array<3,float>(y);
  EXPECT_EQ(b[0], 10.0) << "b[0] != 10";
  EXPECT_EQ(b[1], 20.0) << "b[1] != 20";
  EXPECT_EQ(b[2], 30.0) << "b[2] != 30";
}

TEST_F(VectorTest, BasisVector) {
  Vector<3> a = basisVector<3>(2);
  EXPECT_EQ(a[0], 0.0);
  EXPECT_EQ(a[1], 0.0);
  EXPECT_EQ(a[2], 1.0);
  EXPECT_ANY_THROW(basisVector<3>(3););
}

TEST_F(VectorTest, AddEqualVector) {
  Vector<3> a = x;
  a += y;
  EXPECT_EQ(a[0], 11.0);
  EXPECT_EQ(a[1], 22.0);
  EXPECT_EQ(a[2], 33.0);
}

TEST_F(VectorTest, PositiveVector) {
  Vector<3> a = +x;
  EXPECT_EQ(a[0], 1.0);
  EXPECT_EQ(a[1], 2.0);
  EXPECT_EQ(a[2], 3.0);
}

TEST_F(VectorTest, AddVector) {
  Vector<3> a = x+y;
  EXPECT_EQ(a[0], 11.0);
  EXPECT_EQ(a[1], 22.0);
  EXPECT_EQ(a[2], 33.0);
}

TEST_F(VectorTest, SubtractEqualVector) {
  Vector<3> a = y;
  a -= x;
  EXPECT_EQ(a[0], 9.0);
  EXPECT_EQ(a[1], 18.0);
  EXPECT_EQ(a[2], 27.0);
}

TEST_F(VectorTest, NegateVector) {
  Vector<3> a = -x;
  EXPECT_EQ(a[0], -1.0);
  EXPECT_EQ(a[1], -2.0);
  EXPECT_EQ(a[2], -3.0);
}

TEST_F(VectorTest, SubtractVector) {
  Vector<3> a = y-x;
  EXPECT_EQ(a[0], 9.0);
  EXPECT_EQ(a[1], 18.0);
  EXPECT_EQ(a[2], 27.0);
}

TEST_F(VectorTest, ScalarTimesEqual) {
  Vector<3> a = x;
  a *= 3.0;
  EXPECT_EQ(a[0], 3.0);
  EXPECT_EQ(a[1], 6.0);
  EXPECT_EQ(a[2], 9.0);
}

TEST_F(VectorTest, ScalarMultiplication) {
  Vector<3> a = 3.0 * x;
  EXPECT_EQ(a[0], 3.0);
  EXPECT_EQ(a[1], 6.0);
  EXPECT_EQ(a[2], 9.0);
  Vector<3> b = y * -3.0;
  EXPECT_EQ(b[0], -30.0);
  EXPECT_EQ(b[1], -60.0);
  EXPECT_EQ(b[2], -90.0);
}

TEST_F(VectorTest, ScalarDivideEqual) {
  Vector<3> a = x;
  a /= 4.0;
  EXPECT_EQ(a[0], 0.25);
  EXPECT_EQ(a[1], 0.50);
  EXPECT_EQ(a[2], 0.75);
}

TEST_F(VectorTest, ScalarDivision) {
  Vector<3> c = y / 5.0;
  EXPECT_EQ(c[0], 2.0);
  EXPECT_EQ(c[1], 4.0);
  EXPECT_EQ(c[2], 6.0);
}

TEST_F(VectorTest, InnerProduct) {
  EXPECT_EQ(dot_product(x, y), 140.0);
  EXPECT_EQ(dot_product(x, z), -2.0);
}

TEST_F(VectorTest, CrossProduct) {
  Vector<3> a = cross_product(x, y);
  EXPECT_EQ(a[0], 0.0);
  EXPECT_EQ(a[1], 0.0);
  EXPECT_EQ(a[2], 0.0);
  Vector<3> b = cross_product(x, z);
  EXPECT_EQ(b[0], -3.5);
  EXPECT_EQ(b[1], 1.0);
  EXPECT_EQ(b[2], 0.5);
}

TEST_F(VectorTest, NormSquared) {
  EXPECT_EQ(14.0, norm_squared(x));
  EXPECT_EQ(1400.0, norm_squared(y));
  EXPECT_EQ(1.25, norm_squared(z));
}

TEST_F(VectorTest, Norm) {
  EXPECT_EQ(sqrt(14.0), norm(x));
  EXPECT_EQ(sqrt(1400.0), norm(y));
  EXPECT_EQ(sqrt(1.25), norm(z));
}

class CVectorTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    r = complex<double>(1., 0.);
    i = complex<double>(0., 1.);
    x[0] = 1;
    x[1] = 2;
    x[2] = 3.0 * i;
    y[0] = 10;
    y[1] = 20;
    y[2] = 30.0 * i;
    z[0] = 0;
    z[1] = 0.5;
    z[2] = -1.0 * i;
  }
  complex<double> r, i;
  CVector<3> x, y, z;
};

TEST_F(CVectorTest, ConstructUninitialized) {
  CVector<2> a;
}

TEST_F(CVectorTest, ConstructScalar) {
  CVector<2> a(6.75);
  EXPECT_EQ(a[0], 6.75);
  EXPECT_EQ(a[1], 6.75);
  CVector<1> b(i);
  EXPECT_EQ(b[0], i);
}

TEST_F(CVectorTest, ConstructVector) {
  CVector<3> a(x);
  EXPECT_EQ(a[0], 1.0) << "a[0] != 1";
  EXPECT_EQ(a[1], 2.0) << "a[1] != 2";
  EXPECT_EQ(a[2], 3.0 * i) << "a[2] != 3i";
  CVector<3> b = y;
  EXPECT_EQ(b[0], 10.0) << "b[0] != 10";
  EXPECT_EQ(b[1], 20.0) << "b[1] != 20";
  EXPECT_EQ(b[2], 30.0 * i) << "b[2] != 30i";
}

TEST_F(CVectorTest, AssignScalar) {
  CVector<2> a;
  a = 6.0;
  EXPECT_EQ(6.0, a[0]);
  EXPECT_EQ(6.0, a[1]);
}

TEST_F(CVectorTest, ConvertVectorToArray) {
  Array<3,complex<double> > a(x);
  EXPECT_EQ(a[0], 1.0) << "a[0] != 1";
  EXPECT_EQ(a[1], 2.0) << "a[1] != 2";
  EXPECT_EQ(a[2], 3.0 * i) << "a[2] != 3i";
  Array<3,complex<double> > b = y;
  EXPECT_EQ(b[0], 10.0) << "b[0] != 10";
  EXPECT_EQ(b[1], 20.0) << "b[1] != 20";
  EXPECT_EQ(b[2], 30.0 * i) << "b[2] != 30i";
}

TEST_F(CVectorTest, ConvertVectorToDifferentArray) {
  Array<3,complex<float> > a(x);
  EXPECT_EQ(a[0], complex<float>(1.0)) << "a[0] != 1";
  EXPECT_EQ(a[1], complex<float>(2.0)) << "a[1] != 2";
  EXPECT_EQ(a[2], complex<float>(0.0, 3.0)) << "a[2] != 3i";
  Array<3,complex<float> > b = Array<3,complex<float> >(y);
  EXPECT_EQ(b[0], complex<float>(10.0)) << "b[0] != 10";
  EXPECT_EQ(b[1], complex<float>(20.0)) << "b[1] != 20";
  EXPECT_EQ(b[2], complex<float>(0.0, 30.0)) << "b[2] != 30i";
}

TEST_F(CVectorTest, BasisVector) {
  CVector<3> a = cBasisVector<3>(2);
  EXPECT_EQ(a[0], 0.0);
  EXPECT_EQ(a[1], 0.0);
  EXPECT_EQ(a[2], 1.0);
  EXPECT_ANY_THROW(cBasisVector<3>(3););
}

TEST_F(CVectorTest, AddEqualVector) {
  CVector<3> a = x;
  a += y;
  EXPECT_EQ(a[0], 11.0);
  EXPECT_EQ(a[1], 22.0);
  EXPECT_EQ(a[2], 33.0 * i);
}

TEST_F(CVectorTest, PositiveVector) {
  CVector<3> a = +x;
  EXPECT_EQ(a[0], 1.0);
  EXPECT_EQ(a[1], 2.0);
  EXPECT_EQ(a[2], 3.0 * i);
}

TEST_F(CVectorTest, AddVector) {
  CVector<3> a = x+y;
  EXPECT_EQ(a[0], 11.0);
  EXPECT_EQ(a[1], 22.0);
  EXPECT_EQ(a[2], 33.0 * i);
}

TEST_F(CVectorTest, SubtractEqualVector) {
  CVector<3> a = y;
  a -= x;
  EXPECT_EQ(a[0], 9.0);
  EXPECT_EQ(a[1], 18.0);
  EXPECT_EQ(a[2], 27.0 * i);
}

TEST_F(CVectorTest, NegateVector) {
  CVector<3> a = -x;
  EXPECT_EQ(a[0], -1.0);
  EXPECT_EQ(a[1], -2.0);
  EXPECT_EQ(a[2], -3.0 * i);
}

TEST_F(CVectorTest, SubtractVector) {
  CVector<3> a = y-x;
  EXPECT_EQ(a[0], 9.0);
  EXPECT_EQ(a[1], 18.0);
  EXPECT_EQ(a[2], 27.0 * i);
}

TEST_F(CVectorTest, ScalarTimesEqual) {
  CVector<3> a = x;
  a *= 3.0 * r;
  EXPECT_EQ(a[0], 3.0);
  EXPECT_EQ(a[1], 6.0);
  EXPECT_EQ(a[2], 9.0 * i);
}

TEST_F(CVectorTest, ScalarMultiplication) {
  CVector<3> a = (3.0 * r) * x;
  EXPECT_EQ(a[0], 3.0);
  EXPECT_EQ(a[1], 6.0);
  EXPECT_EQ(a[2], 9.0 * i);
  CVector<3> b = y * (-3.0 * r);
  EXPECT_EQ(b[0], -30.0);
  EXPECT_EQ(b[1], -60.0);
  EXPECT_EQ(b[2], -90.0 * i);
}

TEST_F(CVectorTest, ScalarDivideEqual) {
  CVector<3> a = x;
  a /= 4.0 * r;
  EXPECT_EQ(a[0], 0.25);
  EXPECT_EQ(a[1], 0.50);
  EXPECT_EQ(a[2], 0.75 * i);
}

TEST_F(CVectorTest, ScalarDivision) {
  CVector<3> c = y / (5.0 * r);
  EXPECT_EQ(c[0], 2.0);
  EXPECT_EQ(c[1], 4.0);
  EXPECT_EQ(c[2], 6.0 * i);
}

TEST_F(CVectorTest, InnerProduct) {
  EXPECT_EQ(dot_product(x, y), 140.0);
  EXPECT_EQ(dot_product(x, z), -2.0);
}

TEST_F(CVectorTest, NormSquared) {
  EXPECT_EQ(14.0, norm_squared(x));
  EXPECT_EQ(1400.0, norm_squared(y));
  EXPECT_EQ(1.25, norm_squared(z));
}

TEST_F(CVectorTest, Norm) {
  EXPECT_EQ(sqrt(14.0), norm(x));
  EXPECT_EQ(sqrt(1400.0), norm(y));
  EXPECT_EQ(sqrt(1.25), norm(z));
}

class MatrixTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    x(0,0) = 1;
    x(0,1) = 2;
    x(0,2) = 3;
    x(1,0) = 4;
    x(1,1) = 5;
    x(1,2) = 6;
    y(0,0) = 110;
    y(0,1) = 120;
    y(0,2) = 130;
    y(1,0) = 140;
    y(1,1) = 150;
    y(1,2) = 160;
    z(0,0) = 1.0;
    z(0,1) = 10.0;
    z(1,0) = 2.0;
    z(1,1) = 20.0;
    z(2,0) = 3.0;
    z(2,1) = 30.0;
  }
  Matrix<2,3> x, y;
  Matrix<3,2> z;
};

TEST_F(MatrixTest, ConstructUninitialized) {
  Matrix<2,2> a;
}

TEST_F(MatrixTest, ConstructScalar) {
  Matrix<2,2> a(3);
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 0.0);
  EXPECT_EQ(a(1,0), 0.0);
  EXPECT_EQ(a(1,1), 3.0);
}

TEST_F(MatrixTest, ConstructMatrix) {
  Matrix<2,3> a(x);
  EXPECT_EQ(1.0, a(0,0));
  EXPECT_EQ(2.0, a(0,1));
  EXPECT_EQ(3.0, a(0,2));
  EXPECT_EQ(4.0, a(1,0));
  EXPECT_EQ(5.0, a(1,1));
  EXPECT_EQ(6.0, a(1,2));
  Matrix<2,3> b = y;
  EXPECT_EQ(110.0, b(0,0));
  EXPECT_EQ(120.0, b(0,1));
  EXPECT_EQ(130.0, b(0,2));
  EXPECT_EQ(140.0, b(1,0));
  EXPECT_EQ(150.0, b(1,1));
  EXPECT_EQ(160.0, b(1,2));
}

TEST_F(MatrixTest, AssignScalar) {
  Matrix<2,2> a;
  a(0,0) = 1;
  a(0,1) = 2;
  a(1,0) = 3;
  a(1,1) = 4;
  a = 5;
  EXPECT_EQ(5.0, a(0,0));
  EXPECT_EQ(0.0, a(0,1));
  EXPECT_EQ(0.0, a(1,0));
  EXPECT_EQ(5.0, a(1,1));
}

TEST_F(MatrixTest, ConvertMatrixToArray) {
  Array<6,double> a(x);
  EXPECT_EQ(1.0, a[0]);
  EXPECT_EQ(2.0, a[1]);
  EXPECT_EQ(3.0, a[2]);
  EXPECT_EQ(4.0, a[3]);
  EXPECT_EQ(5.0, a[4]);
  EXPECT_EQ(6.0, a[5]);
  Array<6,double> b = y;
  EXPECT_EQ(110.0, b[0]);
  EXPECT_EQ(120.0, b[1]);
  EXPECT_EQ(130.0, b[2]);
  EXPECT_EQ(140.0, b[3]);
  EXPECT_EQ(150.0, b[4]);
  EXPECT_EQ(160.0, b[5]);
}

TEST_F(MatrixTest, ConvertMatrixToDifferentArray) {
  Array<6,float> a(x);
  EXPECT_EQ(1.0, a[0]);
  EXPECT_EQ(2.0, a[1]);
  EXPECT_EQ(3.0, a[2]);
  EXPECT_EQ(4.0, a[3]);
  EXPECT_EQ(5.0, a[4]);
  EXPECT_EQ(6.0, a[5]);
  Array<6,float> b = Array<6,float>(y);
  EXPECT_EQ(110.0, b[0]);
  EXPECT_EQ(120.0, b[1]);
  EXPECT_EQ(130.0, b[2]);
  EXPECT_EQ(140.0, b[3]);
  EXPECT_EQ(150.0, b[4]);
  EXPECT_EQ(160.0, b[5]);
}

TEST_F(MatrixTest, AccessConst) {
  const Matrix<2,3> a(x);
  EXPECT_EQ(1.0, a(0,0));
  EXPECT_EQ(2.0, a(0,1));
  EXPECT_EQ(3.0, a(0,2));
  EXPECT_EQ(4.0, a(1,0));
  EXPECT_EQ(5.0, a(1,1));
  EXPECT_EQ(6.0, a(1,2));
}

TEST_F(MatrixTest, AddEqualsMatrix) {
  Matrix<2,3> a(x);
  a += y;
  EXPECT_EQ(a(0,0), 111.0);
  EXPECT_EQ(a(0,1), 122.0);
  EXPECT_EQ(a(0,2), 133.0);
  EXPECT_EQ(a(1,0), 144.0);
  EXPECT_EQ(a(1,1), 155.0);
  EXPECT_EQ(a(1,2), 166.0);
}

TEST_F(MatrixTest, PositiveMatrix) {
  Matrix<2,3> a;
  a = +x;
  EXPECT_EQ(a(0,0), 1.0);
  EXPECT_EQ(a(0,1), 2.0);
  EXPECT_EQ(a(0,2), 3.0);
  EXPECT_EQ(a(1,0), 4.0);
  EXPECT_EQ(a(1,1), 5.0);
  EXPECT_EQ(a(1,2), 6.0);
}

TEST_F(MatrixTest, AddMatrix) {
  Matrix<2,3> a = x+y;
  EXPECT_EQ(a(0,0), 111.0);
  EXPECT_EQ(a(0,1), 122.0);
  EXPECT_EQ(a(0,2), 133.0);
  EXPECT_EQ(a(1,0), 144.0);
  EXPECT_EQ(a(1,1), 155.0);
  EXPECT_EQ(a(1,2), 166.0);
}

TEST_F(MatrixTest, SubtractEqualsMatrix) {
  Matrix<2,3> a = y;
  a -= x;
  EXPECT_EQ(a(0,0), 109.0);
  EXPECT_EQ(a(0,1), 118.0);
  EXPECT_EQ(a(0,2), 127.0);
  EXPECT_EQ(a(1,0), 136.0);
  EXPECT_EQ(a(1,1), 145.0);
  EXPECT_EQ(a(1,2), 154.0);
}

TEST_F(MatrixTest, NegateMatrix) {
  Matrix<2,3> a = -x;
  EXPECT_EQ(a(0,0), -1.0);
  EXPECT_EQ(a(0,1), -2.0);
  EXPECT_EQ(a(0,2), -3.0);
  EXPECT_EQ(a(1,0), -4.0);
  EXPECT_EQ(a(1,1), -5.0);
  EXPECT_EQ(a(1,2), -6.0);
}

TEST_F(MatrixTest, SubtractMatrix) {
  Matrix<2,3> a = y-x;
  EXPECT_EQ(a(0,0), 109.0);
  EXPECT_EQ(a(0,1), 118.0);
  EXPECT_EQ(a(0,2), 127.0);
  EXPECT_EQ(a(1,0), 136.0);
  EXPECT_EQ(a(1,1), 145.0);
  EXPECT_EQ(a(1,2), 154.0);
}

TEST_F(MatrixTest, ScalarMultiplyEquals) {
  Matrix<2,3> a(x);
  a *= 3.0;
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 6.0);
  EXPECT_EQ(a(0,2), 9.0);
  EXPECT_EQ(a(1,0), 12.0);
  EXPECT_EQ(a(1,1), 15.0);
  EXPECT_EQ(a(1,2), 18.0);
}

TEST_F(MatrixTest, ScalarDivideEquals) {
  Matrix<2,3> a(x);
  a /= 4.0;
  EXPECT_EQ(a(0,0), 0.25);
  EXPECT_EQ(a(0,1), 0.50);
  EXPECT_EQ(a(0,2), 0.75);
  EXPECT_EQ(a(1,0), 1.00);
  EXPECT_EQ(a(1,1), 1.25);
  EXPECT_EQ(a(1,2), 1.50);
}

TEST_F(MatrixTest, ScalarMultiplication) {
  Matrix<2,3> a = 3.0 * x;
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 6.0);
  EXPECT_EQ(a(0,2), 9.0);
  EXPECT_EQ(a(1,0), 12.0);
  EXPECT_EQ(a(1,1), 15.0);
  EXPECT_EQ(a(1,2), 18.0);
  Matrix<2,3> b = y * -3.0;
  EXPECT_EQ(b(0,0), -330.0);
  EXPECT_EQ(b(0,1), -360.0);
  EXPECT_EQ(b(0,2), -390.0);
  EXPECT_EQ(b(1,0), -420.0);
  EXPECT_EQ(b(1,1), -450.0);
  EXPECT_EQ(b(1,2), -480.0);
  Matrix<2,3> c = y / 5.0;
  EXPECT_EQ(c(0,0), 22.0);
  EXPECT_EQ(c(0,1), 24.0);
  EXPECT_EQ(c(0,2), 26.0);
  EXPECT_EQ(c(1,0), 28.0);
  EXPECT_EQ(c(1,1), 30.0);
  EXPECT_EQ(c(1,2), 32.0);
}

TEST_F(MatrixTest, MatrixMultiplication) {
  Matrix<2,2> a = x * z;
  EXPECT_EQ( 14.0, a(0,0));
  EXPECT_EQ(140.0, a(0,1));
  EXPECT_EQ( 32.0, a(1,0));
  EXPECT_EQ(320.0, a(1,1));
  Matrix<3,3> b = z * x;
  EXPECT_EQ( 41.0, b(0,0));
  EXPECT_EQ( 52.0, b(0,1));
  EXPECT_EQ( 63.0, b(0,2));
  EXPECT_EQ( 82.0, b(1,0));
  EXPECT_EQ(104.0, b(1,1));
  EXPECT_EQ(126.0, b(1,2));
  EXPECT_EQ(123.0, b(2,0));
  EXPECT_EQ(156.0, b(2,1));
  EXPECT_EQ(189.0, b(2,2));
}

TEST_F(MatrixTest, MatrixVectorMultiplication) {
  Vector<3> a;
  a[0] = 1.0;
  a[1] = 2.0;
  a[2] = 3.0;
  Vector<2> b = x * a;
  EXPECT_EQ(14.0, b[0]);
  EXPECT_EQ(32.0, b[1]);
}

TEST_F(MatrixTest, MatrixTranspose)
{
  Matrix<3,2> a(transpose(x));
  EXPECT_EQ(a(0,0), 1);
  EXPECT_EQ(a(0,1), 4);
  EXPECT_EQ(a(1,0), 2);
  EXPECT_EQ(a(1,1), 5);
  EXPECT_EQ(a(2,0), 3);
  EXPECT_EQ(a(2,1), 6);
}

TEST_F(MatrixTest, MatrixInverse) {
  Matrix<2,2> a(1.0);
  Matrix<2,2> ai(inverse(a));
  EXPECT_EQ(1.0, ai(0,0));
  EXPECT_EQ(0.0, ai(0,1));
  EXPECT_EQ(0.0, ai(1,0));
  EXPECT_EQ(1.0, ai(1,1));

  Matrix<1,1> b(4.0);
  Matrix<1,1> bi(inverse(b));
  EXPECT_EQ(0.25, bi(0,0));

  Matrix<2,2> c(1.0);
  c(0,1) = 1.0;
  Matrix<2,2> ci(inverse(c));
  EXPECT_EQ(1.0, ci(0,0));
  EXPECT_EQ(-1.0, ci(0,1));
  EXPECT_EQ(0.0, ci(1,0));
  EXPECT_EQ(1.0, ci(1,1));

  Matrix<2,2> d(0.0);
  d(0,1) = d(1,0) = 1.0;
  Matrix<2,2> di(inverse(d));
  EXPECT_EQ(0.0, di(0,0));
  EXPECT_EQ(1.0, di(0,1));
  EXPECT_EQ(1.0, di(1,0));
  EXPECT_EQ(0.0, di(1,1));

  Matrix<2,2> e;
  e(0,0) = 1;
  e(0,1) = 2;
  e(1,0) = 1;
  e(1,1) = 4;
  Matrix<2,2> ei(inverse(e));
  EXPECT_EQ(2.0, ei(0,0));
  EXPECT_EQ(-1.0, ei(0,1));
  EXPECT_EQ(-0.5, ei(1,0));
  EXPECT_EQ(0.5, ei(1,1));
  Matrix<2,2> eei(e * ei);
  EXPECT_EQ(1.0, eei(0,0));
  EXPECT_EQ(0.0, eei(0,1));
  EXPECT_EQ(0.0, eei(1,0));
  EXPECT_EQ(1.0, eei(1,1));

  Matrix<3,3> f(0.0);
  f(0,0) = 1;
  f(1,0) = 2;
  f(2,0) = 3;
  f(0,1) = 4;
  f(1,1) = 6;
  f(2,1) = 7;
  f(0,2) = 3;
  f(1,2) = 8;
  f(2,2) = 15;
  Matrix<3,3> fi(inverse(f));
  Matrix<3,3> ffi(f * fi);
  EXPECT_NEAR(1.0, ffi(0,0), 1e-12);
  EXPECT_NEAR(0.0, ffi(0,1), 1e-12);
  EXPECT_NEAR(0.0, ffi(0,2), 1e-12);
  EXPECT_NEAR(0.0, ffi(1,0), 1e-12);
  EXPECT_NEAR(1.0, ffi(1,1), 1e-12);
  EXPECT_NEAR(0.0, ffi(1,2), 1e-12);
  EXPECT_NEAR(0.0, ffi(2,0), 1e-12);
  EXPECT_NEAR(0.0, ffi(2,1), 1e-12);
  EXPECT_NEAR(1.0, ffi(2,2), 1e-12);

  Matrix<10,10> g;
  for (unsigned i=0; i<10; ++i)
    for (unsigned j=0; j<10; ++j)
      g(i,j) = tan(10*i+j);
  Matrix<10,10> gi(inverse(g));
  Matrix<10,10> ggi(g * gi);
  for (unsigned i=0; i<10; ++i)
    for (unsigned j=0; j<10; ++j)
      EXPECT_NEAR(i == j ? 1.0 : 0.0, ggi(i,j), 16*DBL_EPSILON);
}

class CMatrixTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    r = complex<double>(1.0, 0.0);
    i = complex<double>(0.0, 1.0);
    x(0,0) = 1.0;
    x(0,1) = 2.0;
    x(0,2) = 3.0;
    x(1,0) = 4.0 * i;
    x(1,1) = 5.0 * i;
    x(1,2) = 6.0 * i;
    y(0,0) = 110.0;
    y(0,1) = 120.0;
    y(0,2) = 130.0;
    y(1,0) = 140.0 * i;
    y(1,1) = 150.0 * i;
    y(1,2) = 160.0 * i;
    z(0,0) = 1.0;
    z(0,1) = 10.0;
    z(1,0) = 2.0;
    z(1,1) = 20.0;
    z(2,0) = 3.0 * i;
    z(2,1) = 30.0 * i;
  }
  complex<double> r, i;
  CMatrix<2,3> x, y;
  CMatrix<3,2> z;
};

TEST_F(CMatrixTest, ConstructUninitialized) {
  CMatrix<2,2> a;
}

TEST_F(CMatrixTest, ConstructScalar) {
  CMatrix<2,2> a(3);
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 0.0);
  EXPECT_EQ(a(1,0), 0.0);
  EXPECT_EQ(a(1,1), 3.0);
}

TEST_F(CMatrixTest, ConstructMatrix) {
  CMatrix<2,3> a(x);
  EXPECT_EQ(1.0, a(0,0));
  EXPECT_EQ(2.0, a(0,1));
  EXPECT_EQ(3.0, a(0,2));
  EXPECT_EQ(4.0 * i, a(1,0));
  EXPECT_EQ(5.0 * i, a(1,1));
  EXPECT_EQ(6.0 * i, a(1,2));
  CMatrix<2,3> b = y;
  EXPECT_EQ(110.0, b(0,0));
  EXPECT_EQ(120.0, b(0,1));
  EXPECT_EQ(130.0, b(0,2));
  EXPECT_EQ(140.0 * i, b(1,0));
  EXPECT_EQ(150.0 * i, b(1,1));
  EXPECT_EQ(160.0 * i, b(1,2));
}

TEST_F(CMatrixTest, AssignScalar) {
  CMatrix<2,2> a;
  a(0,0) = 1;
  a(0,1) = 2;
  a(1,0) = 3;
  a(1,1) = 4;
  a = 5;
  EXPECT_EQ(5.0, a(0,0));
  EXPECT_EQ(0.0, a(0,1));
  EXPECT_EQ(0.0, a(1,0));
  EXPECT_EQ(5.0, a(1,1));
}

TEST_F(CMatrixTest, ConvertMatrixToArray) {
  Array<6,complex<double> > a(x);
  EXPECT_EQ(1.0, a[0]);
  EXPECT_EQ(2.0, a[1]);
  EXPECT_EQ(3.0, a[2]);
  EXPECT_EQ(4.0 * i, a[3]);
  EXPECT_EQ(5.0 * i, a[4]);
  EXPECT_EQ(6.0 * i, a[5]);
  Array<6,complex<double> > b = y;
  EXPECT_EQ(110.0, b[0]);
  EXPECT_EQ(120.0, b[1]);
  EXPECT_EQ(130.0, b[2]);
  EXPECT_EQ(140.0 * i, b[3]);
  EXPECT_EQ(150.0 * i, b[4]);
  EXPECT_EQ(160.0 * i, b[5]);
}

TEST_F(CMatrixTest, ConvertMatrixToDifferentArray) {
  Array<6,complex<float> > a(x);
  EXPECT_EQ(complex<float>(1.0), a[0]);
  EXPECT_EQ(complex<float>(2.0), a[1]);
  EXPECT_EQ(complex<float>(3.0), a[2]);
  EXPECT_EQ(complex<float>(4.0 * i), a[3]);
  EXPECT_EQ(complex<float>(5.0 * i), a[4]);
  EXPECT_EQ(complex<float>(6.0 * i), a[5]);
  Array<6,complex<float> > b = Array<6,complex<float> >(y);
  EXPECT_EQ(complex<float>(110.0), b[0]);
  EXPECT_EQ(complex<float>(120.0), b[1]);
  EXPECT_EQ(complex<float>(130.0), b[2]);
  EXPECT_EQ(complex<float>(140.0 * i), b[3]);
  EXPECT_EQ(complex<float>(150.0 * i), b[4]);
  EXPECT_EQ(complex<float>(160.0 * i), b[5]);
}

TEST_F(CMatrixTest, AccessConst) {
  const CMatrix<2,3> a(x);
  EXPECT_EQ(1.0, a(0,0));
  EXPECT_EQ(2.0, a(0,1));
  EXPECT_EQ(3.0, a(0,2));
  EXPECT_EQ(4.0 * i, a(1,0));
  EXPECT_EQ(5.0 * i, a(1,1));
  EXPECT_EQ(6.0 * i, a(1,2));
}

TEST_F(CMatrixTest, AddEqualsMatrix) {
  CMatrix<2,3> a(x);
  a += y;
  EXPECT_EQ(a(0,0), 111.0);
  EXPECT_EQ(a(0,1), 122.0);
  EXPECT_EQ(a(0,2), 133.0);
  EXPECT_EQ(a(1,0), 144.0 * i);
  EXPECT_EQ(a(1,1), 155.0 * i);
  EXPECT_EQ(a(1,2), 166.0 * i);
}

TEST_F(CMatrixTest, PositiveMatrix) {
  CMatrix<2,3> a;
  a = +x;
  EXPECT_EQ(a(0,0), 1.0);
  EXPECT_EQ(a(0,1), 2.0);
  EXPECT_EQ(a(0,2), 3.0);
  EXPECT_EQ(a(1,0), 4.0 * i);
  EXPECT_EQ(a(1,1), 5.0 * i);
  EXPECT_EQ(a(1,2), 6.0 * i);
}

TEST_F(CMatrixTest, AddMatrix) {
  CMatrix<2,3> a = x+y;
  EXPECT_EQ(a(0,0), 111.0);
  EXPECT_EQ(a(0,1), 122.0);
  EXPECT_EQ(a(0,2), 133.0);
  EXPECT_EQ(a(1,0), 144.0 * i);
  EXPECT_EQ(a(1,1), 155.0 * i);
  EXPECT_EQ(a(1,2), 166.0 * i);
}

TEST_F(CMatrixTest, SubtractEqualsMatrix) {
  CMatrix<2,3> a = y;
  a -= x;
  EXPECT_EQ(a(0,0), 109.0);
  EXPECT_EQ(a(0,1), 118.0);
  EXPECT_EQ(a(0,2), 127.0);
  EXPECT_EQ(a(1,0), 136.0 * i);
  EXPECT_EQ(a(1,1), 145.0 * i);
  EXPECT_EQ(a(1,2), 154.0 * i);
}

TEST_F(CMatrixTest, NegateMatrix) {
  CMatrix<2,3> a = -x;
  EXPECT_EQ(a(0,0), -1.0);
  EXPECT_EQ(a(0,1), -2.0);
  EXPECT_EQ(a(0,2), -3.0);
  EXPECT_EQ(a(1,0), -4.0 * i);
  EXPECT_EQ(a(1,1), -5.0 * i);
  EXPECT_EQ(a(1,2), -6.0 * i);
}

TEST_F(CMatrixTest, SubtractMatrix) {
  CMatrix<2,3> a = y-x;
  EXPECT_EQ(a(0,0), 109.0);
  EXPECT_EQ(a(0,1), 118.0);
  EXPECT_EQ(a(0,2), 127.0);
  EXPECT_EQ(a(1,0), 136.0 * i);
  EXPECT_EQ(a(1,1), 145.0 * i);
  EXPECT_EQ(a(1,2), 154.0 * i);
}

TEST_F(CMatrixTest, ScalarMultiplyEquals) {
  CMatrix<2,3> a(x);
  a *= 3.0 * r;
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 6.0);
  EXPECT_EQ(a(0,2), 9.0);
  EXPECT_EQ(a(1,0), 12.0 * i);
  EXPECT_EQ(a(1,1), 15.0 * i);
  EXPECT_EQ(a(1,2), 18.0 * i);
}

TEST_F(CMatrixTest, ScalarDivideEquals) {
  CMatrix<2,3> a(x);
  a /= 4.0 * r;
  EXPECT_EQ(a(0,0), 0.25);
  EXPECT_EQ(a(0,1), 0.50);
  EXPECT_EQ(a(0,2), 0.75);
  EXPECT_EQ(a(1,0), 1.00 * i);
  EXPECT_EQ(a(1,1), 1.25 * i);
  EXPECT_EQ(a(1,2), 1.50 * i);
}

TEST_F(CMatrixTest, ScalarMultiplication) {
  CMatrix<2,3> a = (3.0 * r) * x;
  EXPECT_EQ(a(0,0), 3.0);
  EXPECT_EQ(a(0,1), 6.0);
  EXPECT_EQ(a(0,2), 9.0);
  EXPECT_EQ(a(1,0), 12.0 * i);
  EXPECT_EQ(a(1,1), 15.0 * i);
  EXPECT_EQ(a(1,2), 18.0 * i);
  CMatrix<2,3> b = y * (-3.0 * r);
  EXPECT_EQ(b(0,0), -330.0);
  EXPECT_EQ(b(0,1), -360.0);
  EXPECT_EQ(b(0,2), -390.0);
  EXPECT_EQ(b(1,0), -420.0 * i);
  EXPECT_EQ(b(1,1), -450.0 * i);
  EXPECT_EQ(b(1,2), -480.0 * i);
  CMatrix<2,3> c = y / (5.0 * r);
  EXPECT_EQ(c(0,0), 22.0);
  EXPECT_EQ(c(0,1), 24.0);
  EXPECT_EQ(c(0,2), 26.0);
  EXPECT_EQ(c(1,0), 28.0 * i);
  EXPECT_EQ(c(1,1), 30.0 * i);
  EXPECT_EQ(c(1,2), 32.0 * i);
}

TEST_F(CMatrixTest, MatrixMultiplication) {
  CMatrix<2,2> a = x * z;
  EXPECT_EQ(   5.0 * r +   9.0 * i, a(0,0));
  EXPECT_EQ(  50.0 * r +  90.0 * i, a(0,1));
  EXPECT_EQ( -18.0 * r +  14.0 * i, a(1,0));
  EXPECT_EQ(-180.0 * r + 140.0 * i, a(1,1));
  CMatrix<3,3> b = z * x;
  EXPECT_EQ(1.0 * r +  40.0 * i, b(0,0));
  EXPECT_EQ(2.0 * r +  50.0 * i, b(0,1));
  EXPECT_EQ(3.0 * r +  60.0 * i, b(0,2));
  EXPECT_EQ(2.0 * r +  80.0 * i, b(1,0));
  EXPECT_EQ(4.0 * r + 100.0 * i, b(1,1));
  EXPECT_EQ(6.0 * r + 120.0 * i, b(1,2));
  EXPECT_EQ(3.0 * i - 120.0 * r, b(2,0));
  EXPECT_EQ(6.0 * i - 150.0 * r, b(2,1));
  EXPECT_EQ(9.0 * i - 180.0 * r, b(2,2));
}

TEST_F(CMatrixTest, MatrixVectorMultiplication) {
  CVector<3> a;
  a[0] = 1.0;
  a[1] = 2.0;
  a[2] = 3.0;
  CVector<2> b = x * a;
  EXPECT_EQ(14.0, b[0]);
  EXPECT_EQ(32.0 * i, b[1]);
}

TEST_F(CMatrixTest, MatrixTranspose)
{
  CMatrix<3,2> a(transpose(x));
  EXPECT_EQ(a(0,0), 1.0);
  EXPECT_EQ(a(0,1), 4.0 * i);
  EXPECT_EQ(a(1,0), 2.0);
  EXPECT_EQ(a(1,1), 5.0 * i);
  EXPECT_EQ(a(2,0), 3.0);
  EXPECT_EQ(a(2,1), 6.0 * i);
}

TEST_F(CMatrixTest, MatrixInverse) {
  CMatrix<2,2> a(1.0);
  CMatrix<2,2> ai(inverse(a));
  EXPECT_EQ(1.0, ai(0,0));
  EXPECT_EQ(0.0, ai(0,1));
  EXPECT_EQ(0.0, ai(1,0));
  EXPECT_EQ(1.0, ai(1,1));

  CMatrix<1,1> b(4.0 * i);
  CMatrix<1,1> bi(inverse(b));
  EXPECT_EQ(-0.25 * i, bi(0,0));

  CMatrix<2,2> c(1.0);
  c(0,1) = 1.0 * i;
  CMatrix<2,2> ci(inverse(c));
  EXPECT_EQ(1.0, ci(0,0));
  EXPECT_EQ(-1.0 * i, ci(0,1));
  EXPECT_EQ(0.0, ci(1,0));
  EXPECT_EQ(1.0, ci(1,1));

  CMatrix<2,2> d(0.0);
  d(0,1) = d(1,0) = 1.0;
  CMatrix<2,2> di(inverse(d));
  EXPECT_EQ(0.0, di(0,0));
  EXPECT_EQ(1.0, di(0,1));
  EXPECT_EQ(1.0, di(1,0));
  EXPECT_EQ(0.0, di(1,1));

  CMatrix<2,2> e;
  e(0,0) = 1;
  e(0,1) = 2;
  e(1,0) = 1;
  e(1,1) = 4;
  CMatrix<2,2> ei(inverse(e));
  EXPECT_EQ(2.0, ei(0,0));
  EXPECT_EQ(-1.0, ei(0,1));
  EXPECT_EQ(-0.5, ei(1,0));
  EXPECT_EQ(0.5, ei(1,1));
  CMatrix<2,2> eei(e * ei);
  EXPECT_EQ(1.0, eei(0,0));
  EXPECT_EQ(0.0, eei(0,1));
  EXPECT_EQ(0.0, eei(1,0));
  EXPECT_EQ(1.0, eei(1,1));

  CMatrix<3,3> f(0.0);
  f(0,0) = 1;
  f(1,0) = 2;
  f(2,0) = 3;
  f(0,1) = 4;
  f(1,1) = 6;
  f(2,1) = 7;
  f(0,2) = 3;
  f(1,2) = 8;
  f(2,2) = 15;
  CMatrix<3,3> fi(inverse(f));
  CMatrix<3,3> ffi(f * fi);
  EXPECT_NEAR(1.0, real(ffi(0,0)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(0,1)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(0,2)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(1,0)), 1e-12);
  EXPECT_NEAR(1.0, real(ffi(1,1)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(1,2)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(2,0)), 1e-12);
  EXPECT_NEAR(0.0, real(ffi(2,1)), 1e-12);
  EXPECT_NEAR(1.0, real(ffi(2,2)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(0,0)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(0,1)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(0,2)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(1,0)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(1,1)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(1,2)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(2,0)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(2,1)), 1e-12);
  EXPECT_NEAR(0.0, imag(ffi(2,2)), 1e-12);

  CMatrix<10,10> g;
  for (unsigned k=0; k<10; ++k)
    for (unsigned j=0; j<10; ++j)
      g(k,j) = tan(10*k+j) * r + cos(10*k + j) * k;
  CMatrix<10,10> gi(inverse(g));
  CMatrix<10,10> ggi(g * gi);
  for (unsigned k=0; k<10; ++k)
    for (unsigned j=0; j<10; ++j) {
      EXPECT_NEAR(k == j ? 1.0 : 0.0, real(ggi(k,j)), 64*DBL_EPSILON);
      EXPECT_NEAR(0.0, imag(ggi(k,j)), 64*DBL_EPSILON);
    }
}

TEST(CircumcenterTest, Easy) {
  Array<3,Vector<2> > xs;
  xs[0][0] = 1;
  xs[0][1] = 0;
  xs[1][0] = 0;
  xs[1][1] = 1;
  xs[2][0] = -1;
  xs[2][1] = 0;
  Vector<2> c(find_circumcenter(xs));
  EXPECT_NEAR(c[0], 0.0, 4*DBL_EPSILON);
  EXPECT_NEAR(c[1], 0.0, 4*DBL_EPSILON);
}

TEST(CircumcenterTest, Hard) {
  Array<11,Vector<10> > xs;
  for (unsigned i=0; i<11; ++i)
    for (unsigned j=0; j<10; ++j)
      xs[i][j] = tan(10*i+j);
  Vector<10> c = find_circumcenter(xs);
  for (unsigned i=0; i<11; ++i)
    xs[i] = xs[i] - c;
  for (unsigned i=0; i<11; ++i)
    for (unsigned j=i+1; j<11; ++j)
      EXPECT_DOUBLE_EQ(norm_squared(xs[i]), norm_squared(xs[j]));
}

TEST(AverageDistanceSquaredTest, SomePoints) {
  Vector<2> x(0);
  Array<3,Vector<2> > ys;
  ys[0][0] = 0;
  ys[0][1] = 3; // 9
  ys[1][0] = 1;
  ys[1][1] = 3; // 10
  ys[2][0] = 2;
  ys[2][1] = 2; // 8
  EXPECT_EQ(9.0, average_distance_squared(x, ys));
}

TEST(FaceTest, SortThem) {
  Array<2,unsigned> aa;
  aa[0] = 1; aa[1] = 2;
  Face<2> a(aa);
  Face<2> b, c, d, e;
  a.points[0] = 1; a.points[1] = 2;
  b.points[0] = 1; b.points[1] = 1;
  c.points[0] = 0; c.points[1] = 5;
  d.points[0] = 2; d.points[1] = 0;
  e.points[0] = 1; e.points[1] = 0;
  std::vector<Face<2> > faces;
  faces.push_back(a);
  faces.push_back(b);
  faces.push_back(c);
  faces.push_back(d);
  faces.push_back(e);
  std::sort(faces.begin(), faces.end());
  EXPECT_EQ(faces[0], c);
  EXPECT_EQ(faces[1], e);
  EXPECT_EQ(faces[2], b);
  EXPECT_EQ(faces[3], a);
  EXPECT_EQ(faces[4], d);
}

class SimplexTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    std::vector<Vector<2> > points;
    Vector<2> a, b, c;
    a[0] = 0; a[1] = 0;
    b[0] = 6; b[1] = 0;
    c[0] = 0; c[1] = 8;
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);
    Array<3,unsigned> point_indices;
    point_indices[0] = 0;
    point_indices[1] = 1;
    point_indices[2] = 2;
    s = Simplex<2>(points, point_indices);
  }
  Simplex<2> s;
};

TEST_F(SimplexTest, Getters) {
  EXPECT_EQ(s.formingPoint(0), unsigned(0));
  EXPECT_EQ(s.formingPoint(1), unsigned(1));
  EXPECT_EQ(s.formingPoint(2), unsigned(2));
  EXPECT_EQ(s.adjacency(0), unsigned(0));
  EXPECT_EQ(s.adjacency(1), unsigned(0));
  EXPECT_EQ(s.adjacency(2), unsigned(0));
  EXPECT_EQ(s.circumcenter()[0], 3);
  EXPECT_EQ(s.circumcenter()[1], 4);
  EXPECT_EQ(s.radiusSquared(), 25);
  s.adjacency(0) = 3;
  const Simplex<2> &s_ref = s;
  EXPECT_EQ(s_ref.adjacency(0), unsigned(3));
}

TEST_F(SimplexTest, InsideCircumsphere) {
  Vector<2> x;
  x[0] = 1; x[1] = 1;
  EXPECT_EQ(s.isInsideCircumsphere(x), true);
  x[0] = -1; x[1] = -1;
  EXPECT_EQ(s.isInsideCircumsphere(x), false);
  x[0] = 1; x[1] = -1;
  EXPECT_EQ(s.isInsideCircumsphere(x), false);
  x[0] = 0; x[1] = 1;
  EXPECT_EQ(s.isInsideCircumsphere(x), true);
  x[0] = 1; x[1] = 0;
  EXPECT_EQ(s.isInsideCircumsphere(x), true);
}

TEST_F(SimplexTest, Faces) {
  Array<3,Face<2> > fs = s.faces();
  EXPECT_EQ(fs[0].points[0], unsigned(1));
  EXPECT_EQ(fs[0].points[1], unsigned(2));
  EXPECT_EQ(fs[1].points[0], unsigned(0));
  EXPECT_EQ(fs[1].points[1], unsigned(2));
  EXPECT_EQ(fs[2].points[0], unsigned(0));
  EXPECT_EQ(fs[2].points[1], unsigned(1));
}

class DelaunayTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    Array<3,Vector<2> > pts2;
    pts2[0][0] = -1;
    pts2[0][1] = -1;
    pts2[1][0] = 1;
    pts2[1][1] = -1;
    pts2[2][0] = 0;
    pts2[2][1] = 1;
    d = Delaunay<2>(pts2);

    Array<4,Vector<3> > pts3;
    pts3[0][0] = 10;
    pts3[0][1] = 10;
    pts3[0][2] = 10;
    pts3[1][0] = 10;
    pts3[1][1] = -10;
    pts3[1][2] = -10;
    pts3[2][0] = -10;
    pts3[2][1] = 10;
    pts3[2][2] = -10;
    pts3[3][0] = -10;
    pts3[3][1] = -10;
    pts3[3][2] = 10;
    t = Delaunay<3>(pts3);
  }
  Delaunay<2> d;
  Delaunay<3> t;
};

TEST_F(DelaunayTest, Construct)
{
  EXPECT_EQ(d.numPoints(), unsigned(3));
  EXPECT_EQ(d.getPoint(0)[0], -1);
  EXPECT_EQ(d.getPoint(0)[1], -1);
  EXPECT_EQ(d.getPoint(1)[0], 1);
  EXPECT_EQ(d.getPoint(1)[1], -1);
  EXPECT_EQ(d.getPoint(2)[0], 0);
  EXPECT_EQ(d.getPoint(2)[1], 1);
  EXPECT_EQ(d.maxSimplex(), unsigned(1));
  EXPECT_EQ(d.hasSimplex(0), false);
  EXPECT_EQ(d.hasSimplex(1), true);
  EXPECT_EQ(d.getSimplex(1).formingPoint(0), unsigned(0));
  EXPECT_EQ(d.getSimplex(1).formingPoint(1), unsigned(1));
  EXPECT_EQ(d.getSimplex(1).formingPoint(2), unsigned(2));
  EXPECT_EQ(d.getSimplex(1).adjacency(0), unsigned(0));
  EXPECT_EQ(d.getSimplex(1).adjacency(1), unsigned(0));
  EXPECT_EQ(d.getSimplex(1).adjacency(2), unsigned(0));
}

TEST_F(DelaunayTest, inefficientAddFirstPoint)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.inefficientAddPoint(x);
  EXPECT_EQ(e.numPoints(), unsigned(4));
  EXPECT_EQ(e.getPoint(3)[0], 0);
  EXPECT_EQ(e.getPoint(3)[1], 0);
  EXPECT_EQ(e.maxSimplex(), unsigned(4));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), true);
  EXPECT_EQ(e.getSimplex(2).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(2).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(2).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(2).adjacency(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(3), true);
  EXPECT_EQ(e.getSimplex(3).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(3).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(3).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(3).adjacency(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(4), true);
  EXPECT_EQ(e.getSimplex(4).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(4).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(0), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).adjacency(2), unsigned(0));
}

TEST_F(DelaunayTest, addFirstPoint)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.addPoint(x, 1);
  EXPECT_EQ(e.numPoints(), unsigned(4));
  EXPECT_EQ(e.getPoint(3)[0], 0);
  EXPECT_EQ(e.getPoint(3)[1], 0);
  EXPECT_EQ(e.maxSimplex(), unsigned(4));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), true);
  EXPECT_EQ(e.getSimplex(2).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(2).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(2).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(2).adjacency(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(3), true);
  EXPECT_EQ(e.getSimplex(3).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(3).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(3).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(3).adjacency(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(4), true);
  EXPECT_EQ(e.getSimplex(4).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(4).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(0), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).adjacency(2), unsigned(0));
}

TEST_F(DelaunayTest, inefficientAddSecondPoint1)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.inefficientAddPoint(x);
  Vector<2> y(0);
  y[1] = -0.5;
  e.inefficientAddPoint(y);
  EXPECT_EQ(e.numPoints(), unsigned(5));
  EXPECT_EQ(e.getPoint(4)[0], 0);
  EXPECT_EQ(e.getPoint(4)[1], -0.5);
  EXPECT_EQ(e.maxSimplex(), unsigned(7));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), false);
  EXPECT_EQ(e.hasSimplex(3), true);
  EXPECT_EQ(e.getSimplex(3).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(3).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(3).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(3).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(3).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(4), true);
  EXPECT_EQ(e.getSimplex(4).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(4).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(0), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(1), unsigned(7));
  EXPECT_EQ(e.getSimplex(4).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(5), true);
  EXPECT_EQ(e.getSimplex(5).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(5).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(5).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(5).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(5).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(5).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(6), true);
  EXPECT_EQ(e.getSimplex(6).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(6).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(6).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(6).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(6).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(6).adjacency(2), unsigned(3));
  EXPECT_EQ(e.hasSimplex(7), true);
  EXPECT_EQ(e.getSimplex(7).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(7).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(7).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(7).adjacency(0), unsigned(6));
  EXPECT_EQ(e.getSimplex(7).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(7).adjacency(2), unsigned(4));
}

TEST_F(DelaunayTest, addSecondPoint1)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.addPoint(x, 1);
  Vector<2> y(0);
  y[1] = -0.5;
  e.addPoint(y, 2);
  EXPECT_EQ(e.numPoints(), unsigned(5));
  EXPECT_EQ(e.getPoint(4)[0], 0);
  EXPECT_EQ(e.getPoint(4)[1], -0.5);
  EXPECT_EQ(e.maxSimplex(), unsigned(7));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), false);
  EXPECT_EQ(e.hasSimplex(3), true);
  EXPECT_EQ(e.getSimplex(3).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(3).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(3).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(3).adjacency(0), unsigned(4));
  EXPECT_EQ(e.getSimplex(3).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(3).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(4), true);
  EXPECT_EQ(e.getSimplex(4).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(4).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(4).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(0), unsigned(3));
  EXPECT_EQ(e.getSimplex(4).adjacency(1), unsigned(7));
  EXPECT_EQ(e.getSimplex(4).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(5), true);
  EXPECT_EQ(e.getSimplex(5).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(5).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(5).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(5).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(5).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(5).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(6), true);
  EXPECT_EQ(e.getSimplex(6).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(6).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(6).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(6).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(6).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(6).adjacency(2), unsigned(3));
  EXPECT_EQ(e.hasSimplex(7), true);
  EXPECT_EQ(e.getSimplex(7).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(7).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(7).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(7).adjacency(0), unsigned(6));
  EXPECT_EQ(e.getSimplex(7).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(7).adjacency(2), unsigned(4));
}

TEST_F(DelaunayTest, inefficientAddSecondPoint2)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.inefficientAddPoint(x);
  Vector<2> y(0);
  y[1] = 0.5;
  e.inefficientAddPoint(y);
  EXPECT_EQ(e.numPoints(), unsigned(5));
  EXPECT_EQ(e.getPoint(4)[0], 0);
  EXPECT_EQ(e.getPoint(4)[1], 0.5);
  EXPECT_EQ(e.maxSimplex(), unsigned(8));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), true);
  EXPECT_EQ(e.getSimplex(2).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(2).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(2).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(2).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(2).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(3), false);
  EXPECT_EQ(e.hasSimplex(4), false);
  EXPECT_EQ(e.hasSimplex(5), true);
  EXPECT_EQ(e.getSimplex(5).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(5).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(5).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(5).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(5).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(5).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(6), true);
  EXPECT_EQ(e.getSimplex(6).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(6).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(6).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(6).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(6).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(6).adjacency(2), unsigned(2));
  EXPECT_EQ(e.hasSimplex(7), true);
  EXPECT_EQ(e.getSimplex(7).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(7).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(7).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(7).adjacency(0), unsigned(5));
  EXPECT_EQ(e.getSimplex(7).adjacency(1), unsigned(8));
  EXPECT_EQ(e.getSimplex(7).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(8), true);
  EXPECT_EQ(e.getSimplex(8).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(8).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(8).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(8).adjacency(0), unsigned(6));
  EXPECT_EQ(e.getSimplex(8).adjacency(1), unsigned(7));
  EXPECT_EQ(e.getSimplex(8).adjacency(2), unsigned(2));
}

TEST_F(DelaunayTest, addSecondPoint2A)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.addPoint(x, 1);
  Vector<2> y(0);
  y[1] = 0.5;
  e.addPoint(y, 3);
  EXPECT_EQ(e.numPoints(), unsigned(5));
  EXPECT_EQ(e.getPoint(4)[0], 0);
  EXPECT_EQ(e.getPoint(4)[1], 0.5);
  EXPECT_EQ(e.maxSimplex(), unsigned(8));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), true);
  EXPECT_EQ(e.getSimplex(2).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(2).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(2).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(2).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(2).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(3), false);
  EXPECT_EQ(e.hasSimplex(4), false);
  EXPECT_EQ(e.hasSimplex(5), true);
  EXPECT_EQ(e.getSimplex(5).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(5).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(5).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(5).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(5).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(5).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(6), true);
  EXPECT_EQ(e.getSimplex(6).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(6).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(6).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(6).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(6).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(6).adjacency(2), unsigned(2));
  EXPECT_EQ(e.hasSimplex(7), true);
  EXPECT_EQ(e.getSimplex(7).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(7).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(7).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(7).adjacency(0), unsigned(5));
  EXPECT_EQ(e.getSimplex(7).adjacency(1), unsigned(8));
  EXPECT_EQ(e.getSimplex(7).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(8), true);
  EXPECT_EQ(e.getSimplex(8).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(8).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(8).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(8).adjacency(0), unsigned(6));
  EXPECT_EQ(e.getSimplex(8).adjacency(1), unsigned(7));
  EXPECT_EQ(e.getSimplex(8).adjacency(2), unsigned(2));
}

TEST_F(DelaunayTest, addSecondPoint2B)
{
  Delaunay<2> e = d;
  Vector<2> x(0);
  e.addPoint(x, 1);
  Vector<2> y(0);
  y[1] = 0.5;
  e.addPoint(y, 4);
  EXPECT_EQ(e.numPoints(), unsigned(5));
  EXPECT_EQ(e.getPoint(4)[0], 0);
  EXPECT_EQ(e.getPoint(4)[1], 0.5);
  EXPECT_EQ(e.maxSimplex(), unsigned(8));
  EXPECT_EQ(e.hasSimplex(1), false);
  EXPECT_EQ(e.hasSimplex(2), true);
  EXPECT_EQ(e.getSimplex(2).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(2).formingPoint(1), unsigned(1));
  EXPECT_EQ(e.getSimplex(2).formingPoint(2), unsigned(3));
  EXPECT_EQ(e.getSimplex(2).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(2).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(2).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(3), false);
  EXPECT_EQ(e.hasSimplex(4), false);
  EXPECT_EQ(e.hasSimplex(5), true);
  EXPECT_EQ(e.getSimplex(5).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(5).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(5).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(5).adjacency(0), unsigned(7));
  EXPECT_EQ(e.getSimplex(5).adjacency(1), unsigned(6));
  EXPECT_EQ(e.getSimplex(5).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(6), true);
  EXPECT_EQ(e.getSimplex(6).formingPoint(0), unsigned(0));
  EXPECT_EQ(e.getSimplex(6).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(6).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(6).adjacency(0), unsigned(8));
  EXPECT_EQ(e.getSimplex(6).adjacency(1), unsigned(5));
  EXPECT_EQ(e.getSimplex(6).adjacency(2), unsigned(2));
  EXPECT_EQ(e.hasSimplex(7), true);
  EXPECT_EQ(e.getSimplex(7).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(7).formingPoint(1), unsigned(2));
  EXPECT_EQ(e.getSimplex(7).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(7).adjacency(0), unsigned(5));
  EXPECT_EQ(e.getSimplex(7).adjacency(1), unsigned(8));
  EXPECT_EQ(e.getSimplex(7).adjacency(2), unsigned(0));
  EXPECT_EQ(e.hasSimplex(8), true);
  EXPECT_EQ(e.getSimplex(8).formingPoint(0), unsigned(1));
  EXPECT_EQ(e.getSimplex(8).formingPoint(1), unsigned(3));
  EXPECT_EQ(e.getSimplex(8).formingPoint(2), unsigned(4));
  EXPECT_EQ(e.getSimplex(8).adjacency(0), unsigned(6));
  EXPECT_EQ(e.getSimplex(8).adjacency(1), unsigned(7));
  EXPECT_EQ(e.getSimplex(8).adjacency(2), unsigned(2));
}

TEST_F(DelaunayTest, Add1000PointsShouldNotThrow)
{
  Delaunay<2> e = d;
  Vector<2> x;
  srand(0);
  for (int i=0; i<1000; ++i) {
    x[0] = 0.25 * double(rand()) / double(RAND_MAX);
    x[1] = 0.25 * double(rand()) / double(RAND_MAX);
    e.inefficientAddPoint(x);
  }
  // The real test is that the above loop doesn't throw any exceptions
  EXPECT_EQ(e.numPoints(), unsigned(1003));
}

bool operator<(const Vector<3> &x, const Vector<3> &y)
{
  if (x[0] < y[0]) return false;
  if (x[0] > y[0]) return true;
  if (x[1] < y[1]) return false;
  if (x[1] > y[1]) return true;
  if (x[2] < y[2]) return false;
  if (x[2] > y[2]) return true;
  return false;
}

union double_as_long {
  double f;
  unsigned long i;
};

double prev_d(double x)
{
  double_as_long dal;
  dal.f = x;
  if (x == 0.0)
    dal.i = 0x8000000000000001;
  else
    dal.i -= 1;
  return dal.f;
}

double next_d(double x)
{
  double_as_long dal;
  dal.f = x;
  dal.i += 1;
  return dal.f;
}

TEST_F(DelaunayTest, AddStructuredPointsShouldNotThrow)
{
  Delaunay<3> u(t);
  Vector<3> x;
  srand(0);
  set<Vector<3> > s;
  for (int i = 0; i < 1000; ++i) {
    x[0] = double(rand() % 1025 - 512) / double(512.0);
    x[1] = double(rand() % 1025 - 512) / double(512.0);
    x[2] = double(rand() % 1025 - 512) / double(512.0);
    if (s.find(x) == s.end()) {
      s.insert(x);
      switch (rand() % 4) {
      case 0:
        x[0] = next_d(x[0]);
        break;
      case 1:
        x[0] = prev_d(x[0]);
        break;
      }
      switch (rand() % 4) {
      case 0:
        x[1] = next_d(x[1]);
        break;
      case 1:
        x[1] = prev_d(x[1]);
        break;
      }
      switch (rand() % 4) {
      case 0:
        x[2] = next_d(x[2]);
        break;
      case 1:
        x[2] = prev_d(x[2]);
        break;
      }
      u.inefficientAddPoint(x);
    }
  }
}

TEST_F(DelaunayTest, Add1000PointsShouldNotThrowInThreeDimensions)
{
  Delaunay<3> u(t);
  Vector<3> x;
  srand(0);
  for (int i=0; i<1000; ++i) {
    x[0] = 2.0 * double(rand()) / double(RAND_MAX) - 1.0;
    x[1] = 2.0 * double(rand()) / double(RAND_MAX) - 1.0;
    x[1] = 2.0 * double(rand()) / double(RAND_MAX) - 1.0;
    u.inefficientAddPoint(x);
  }
  // The real test is that the above loop doesn't throw any exceptions
  EXPECT_EQ(u.numPoints(), unsigned(1004));
}

TEST_F(DelaunayTest, AddingPointsOnARegularGridShouldNotThrowInThreeDimensions)
{
  Delaunay<3> u(t);
  Vector<3> x;
  x[0] = 0.1457;
  x[1] = -0.07623;
  x[2] = 1.0;
  u.inefficientAddPoint(x);
  x[0] = 0.03895;
  x[1] = -0.104673;
  x[2] = -1.0;
  u.inefficientAddPoint(x);
  x[2] = 0.0;
  const int k = 10;
  for (int i = -k; i <= k; ++i)
    for (int j = -k; j <= k; ++j) {
      x[0] = double(i) / double(k);
      x[1] = double(j) / double(k);
      u.inefficientAddPoint(x);
    }
  EXPECT_EQ(u.numPoints(), unsigned((2*k+1)*(2*k+1)+6));
}

class TestFunction : public RealFunction<2>
{
public:
  double operator()(const Vector<2> &x) const
  {
    // f(x,y) = x * y^2
    return x[0] * x[1] * x[1];
  }
};

TEST(FunctionTest, NumericalDerivativeTest)
{
  TestFunction f;
  Vector<2> x;
  x[0] = 2.0;
  x[1] = 3.0;
  Vector<2> grad = f.gradient(x);
  EXPECT_NEAR(grad[0],  9.0, 5e-11);
  EXPECT_NEAR(grad[1], 12.0, 5e-11);
  Matrix<2,2> hess = f.hessian(x);
  EXPECT_NEAR(hess(0,0),  0.0, 1e-7);
  EXPECT_NEAR(hess(0,1),  6.0, 1e-7);
  EXPECT_NEAR(hess(1,0),  6.0, 1e-7);
  EXPECT_NEAR(hess(1,1),  4.0, 1e-7);
}

TEST(PolynomialTest, ConstructEmpty) {
  Polynomial p;
}

TEST(PolynomialTest, EvaluateEmpty) {
  const Polynomial p;
  EXPECT_EQ(p(0.), 0.);
  EXPECT_EQ(p(7.), 0.);
  EXPECT_EQ(p(-33.), 0.);
}

TEST(PolynomialTest, ConstructConstant) {
  Polynomial p(3.);
}

TEST(PolynomialTest, EvaluateConstant) {
  const Polynomial p(3.);
  EXPECT_EQ(p(0.), 3.);
  EXPECT_EQ(p(7.), 3.);
  EXPECT_EQ(p(-33.), 3.);
}

TEST(PolynomialTest, ConstructLinear) {
  Polynomial p(3., 1);
}

TEST(PolynomialTest, EvaluateLinear) {
  const Polynomial p(3., 1);
  EXPECT_EQ(p(0.), 0.);
  EXPECT_EQ(p(7.), 21.);
  EXPECT_EQ(p(-33.), -99.);
}

TEST(PolynomialTest, ConstructSquared) {
  Polynomial p(5., 2);
}

TEST(PolynomialTest, EvaluateSquared) {
  const Polynomial p(5., 2);
  EXPECT_EQ(p(0.), 0.);
  EXPECT_EQ(p(7.), 245.);
  EXPECT_EQ(p(-11.), 605.);
}

TEST(PolynomialTest, SimpleAdditionTest) {
  const Polynomial p(1., 2);
  const Polynomial q(3., 1);
  const Polynomial r = p + q;
  EXPECT_EQ(r(0.), p(0.) + q(0.));
  EXPECT_EQ(r(3.), p(3.) + q(3.));
  EXPECT_EQ(r(8.), p(8.) + q(8.));
}

TEST(PolynomialTest, ComplexAdditionTest) {
  // x^2+4x
  const Polynomial p = Polynomial(1., 2) + Polynomial(4., 1);
  const Polynomial q = Polynomial(2., 2) + Polynomial(6.);
  const Polynomial r = p + q;
  EXPECT_EQ(r(0.), p(0.) + q(0.));
  EXPECT_EQ(r(3.), p(3.) + q(3.));
  EXPECT_EQ(r(8.), p(8.) + q(8.));
}

TEST(PolynomialTest, MultiplyMonomials) {
  // x * x
  const Polynomial p(1., 1);
  const Polynomial q = p * p;
  EXPECT_EQ(q(0.), p(0.) * p(0.));
  EXPECT_EQ(q(5.), p(5.) * p(5.));
  EXPECT_EQ(q(7.), p(7.) * p(7.));
}

TEST(PolynomialTest, MultiplyAffine) {
  // (x+3) * (x-2)
  const Polynomial x(1., 1);
  const Polynomial p = x + 3.;
  const Polynomial q = x - 2.;
  const Polynomial r = p * q;
  EXPECT_EQ(r(0.), p(0.) * q(0.));
  EXPECT_EQ(r(5.), p(5.) * q(5.));
  EXPECT_EQ(r(7.), p(7.) * q(7.));
}

TEST(PolynomialTest, PowPoly) {
  // (2x+3)^4
  const Polynomial x(1., 1);
  const Polynomial p = 2. * x + 3.;
  const Polynomial q = pow(p, 4);
  EXPECT_EQ(pow(p(0.), 4), q(0.));
  EXPECT_EQ(pow(p(3.), 4), q(3.));
  EXPECT_EQ(pow(p(-2.), 4), q(-2.));
}

TEST(PolynomialTest, MixPolynomialsWithDoubles)
{
  const Polynomial x(1., 1);
  const Polynomial f1(x + 1.);
  const Polynomial f2(x - 1.);
  const Polynomial f3(1. + x);
  const Polynomial f4(1. - x);
  const Polynomial f5(-1.);
  const Polynomial f6(-x);
  const Polynomial f7(+1.);
  const Polynomial f8(+x);
  const Polynomial f9(x * 2.);
  const Polynomial f10(2. * x);
  const Polynomial f11(x / 2.);
  Polynomial f12;
  f12 = 1.;
}

TEST(PolynomialTest, SimpleDerivative)
{
  const Polynomial x(1., 1);
  const Polynomial f = x * x;
  const Polynomial df = f.derivative();
  const Polynomial g = 2. * x;
  EXPECT_EQ(df(0.), g(0.));
  EXPECT_EQ(df(7.), g(7.));
  EXPECT_EQ(df(9.), g(9.));
}

TEST(PolynomialTest, Derivative)
{
  const Polynomial x(1., 1);
  const Polynomial f = pow(x + 1., 4);
  const Polynomial df = f.derivative();
  const Polynomial g = 4. * x * x * x + 12. * x * x + 12. * x + 4.;
  EXPECT_EQ(df(0.), g(0.));
  EXPECT_EQ(df(7.), g(7.));
  EXPECT_EQ(df(9.), g(9.));
}

TEST(PolynomialTest, SecondDerivative)
{
  const Polynomial x(1., 1);
  const Polynomial f = pow(x + 1., 4);
  const Polynomial ddf = f.derivative(2);
  const Polynomial g = 12. * x * x + 24. * x + 12.;
  EXPECT_EQ(ddf(0.), g(0.));
  EXPECT_EQ(ddf(7.), g(7.));
  EXPECT_EQ(ddf(9.), g(9.));
}

TEST(PolynomialTest, ThirdDerivative)
{
  const Polynomial x(1., 1);
  const Polynomial f = pow(x + 1., 4);
  const Polynomial dddf = f.derivative(3);
  const Polynomial g = 24. * x + 24.;
  EXPECT_EQ(dddf(0.), g(0.));
  EXPECT_EQ(dddf(7.), g(7.));
  EXPECT_EQ(dddf(9.), g(9.));
}

TEST(PolynomialTest, OperatorEquals)
{
  Polynomial x(1., 1), x2(1., 2);
  Polynomial y1(x);
  y1 += x2;
  EXPECT_EQ(y1(7), (x + x2)(7));
  Polynomial y2(x);
  y2 -= x2;
  EXPECT_EQ(y2(7), (x - x2)(7));
  Polynomial y3(x);
  y3 *= x2;
  EXPECT_EQ(y3(7), (x * x2)(7));
  Polynomial y4(x);
  y4 /= 2.0;
  EXPECT_EQ(y4(7), (x / 2.0)(7));
}

TEST(PolynomialTest, PolynomialsActLikeVectors)
{
  Polynomial f, g;
  f += g;
  g = -f;
  f *= 2.0;
  g = +f;
  f = f + g;
  f -= g;
  f = f - g;
  f = f * 2.0;
  f = 2.0 * f;
  f /= 2.0;
  f = f / 2.0;
}

TEST(PolynomialTest, AddScalars)
{
  Polynomial x(1., 1);
  Polynomial f;
  f = 2.;
  f += x;
  f += 1.;
  EXPECT_EQ(f(1.), 4.0);
  f = f + 1.;
  EXPECT_EQ(f(1.), 5.0);
  f = 1. + f;
  EXPECT_EQ(f(1.), 6.0);
}

TEST(PolynomialTest, SubtractScalars)
{
  Polynomial x(1., 1);
  Polynomial f;
  f = 2.;
  f += x;
  f -= 1.;
  EXPECT_EQ(f(1.), 2.0);
  f = f - 1.;
  EXPECT_EQ(f(1.), 1.0);
  f = 5. - f;
  EXPECT_EQ(f(1.), 4.0);
}

TEST(PolynomialTest, CommutativeAlgebraTest)
{
  Polynomial x(1., 1);
  Polynomial f(x);
  f *= x;
  EXPECT_EQ(f(3.), 9.);
}

inline bool operator==(const Quaternion &x, const Quaternion &y)
{
  return
    x.real() == y.real() && x.imag() == y.imag() &&
    x.jmag() == y.jmag() && x.kmag() == y.kmag();
}

class QuaternionTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    r = Quaternion(1.0);
    i = Quaternion(0.0, 1.0, 0.0, 0.0);
    j = Quaternion(0.0, 0.0, 1.0, 0.0);
    k = Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  Quaternion r, i, j, k;
};

TEST_F(QuaternionTest, PlusEquals)
{
  Quaternion x(r);
  x += i;
  x += i;
  x += j;
  x += j;
  x += j;
  x += k;
  x += k;
  x += k;
  x += k;
  EXPECT_EQ(x, Quaternion(1.0, 2.0, 3.0, 4.0));
}

TEST_F(QuaternionTest, Negate)
{
  EXPECT_EQ(-r, Quaternion(-1.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(-i, Quaternion(0.0, -1.0, 0.0, 0.0));
  EXPECT_EQ(-j, Quaternion(0.0, 0.0, -1.0, 0.0));
  EXPECT_EQ(-k, Quaternion(0.0, 0.0, 0.0, -1.0));
}

TEST_F(QuaternionTest, ScalarTimesEquals)
{
  Quaternion x(1.0, 2.0, 3.0, 4.0);
  x *= 3.0;
  EXPECT_EQ(x, Quaternion(3.0, 6.0, 9.0, 12.0));
}

TEST_F(QuaternionTest, Multiply)
{
  EXPECT_EQ(r * r, r);
  EXPECT_EQ(r * i, i);
  EXPECT_EQ(r * j, j);
  EXPECT_EQ(r * k, k);
  EXPECT_EQ(i * r, i);
  EXPECT_EQ(i * i, -r);
  EXPECT_EQ(i * j, k);
  EXPECT_EQ(i * k, -j);
  EXPECT_EQ(j * r, j);
  EXPECT_EQ(j * i, -k);
  EXPECT_EQ(j * j, -r);
  EXPECT_EQ(j * k, i);
  EXPECT_EQ(k * r, k);
  EXPECT_EQ(k * i, j);
  EXPECT_EQ(k * j, -i);
  EXPECT_EQ(k * k, -r);
}

TEST_F(QuaternionTest, Norm)
{
  EXPECT_EQ(norm_squared(r + i + j + k), 4.0);
  EXPECT_EQ(norm(r + i + j + k), 2.0);
  EXPECT_EQ(norm_squared(3.0 * r + 4.0 * i + 12.0 * j + 84.0 * k), 7225.0);
  EXPECT_EQ(norm(3.0 * r + 4.0 * i + 12.0 * j + 84.0 * k), 85.0);
}

TEST(WaveFunctionTest, FactorialTest)
{
  EXPECT_EQ(factorial(0), 1.);
  EXPECT_EQ(factorial(1), 1.);
  EXPECT_EQ(factorial(2), 2.);
  EXPECT_EQ(factorial(3), 6.);
  EXPECT_EQ(factorial(4), 24.);
  EXPECT_EQ(factorial(5), 120.);
  EXPECT_EQ(factorial(6), 720.);
  EXPECT_EQ(factorial(7), 5040.);
  EXPECT_EQ(factorial(8), 40320.);
}
