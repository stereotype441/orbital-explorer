# This file is part of the Electron Orbital Explorer. The Electron
# Orbital Explorer is distributed under the Simplified BSD License
# (also called the "BSD 2-Clause License"), in hopes that these
# rendering techniques might be used by other programmers in
# applications such as scientific visualization, video gaming, and so
# on. If you find value in this software and use its technologies for
# another purpose, I would love to hear back from you at bjthinks (at)
# gmail (dot) com. If you improve this software and agree to release
# your modifications under the below license, I encourage you to fork
# the development tree on github and push your modifications. The
# Electron Orbital Explorer's development URL is:
# https://github.com/bjthinks/orbital-explorer
# (This paragraph is not part of the software license and may be
# removed.)
#
# Copyright (c) 2013, Brian W. Johnson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# + Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# + Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from radial_analyzer import *
import unittest


class TestPolynomial(unittest.TestCase):

    def testPolynomial(self):
        z = Polynomial()
        n = Polynomial(1)
        x = Polynomial(1, 1)

        self.assertEqual(z, z)
        self.assertEqual(n, n)
        self.assertEqual(x, x)
        self.assertNotEqual(z, n)
        self.assertNotEqual(z, x)
        self.assertNotEqual(n, x)

        self.assertEqual(z, Polynomial(0))
        self.assertEqual(z, Polynomial(0, 0))
        self.assertEqual(z, Polynomial(0.0))
        self.assertEqual(z, Polynomial(0.0, 0))
        self.assertEqual(n, Polynomial(1, 0))
        self.assertEqual(n, Polynomial(1.0, 0))
        self.assertEqual(x, Polynomial(1.0, 1))

        self.assertEqual(z, Polynomial(0, 3))
        self.assertNotEqual(n, Polynomial(0, 3))
        self.assertNotEqual(x, Polynomial(0, 3))

        self.assertNotEqual(z, Polynomial(3, 0))
        self.assertNotEqual(n, Polynomial(3, 0))
        self.assertNotEqual(x, Polynomial(3, 0))

        self.assertNotEqual(z, Polynomial(3, 1))
        self.assertNotEqual(n, Polynomial(3, 1))
        self.assertNotEqual(x, Polynomial(3, 1))

        self.assertNotEqual(z, Polynomial(1, 3))
        self.assertNotEqual(n, Polynomial(1, 3))
        self.assertNotEqual(x, Polynomial(1, 3))

        self.assertEqual(Polynomial(6), Polynomial(6, 0))
        self.assertNotEqual(Polynomial(6), Polynomial(1, 1))
        self.assertNotEqual(Polynomial(6), Polynomial(1, 6))
        self.assertNotEqual(Polynomial(6), Polynomial(6, 1))
        self.assertNotEqual(Polynomial(6), Polynomial(6, 6))

        self.assertEqual(Polynomial().degree, -1)
        self.assertEqual(Polynomial(0).degree, -1)
        self.assertEqual(Polynomial(6).degree, 0)
        self.assertEqual(Polynomial(4, 0).degree, 0)
        self.assertEqual(Polynomial(3, 1).degree, 1)
        self.assertEqual(Polynomial(1, 3).degree, 3)
        self.assertEqual(Polynomial(0, 1).degree, -1)
        self.assertEqual(Polynomial(0, 5).degree, -1)

        self.assertEqual(z(0), 0)
        self.assertEqual(n(0), 1)
        self.assertEqual(x(0), 0)

        self.assertEqual(z(1), 0)
        self.assertEqual(n(1), 1)
        self.assertEqual(x(1), 1)

        self.assertEqual(z(3), 0)
        self.assertEqual(n(3), 1)
        self.assertEqual(x(3), 3)

        x2 = Polynomial(1, 2)
        self.assertEqual(x2(0), 0)
        self.assertEqual(x2(1), 1)
        self.assertEqual(x2(3), 9)

        x3 = Polynomial(1, 3)
        self.assertEqual(x3(0), 0)
        self.assertEqual(x3(1), 1)
        self.assertEqual(x3(3), 27)

        self.assertEqual(Polynomial(5, 0)(0), 5)
        self.assertEqual(Polynomial(5, 0)(1), 5)
        self.assertEqual(Polynomial(5, 0)(3), 5)

        self.assertEqual(Polynomial(5, 1)(0), 0)
        self.assertEqual(Polynomial(5, 1)(1), 5)
        self.assertEqual(Polynomial(5, 1)(3), 15)

        self.assertEqual(Polynomial(5, 2)(0), 0)
        self.assertEqual(Polynomial(5, 2)(1), 5)
        self.assertEqual(Polynomial(5, 2)(3), 45)

        xp1 = x + n
        self.assertEqual(xp1(0), 1)
        self.assertEqual(xp1(1), 2)
        self.assertEqual(xp1(3), 4)

        self.assertEqual((n + x)(0), 1)
        self.assertEqual((n + x)(1), 2)
        self.assertEqual((n + x)(3), 4)

        self.assertEqual((x3 + x + n)(0), 1)
        self.assertEqual((x3 + x + n)(1), 3)
        self.assertEqual((x3 + x + n)(3), 31)

        self.assertEqual((x + 1)(3), 4)
        self.assertEqual((x + 1.5)(3), 4.5)

        self.assertEqual((1 + x)(3), 4)
        self.assertEqual((1.5 + x)(3), 4.5)

        self.assertEqual(z, +z)
        self.assertEqual(n, +n)
        self.assertEqual(x, +x)

        self.assertEqual(-z, z)
        self.assertEqual(-n, Polynomial(-1, 0))
        self.assertEqual(-x, Polynomial(-1, 1))

        self.assertEqual((-(x3 + x + n))(0), -1)
        self.assertEqual((-(x3 + x + n))(1), -3)
        self.assertEqual((-(x3 + x + n))(3), -31)

        xm1 = x - 1
        self.assertEqual(xm1(0), -1)
        self.assertEqual(xm1(1), 0)
        self.assertEqual(xm1(3), 2)

        self.assertEqual(n - n, z)
        self.assertEqual(x - x, z)

        self.assertEqual(n + (-n), z)
        self.assertEqual(x + (-x), z)

        self.assertEqual((x3 + x) - x3, x)
        self.assertEqual((-x3 + n) + x3, n)
        self.assertEqual((x2 - x3) + x3, x2)
        self.assertEqual((x3 - x2) + x2, x3)

        self.assertEqual(x - n, x - 1)
        self.assertEqual(1 - x, n - x)

        self.assertEqual(x - n, x - 1.0)
        self.assertEqual(1.0 - x, n - x)

        self.assertEqual(z * z,  z)
        self.assertEqual(z * n,  z)
        self.assertEqual(z * x,  z)
        self.assertEqual(z * x2, z)
        self.assertEqual(z * x3, z)

        self.assertEqual(n * z,  z)
        self.assertEqual(n * n,  n)
        self.assertEqual(n * x,  x)
        self.assertEqual(n * x2, x2)
        self.assertEqual(n * x3, x3)

        self.assertEqual(x * z,  z)
        self.assertEqual(x * n,  x)
        self.assertEqual(x * x,  x2)
        self.assertEqual(x * x2, x3)
        self.assertEqual(x * x3, Polynomial(1, 4))

        self.assertEqual(x2 * z,  z)
        self.assertEqual(x2 * n,  x2)
        self.assertEqual(x2 * x,  x3)
        self.assertEqual(x2 * x2, Polynomial(1, 4))
        self.assertEqual(x2 * x3, Polynomial(1, 5))

        self.assertEqual(x3 * z,  z)
        self.assertEqual(x3 * n,  x3)
        self.assertEqual(x3 * x,  Polynomial(1, 4))
        self.assertEqual(x3 * x2, Polynomial(1, 5))
        self.assertEqual(x3 * x3, Polynomial(1, 6))

        self.assertEqual(2 * x, Polynomial(2, 1))
        self.assertEqual(x * 2, Polynomial(2, 1))

        self.assertEqual(3 * (x3 * 2) * 4 * x, Polynomial(24, 4))

        self.assertEqual((x + 1) * (x + 1), x2 + 2 * x + 1)
        self.assertEqual((x + 1.0) * (x + 1), x2 + 2 * x + 1)
        self.assertEqual((x + 1) * (x + 1), x2 + 2.0 * x + 1)

        self.assertEqual((x + 1) * (x - 1), x2 - 1)
        self.assertEqual((x2 + 1) * (x - 1), x3 - x2 + x - 1)

        self.assertEqual(x / 2, 0.5 * x)
        self.assertEqual(((x + 1) ** 4) / 4,
                         0.25 * x ** 4 + x3 + 1.5 * x2 + x + 0.25)

        self.assertEqual(z ** 0, n)
        self.assertEqual(z ** 1, z)
        self.assertEqual(z ** 2, z)
        self.assertEqual(z ** 3, z)

        self.assertEqual(n ** 0, n)
        self.assertEqual(n ** 1, n)
        self.assertEqual(n ** 2, n)
        self.assertEqual(n ** 3, n)

        self.assertEqual(x ** 0, n)
        self.assertEqual(x ** 1, x)
        self.assertEqual(x ** 2, x2)
        self.assertEqual(x ** 3, x3)

        self.assertEqual(x2 ** 0, n)
        self.assertEqual(x2 ** 1, x2)
        self.assertEqual(x2 ** 2, Polynomial(1, 4))
        self.assertEqual(x2 ** 3, Polynomial(1, 6))

        self.assertEqual((3 * x) ** 0, n)
        self.assertEqual((3 * x) ** 1, 3 * x)
        self.assertEqual((3 * x) ** 2, 9 * x2)
        self.assertEqual((3 * x) ** 3, 27 * x3)

        self.assertEqual((2 * x + 3) ** 0, n)
        self.assertEqual((2 * x + 3) ** 1, 2 * x + 3)
        self.assertEqual((2 * x + 3) ** 2, 4 * x2 + 12 * x + 9)
        self.assertEqual((2 * x + 3) ** 3, 8 * x3 + 36 * x2 + 54 * x + 27)

        self.assertRaises(ArithmeticError, x.__pow__, -1)

        self.assertEqual(z.derivative(), z)
        self.assertEqual(n.derivative(), z)
        self.assertEqual(x.derivative(), n)
        self.assertEqual(x2.derivative(), 2 * x)
        self.assertEqual(x3.derivative(), 3 * x2)
        self.assertEqual(((x + 1) ** 4).derivative(),
                         4 * x3 + 12 * x2 + 12 * x + 4)

        self.assertEqual(z.leadingCoefficient, 0)
        self.assertEqual(z.constantTerm, 0)
        self.assertEqual(n.leadingCoefficient, 1)
        self.assertEqual(n.constantTerm, 1)
        self.assertEqual(x.leadingCoefficient, 1)
        self.assertEqual(x.constantTerm, 0)
        self.assertEqual((3 * x + 5).leadingCoefficient, 3)
        self.assertEqual((3 * x + 5).constantTerm, 5)

        # These come last, to check for inadvertent modifications
        self.assertEqual(z, Polynomial(0, 0))
        self.assertEqual(n, Polynomial(1, 0))
        self.assertEqual(x, Polynomial(1, 1))
        self.assertEqual(x2, Polynomial(1, 2))
        self.assertEqual(x3, Polynomial(1, 3))

        # Also test the list constructor
        coeffs = [1, 2, 3]
        f = Polynomial(coeffs)
        self.assertEqual(f, 1 + 2 * x + 3 * x2)
        coeffs.append(4)
        g = Polynomial(coeffs)
        self.assertEqual(f, 1 + 2 * x + 3 * x2)
        self.assertEqual(g, 1 + 2 * x + 3 * x2 + 4 * x3)
        self.assertEqual(Polynomial([0, 0]), Polynomial())
        self.assertEqual(Polynomial([0]), Polynomial())
        self.assertEqual(Polynomial([]), Polynomial())


class TestCombinatorics(unittest.TestCase):

    def testCombinatorics(self):
        self.assertEqual(factorial(0), 1)
        self.assertEqual(factorial(1), 1)
        self.assertEqual(factorial(2), 2)
        self.assertEqual(factorial(3), 6)
        self.assertEqual(factorial(4), 24)
        self.assertEqual(factorial(5), 120)
        self.assertRaises(ArithmeticError, factorial, -1)

        self.assertEqual(choose(0, 0), 1)
        self.assertEqual(choose(1, 0), 1)
        self.assertEqual(choose(1, 1), 1)
        self.assertEqual(choose(2, 0), 1)
        self.assertEqual(choose(2, 1), 2)
        self.assertEqual(choose(2, 2), 1)
        self.assertEqual(choose(3, 0), 1)
        self.assertEqual(choose(3, 1), 3)
        self.assertEqual(choose(3, 2), 3)
        self.assertEqual(choose(3, 3), 1)
        self.assertEqual(choose(4, 0), 1)
        self.assertEqual(choose(4, 1), 4)
        self.assertEqual(choose(4, 2), 6)
        self.assertEqual(choose(4, 3), 4)
        self.assertEqual(choose(4, 4), 1)
        self.assertEqual(choose(200, 100),
            90548514656103281165404177077484163874504589675413336841320)


class TestLaguerre(unittest.TestCase):

    def testLaguerre(self):
        x = Polynomial(1, 1)

        self.assertEqual(laguerre(0, 0), Polynomial(1))
        self.assertEqual(laguerre(0, 1), Polynomial(1))
        self.assertEqual(laguerre(0, 2), Polynomial(1))
        self.assertEqual(laguerre(0, 3), Polynomial(1))

        self.assertEqual(laguerre(1, 0), -x + 1)
        self.assertEqual(laguerre(1, 1), -x + 2)
        self.assertEqual(laguerre(1, 2), -x + 3)
        self.assertEqual(laguerre(1, 3), -x + 4)

        self.assertEqual(laguerre(2, 0), (x**2 - 4 * x + 2) / 2)
        self.assertEqual(laguerre(2, 1), (x**2 - 6 * x + 6) / 2)
        self.assertEqual(laguerre(2, 2), (x**2 - 8 * x + 12) / 2)
        self.assertEqual(laguerre(2, 3), (x**2 - 10 * x + 20) / 2)

        self.assertEqual(laguerre(3, 0), (-x**3 + 9 * x**2 - 18 * x + 6) / 6)
        self.assertEqual(laguerre(3, 1), (-x**3 + 12 * x**2 - 36 * x + 24) / 6)
        self.assertEqual(laguerre(3, 2), (-x**3 + 15 * x**2 - 60 * x + 60) / 6)
        self.assertEqual(laguerre(3, 3), (-x**3 + 18 * x**2 - 90 * x + 120) / 6)

        self.assertEqual(laguerre(4, 0),
                         (x**4 - 16 * x**3 + 72 * x**2 - 96 * x + 24) / 24)


class TestBisect(unittest.TestCase):

    def testBisect(self):
        x = Polynomial(1, 1)

        self.assertEqual(bisect(x, 0, 1), 0)
        self.assertEqual(bisect(x, -1, 0), 0)
        self.assertEqual(bisect(x, -1, 1), 0)
        self.assertEqual(bisect(-x, -1, 1), 0)
        self.assertEqual(bisect(x, -1, 3), 0)
        self.assertEqual(bisect(-x, -1, 3), 0)
        self.assertEqual(bisect(x, -3, 1), 0)
        self.assertEqual(bisect(-x, -3, 1), 0)
        self.assertLess(abs(bisect(x * x - x + 1e-100, 0.5, 2) - 1), 1e-15)
        self.assertRaises(Exception, bisect, x, 1, 2)
        self.assertRaises(Exception, bisect, x, 1, -1)


class TestRoots(unittest.TestCase):

    def testRoots(self):
        x = Polynomial(1, 1)

        self.assertRaises(Exception, roots, Polynomial(0))
        self.assertEqual(roots(Polynomial(1)), [])
        self.assertEqual(roots(x), [0])
        self.assertEqual(roots(x - 1), [1])
        self.assertEqual(roots(2 * x - 5), [2.5])
        self.assertEqual(roots(x * x - 1), [-1, 1])
        self.assertEqual(roots(-x * x + 1), [-1, 1])
        self.assertEqual(roots(x * x + 1), [])
        self.assertEqual(roots(-x * x - 1), [])
        self.assertEqual(roots(x * x), [0])
        self.assertEqual(roots(x ** 3 - x), [-1, 0, 1])
        self.assertEqual(roots(-x ** 3 + x), [-1, 0, 1])
        self.assertEqual(roots(x ** 3 - x ** 2), [0, 1])
        self.assertEqual(roots(x ** 3 + x ** 2), [-1, 0])
        self.assertEqual(roots(x ** 3 + x), [0])
        self.assertEqual(roots(-x ** 3 - x), [0])
        self.assertEqual(roots((x ** 2 + 1) * (x - 1)), [1])
        self.assertEqual(roots((x ** 2 + 1) * (x + 1)), [-1])
        self.assertEqual(roots(-(x ** 2 + 1) * (x - 1)), [1])
        self.assertEqual(roots(-(x ** 2 + 1) * (x + 1)), [-1])
        self.assertEqual(roots(x * x * (x + 1)), [-1, 0])
        self.assertEqual(roots(x * x * (x - 1)), [0, 1])
        self.assertEqual(roots(-x * x * (x + 1)), [-1, 0])
        self.assertEqual(roots(-x * x * (x - 1)), [0, 1])
        self.assertEqual(roots(x ** 3), [0])
        self.assertEqual(roots((x - 2) ** 3), [2])
        self.assertEqual(roots((x + 2) ** 3), [-2])


if __name__ == '__main__':
    unittest.main()
