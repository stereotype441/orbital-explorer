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

        # These come last, to check for inadvertent modifications
        self.assertEqual(z, Polynomial(0, 0))
        self.assertEqual(n, Polynomial(1, 0))
        self.assertEqual(x, Polynomial(1, 1))
        self.assertEqual(x2, Polynomial(1, 2))
        self.assertEqual(x3, Polynomial(1, 3))

if __name__ == '__main__':
    unittest.main()
