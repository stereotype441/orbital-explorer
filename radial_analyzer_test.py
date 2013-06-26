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
        self.assertEqual(n, Polynomial(1, 0))

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

if __name__ == '__main__':
    unittest.main()
