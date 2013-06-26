from radial_analyzer import *
import unittest

class TestPolynomial(unittest.TestCase):

    def testPolynomial(self):
        self.assertEqual(Polynomial(), Polynomial())

if __name__ == '__main__':
    unittest.main()
