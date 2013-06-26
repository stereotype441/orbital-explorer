class Polynomial(object):
    """Polynomials, immutable, with floating point coefficients"""

    def __init__(self, c = 0, n = 0):
        self.__coeffs = [0 for i in range(n)] + [c]
        self.__standardize()

    def __standardize(self):
        while self.degree >= 0 and self.__coeffs[self.degree] == 0:
            self.__coeffs.pop()

    @property
    def degree(self):
        return len(self.__coeffs) - 1

    def __eq__(self, other):
        if self.degree != other.degree:
            return False
        for i in range(self.degree + 1):
            if self.__coeffs[i] != other.__coeffs[i]:
                return False
        return True
    def __ne__(self, other):
        return not (self == other)

    def __call__(self, x):
        total = 0
        for i in range(self.degree, -1, -1):
            total *= x
            total += self.__coeffs[i]
        return total
