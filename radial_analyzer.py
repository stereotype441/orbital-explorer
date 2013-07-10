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


import numbers
from math import exp


class Polynomial:
    """Polynomials, immutable, with floating point coefficients"""

    # Paul's suggestion: make a list constructor.

    def __init__(self, c = 0, n = 0):
        '''Polynomial(c, n) creates the polynomial c*x^n.
        Polynomial([c0, c1, ..., cn]) creates the polynomial c0 + c1 x + ...'''
        # self.__coeffs[n] is the coefficient of x^n.  Invariant:
        # if len(self.__coeffs) > 0 then self.__coeffs[-1] != 0
        if isinstance(c, list):
            self.__coeffs = list(c)
        else:
            self.__coeffs = [0] * n + [c]
        self.__standardize()

    def __standardize(self):
        while self.degree >= 0 and self.__coeffs[-1] == 0:
            self.__coeffs.pop()

    @property
    def degree(self):
        return len(self.__coeffs) - 1

    @property
    def constantTerm(self):
        if self.degree == -1:
            return 0
        else:
            return self.__coeffs[0]

    @property
    def leadingCoefficient(self):
        if self.degree == -1:
            return 0
        else:
            return self.__coeffs[-1]

    def __eq__(self, other):
        return self.__coeffs == other.__coeffs

    def __ne__(self, other):
        return not (self == other)

    def __call__(self, x):
        total = 0
        for c in reversed(self.__coeffs):
            total *= x
            total += c
        return total

    def __add__(self, other):
        if isinstance(other, numbers.Number):
            return self + Polynomial(other)
        if self.degree < other.degree:
            sm = self.__coeffs
            lg = other.__coeffs
        else:
            sm = other.__coeffs
            lg = self.__coeffs
        s = list(lg)
        for i in range(len(sm)):
            s[i] += sm[i]
        return Polynomial(s)

    def __radd__(self, other):
        return self + other

    def __pos__(self):
        return self

    def __neg__(self):
        return Polynomial([-x for x in self.__coeffs])

    def __sub__(self, other):
        return self + (-other)

    def __rsub__(self, other):
        return (-self) + other

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return self * Polynomial(other)
        p = [0] * (self.degree + other.degree + 1)
        for i in range(len(self.__coeffs)):
            for j in range(len(other.__coeffs)):
                p[i + j] += self.__coeffs[i] * other.__coeffs[j]
        return Polynomial(p)

    def __rmul__(self, other):
        return self * other

    def __truediv__(self, other):
        return self * (1 / other)

    def __pow__(self, e):
        if e < 0:
            raise ArithmeticError('Polynomial to a negative power')
        if e == 0:
            return Polynomial(1)
        if e == 1:
            return self
        if e % 2 == 0:
            return (self * self) ** (e >> 1)
        return self * (self ** (e - 1))

    def derivative(self):
        return Polynomial([i * self.__coeffs[i]
                           for i in range(1, self.degree + 1)])


def factorial(n):
    if n < 0:
        raise ArithmeticError('Factorial of a negative number')
    f = 1
    for i in range(2, n + 1):
        f *= i
    return f


def choose(n, k):
    return factorial(n) // (factorial(k) * factorial(n - k))


def laguerre(n, a):
    x = Polynomial(1, 1)
    f = 0
    for i in range(n + 1):
        f += ((-1) ** i) * choose(n + a, n - i) * (x ** i) / factorial(i)
    return f


def bisect(f, lower, upper):
    if not (lower < upper):
        raise Exception('bisect: lower not less than upper')
    f_lower = f(lower)
    if f_lower == 0:
        return lower
    f_upper = f(upper)
    if f_upper == 0:
        return upper
    if (f_lower < 0 and f_upper < 0) or (f_lower > 0 and f_upper > 0):
        raise Exception('bisect: no sign change present')

    while True:
        mid = (lower + upper) / 2
        if not (lower < mid < upper):
            return mid
        f_mid = f(mid)
        if f_mid == 0:
            return mid
        if f_mid < 0:
            if f_lower < 0:
                lower = mid
                f_lower = f_mid
            else:
                upper = mid
                f_upper = f_mid
        else:
            if f_lower > 0:
                lower = mid
                f_lower = f_mid
            else:
                upper = mid
                f_upper = f_mid


def roots(f):
    if f.degree < 1:
        if f.constantTerm != 0:
            return []
        raise Exception('roots called on the zero polynomial')
    if f.degree == 1:
        return [-f.constantTerm / f.leadingCoefficient]
    df = f.derivative()
    df_roots = roots(df)

    leading_coeff_f = f.leadingCoefficient
    degree_f = f.degree

    # First, handle the case where df has no roots
    if len(df_roots) == 0:
        assert degree_f % 2 == 1
        f0 = f(0)
        if f0 == 0:
            return [0]
        if leading_coeff_f > 0 and f0 < 0:
            upper = 1
            while f(upper) <= 0:
                upper += 1
            return [bisect(f, 0, upper)]
        if leading_coeff_f > 0 and f0 > 0:
            lower = -1
            while f(lower) >= 0:
                lower -= 1
            return [bisect(f, lower, 0)]
        if leading_coeff_f < 0 and f0 > 0:
            upper = 1
            while f(upper) >= 0:
                upper += 1
            return [bisect(f, 0, upper)]
        if leading_coeff_f < 0 and f0 < 0:
            lower = -1
            while f(lower) <= 0:
                lower -= 1
            return [bisect(f, lower, 0)]
        raise Exception('Impossible monotonic polynomial')

    r = []

    # Check for a root to the left of the first root of df
    first_df_root = df_roots[0]
    f_at_first_df_root = f(first_df_root)
    negative_behavior_f = leading_coeff_f * ((-1) ** degree_f)
    if negative_behavior_f > 0 and f_at_first_df_root < 0:
        lower_bound_on_first_root = first_df_root - 1
        while f(lower_bound_on_first_root) <= 0:
            lower_bound_on_first_root -= 1
        r.append(bisect(f, lower_bound_on_first_root, first_df_root))
    if negative_behavior_f < 0 and f_at_first_df_root > 0:
        lower_bound_on_first_root = first_df_root - 1
        while f(lower_bound_on_first_root) >= 0:
            lower_bound_on_first_root -= 1
        r.append(bisect(f, lower_bound_on_first_root, first_df_root))

    # Look at each pair of roots of df
    for i in range(len(df_roots) - 1):
        dr1 = df_roots[i]
        dr2 = df_roots[i + 1]
        fdr1 = f(dr1)
        fdr2 = f(dr2)
        if fdr1 > 0 and fdr2 < 0 or fdr1 < 0 and fdr2 > 0:
            r.append(bisect(f, dr1, dr2))
        if fdr1 == 0:
            r.append(dr1)

    # Last one -- just check if it's a root of f
    if f(df_roots[-1]) == 0:
        r.append(df_roots[-1])

    # Check for a root to the right of the last root of df
    last_df_root = df_roots[-1]
    f_at_last_df_root = f(last_df_root)
    positive_behavior_f = leading_coeff_f
    if positive_behavior_f > 0 and f_at_last_df_root < 0:
        upper_bound_on_last_root = last_df_root + 1
        while f(upper_bound_on_last_root) <= 0:
            upper_bound_on_last_root += 1
        r.append(bisect(f, last_df_root, upper_bound_on_last_root))
    if positive_behavior_f < 0 and f_at_last_df_root > 0:
        upper_bound_on_last_root = last_df_root + 1
        while f(upper_bound_on_last_root) >= 0:
            upper_bound_on_last_root += 1
        r.append(bisect(f, last_df_root, upper_bound_on_last_root))

    return r


def list_to_cpp(nums):
    if nums == []:
        return '    {}'
    return '    {\n      ' + ',\n      '.join([str(n) for n in nums]) + \
        '\n    }'


max_n = 16


def make_table3(name, func):
    '''Make a C++ table of arrays for each n and L'''
    sn = str(max_n)
    print('const double ' + name + '[' + sn + '][' + sn + '][' + sn + '] = {')
    for n in range(1, max_n + 1):
        print('  // n ==', n)
        print('  {')
        for L in range(0, n):
            print('    // L ==', L)
            s = list_to_cpp(func(n, L))
            if L != n - 1:
                s += (',')
            print(s)
        if n != max_n:
            print('  },')
        else:
            print('  }')
    print('};')


def make_table2(name, func):
    '''Make a C++ table of values for each n and L'''
    sn = str(max_n)
    print('const double ' + name + '[' + sn + '][' + sn + '] = {')
    for n in range(1, max_n + 1):
        print('  // n ==', n)
        print('  {')
        for L in range(0, n):
            print('    // L ==', L)
            s = '    ' + str(func(n, L))
            if L != n - 1:
                s += (',')
            print(s)
        if n != max_n:
            print('  },')
        else:
            print('  }')
    print('};')


def print_license():
    print('/*')
    print(' * This file is part of the Electron Orbital Explorer. The Electron')
    print(' * Orbital Explorer is distributed under the Simplified BSD License')
    print(' * (also called the "BSD 2-Clause License"), in hopes that these')
    print(' * rendering techniques might be used by other programmers in')
    print(' * applications such as scientific visualization, video gaming, and so')
    print(' * on. If you find value in this software and use its technologies for')
    print(' * another purpose, I would love to hear back from you at bjthinks (at)')
    print(' * gmail (dot) com. If you improve this software and agree to release')
    print(' * your modifications under the below license, I encourage you to fork')
    print(' * the development tree on github and push your modifications. The')
    print(' * Electron Orbital Explorer\'s development URL is:')
    print(' * https://github.com/bjthinks/orbital-explorer')
    print(' * (This paragraph is not part of the software license and may be')
    print(' * removed.)')
    print(' *')
    print(' * Copyright (c) 2013, Brian W. Johnson')
    print(' * All rights reserved.')
    print(' *')
    print(' * Redistribution and use in source and binary forms, with or without')
    print(' * modification, are permitted provided that the following conditions')
    print(' * are met:')
    print(' *')
    print(' * + Redistributions of source code must retain the above copyright')
    print(' *   notice, this list of conditions and the following disclaimer.')
    print(' *')
    print(' * + Redistributions in binary form must reproduce the above copyright')
    print(' *   notice, this list of conditions and the following disclaimer in')
    print(' *   the documentation and/or other materials provided with the')
    print(' *   distribution.')
    print(' *')
    print(' * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS')
    print(' * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT')
    print(' * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS')
    print(' * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE')
    print(' * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,')
    print(' * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,')
    print(' * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;')
    print(' * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER')
    print(' * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT')
    print(' * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN')
    print(' * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE')
    print(' * POSSIBILITY OF SUCH DAMAGE.')
    print(' */')


'''
The radial factor of the wave function is of the form:
(x ^ L) * exp(-x / 2) * Laguerre(x)

To find radial nodes, we set this to zero, and look for nonzero
solutions. These occur iff the Laguerre polynomial factor is zero.
'''


def radial_nodes(n, L):
    return roots(laguerre(n - L - 1, 2 * L + 1))


'''
To find radial maxima, we set the derivative of the radial factor to
zero, like so:
(L * Laguerre(x) + x * (-1 / 2) * Laguerrre(x) + x * Laguerre'(x))
    * (x ^ (L-1)) * exp(-x / 2) = 0
Note that this is correct only for positive L, and we must handle the
case L=0 separately.
Simplifying, and ignoring the solution x=0, we get:
(L - x / 2) * Laguerre(x) + x * Laguerre'(x) = 0
For the special case L=0, we instead have:
(-1 / 2) * Laguerre(x) + Laguerre'(x) = 0,
which differs only in not having zero as a root. (Note that an extra
root at x=0 would confuse the C++ use of the table, where zero is
treated as an 'end of data' marker.)
'''


def radial_maxima(n, L):
    x = Polynomial(1,1)
    la = laguerre(n - L - 1, 2 * L + 1)
    dla = la.derivative()
    if L != 0:
        f = (L - x / 2) * la + x * dla
    else:
        f = (-1 / 2) * la + dla
    return roots(f)


def radial_extent(n, L):
    maxes = radial_maxima(n, L)
    maxes.append(0)
    def f(r):
        return abs((r ** L) * exp(-r / 2) * laguerre(n - L - 1, 2 * L + 1)(r))
    big_f = max([f(r) for r in maxes])
    upper_x = max(maxes) + 1
    while f(upper_x) > big_f / 1e5:
        upper_x += 1
    return upper_x


def radial_extent2(n, L):
    maxes = radial_maxima(n, L)
    maxes.append(0)
    def f(r):
        return ((r ** L) * exp(-r / 2) * laguerre(n - L - 1, 2 * L + 1)(r)) ** 2
    big_f = max([f(r) for r in maxes])
    upper_x = max(maxes) + 1
    while f(upper_x) > big_f / 1e5:
        upper_x += 1
    return upper_x


if __name__ == '__main__':
    print_license()
    print('')
    print('#include "radial_data.hh"')
    print('')
    make_table3('radial_nodes', radial_nodes)
    print('')
    make_table3('radial_maxima', radial_maxima)
    print('')
    make_table2('radial_extent', radial_extent)
    print('')
    make_table2('radial_extent2', radial_extent2)
