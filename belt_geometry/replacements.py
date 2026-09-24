"""
This file contains the pure python replacements for numpy and scipy functions.
"""

import math


class Vec2:
    def __init__(self, x, y=None):
        if y is None:
            if isinstance(x, Vec2):
                self.x, self.y = x.x, x.y
            else:
                self.x, self.y = x
        else:
            self.x = x
            self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __truediv__(self, scalar):
        return Vec2(self.x / scalar, self.y / scalar)

    def norm(self):
        return math.hypot(self.x, self.y)

    def normalized(self):
        n = self.norm()
        if n == 0:
            return Vec2(0, 0)
        return self / n

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def to_list(self):
        return [self.x, self.y]
    

def compute_point_distance(p1, p2):
    """
    Pure python version of computing the norm/distance between two points in 2D space.
    """
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def secant_method(error, x0, x1, tol=1e-9, max_iter=100):
    """
    Secant method from:
    https://en.wikipedia.org/wiki/Secant_method
    """

    for _ in range(max_iter):
        f_x0 = error(x0)
        f_x1 = error(x1)
        x2 = x1 - f_x1 * (x1 - x0) / (f_x1 - f_x0)

        if abs(x2 - x1) < tol:
            return x2

        x0, x1 = x1, x2

    raise ValueError("Secant method did not converge!")


def bisection_method(error, start, end, tol=1e-6, max_iter=100):
    """
    Bisection method from:
    https://www.geeksforgeeks.org/dsa/program-for-bisection-method/
    """
    if (error(start) * error(end) >= 0):
        raise ValueError("The solution is not within the (start, end) span!")
    
    x = start
    for _ in range(max_iter):

        # Find middle point
        x = (start + end) / 2

        # Check if middle point is root
        if abs(error(x)) < tol:
            return x

        # Decide the side to repeat the steps
        if error(x) * error(start) < 0:
            end = x
        else:
            start = x
    
    raise ValueError("Bisection method did not converge!")
    