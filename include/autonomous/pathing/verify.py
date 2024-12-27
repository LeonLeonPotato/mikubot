import numpy as np
from scipy.interpolate import CubicSpline

def verify_spline_equation(x, y):
    spline = CubicSpline(x, y, bc_type='natural')

    for n in range(0, len(x) - 2):
        a, b, c, d = spline.c[n]
        

verify_spline_equation(
    [0, 1, 2, 3, 4, 5],
    [0, 1, 2, 3, 4, 7]
)