import matplotlib.pyplot as plt
import numpy as np
import time
from typing import List

differential_matrix_1 = np.array([
    [1, 1, 1, 1, 1, 1],
    [0, 1, 2, 3, 4, 5],
    [0, 0, 2, 6, 12, 20],
    [0, 0, 0, 6, 24, 60],
    [0, 0, 0, 0, 24, 120],
    [0, 0, 0, 0, 0, 120]
])

differential_matrix_0 = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 2, 0, 0, 0],
    [0, 0, 0, 6, 0, 0],
    [0, 0, 0, 0, 24, 0],
    [0, 0, 0, 0, 0, 120]
])

def compute_spline(Y, ic0, ic1, bc0, bc1) -> List[np.polynomial.Polynomial]:
    num_vars = 6 * (len(Y) - 1)
    A = np.zeros((num_vars, num_vars))
    B = np.zeros((num_vars,))
    for i in range(0, len(Y) - 1):
        r = i*6

        # at t=0, fi(0) = Y[i]
        A[r, r] = 1 
        B[r] = Y[i]

        # at t=1, fi(1) = Y[i+1]
        A[r+1, r : r+6] = 1
        B[r+1] = Y[i+1]

        if (i == len(Y) - 2): continue

        for j in range(1, 5): 
            A[r+j+1, r : r+6] = differential_matrix_1[j]
            A[r+j+1, r+6 : r+12] = -differential_matrix_0[j]
            B[r+j+1] = 0

    # at t=0, f0'(0) = ic[0]
    A[-4, :6] = np.array([0, 1, 0, 0, 0, 0])
    B[-4] = ic0

    # at t=0, f0''(0) = ic[1]
    A[-3, :6] = np.array([0, 0, 2, 0, 0, 0])
    B[-3] = ic1

    # at t=1, f(n-1)'(1) = fc[0]
    A[-2, -6:] = np.array([0, 1, 2, 3, 4, 5])
    B[-2] = bc0

    # at t=1, f(n-1)''(1) = fc[1]
    A[-1, -6:] = np.array([0, 0, 2, 6, 12, 20])
    B[-1] = bc1

    X = np.linalg.solve(A, B)
    ret = []
    for i in range(0, X.shape[0]//6):
        ret.append(np.polynomial.Polynomial(X[i*6:i*6+6]))
    return ret

def compute(poly, t):
    idx = int(t)
    if idx == len(poly): idx -= 1
    t = t % 1.0 + int(t == len(poly))
    return poly[idx](t)

def calc_length(Xpoints, Ypoints):
    length = [0]
    for i in range(len(Xpoints) - 1):
        dx = Xpoints[i + 1] - Xpoints[i]
        dy = Ypoints[i + 1] - Ypoints[i]
        length.append(length[-1] + np.sqrt(dx ** 2 + dy ** 2))
    return length
