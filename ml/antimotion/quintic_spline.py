import matplotlib.pyplot as plt
import numpy as np
import time
import scipy.sparse as sparse
from scipy.sparse.linalg import spsolve
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
    data = []
    xi = []
    yi = []
    B = np.zeros((num_vars,))
    for i in range(0, len(Y) - 1):
        r = i*6

        # at t=0, fi(0) = Y[i]
        data.append(1); xi.append(r); yi.append(r)
        B[r] = Y[i]

        # at t=1, fi(1) = Y[i+1]
        data.extend(differential_matrix_1[0])
        xi.extend([r+1]*6)
        yi.extend([r, r+1, r+2, r+3, r+4, r+5])
        B[r+1] = Y[i+1]

        if (i == len(Y) - 2): continue

        for j in range(2, 6):
            data.extend(differential_matrix_1[j-1])
            xi.extend([r+j] * 6)
            yi.extend([r, r+1, r+2, r+3, r+4, r+5])
            data.extend(-differential_matrix_0[j-1])
            xi.extend([r+j] * 6)
            yi.extend([r+6, r+7, r+8, r+9, r+10, r+11])
            B[r+j] = 0

    # at t=0, f0'(0) = ic[0]
    data.extend(differential_matrix_0[1])
    xi.extend([num_vars-4]*6)
    yi.extend([0, 1, 2, 3, 4, 5])
    B[-4] = ic0

    # at t=0, f0''(0) = ic[1]
    data.extend(differential_matrix_0[2])
    xi.extend([num_vars-3]*6)
    yi.extend([0, 1, 2, 3, 4, 5])
    B[-3] = ic1

    # at t=1, f(n-1)'(1) = fc[0]
    data.extend(differential_matrix_1[1])
    xi.extend([num_vars-2]*6)
    yi.extend([num_vars-6, num_vars-5, num_vars-4, num_vars-3, num_vars-2, num_vars-1])
    B[-2] = bc0

    # at t=1, f(n-1)''(1) = fc[1]
    data.extend(differential_matrix_1[2])
    xi.extend([num_vars-1]*6)
    yi.extend([num_vars-6, num_vars-5, num_vars-4, num_vars-3, num_vars-2, num_vars-1])
    B[-1] = bc1

    A = sparse.csc_matrix((data, (xi, yi)), shape=(num_vars, num_vars))
    X = spsolve(A, B)
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

if __name__ == "__main__":
    times = []
    for length in range(10, 10000, 100):
        Y = np.random.rand(length)

        t = time.time()
        poly = compute_spline(Y, 1, 2, 9, -9)
        t = (time.time() - t) * 1000
        print(f"Time taken: {t:.4f}ms for {length} points")
        times.append(t)
    
    plt.plot(range(10, 10000, 100), times)
    plt.xlabel("Number of points")
    plt.ylabel("Time taken (ms)")
    plt.title("Time taken to compute quintic spline using sparse matrix method")
    plt.grid()
    plt.show()