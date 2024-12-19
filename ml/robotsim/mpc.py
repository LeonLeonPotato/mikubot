import cvxpy as cp
import numpy as np
import time

def mpc(n, x0, poses, dt=0.1):
    u = cp.Variable((n+1, 2))
    def get_A(k):
        return np.array([
            [1, 0, u[k, 0] * np.cos(x0[2]) * dt],
            [0, 1, -u[k, 0] * np.sin(x0[2]) * dt],
            [0, 0, 1]
        ])
    B0 = np.array([
        [np.sin(x0[2]) * dt, 0],
        [np.cos(x0[2]) * dt, 0],
        [0, dt]
    ])

    Ahat = [get_A(0)]
    for i in range(n):
        Ahat.append(get_A(i) @ Ahat[-1])
    
    Bhat = np.empty((n+1, n+1, 3, 2), dtype=object)
    for i in range(n+1):
        track = np.eye(3)
        for j in range(i, -1, -1):
            Bhat[i, j] = track @ B0
            track = get_A(j) @ track

    


mpc(3, [2, 2, 2], [])
