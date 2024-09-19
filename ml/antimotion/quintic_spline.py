import numpy as np
import math
import scipy.sparse as sparse
from scipy.sparse.linalg import spsolve

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

class Polynomial:
    def __init__(self, coeffs=None):
        if coeffs is None:
            coeffs = np.zeros(6)
        self.coeffs = coeffs

    def compute(self, t):
        return np.polyval(self.coeffs[::-1], t)

    def derivative(self, t):
        der_coeffs = np.polyder(self.coeffs[::-1])
        return np.polyval(der_coeffs, t)

    def __call__(self, t):
        return self.compute(t)

    def debug_out(self):
        return str(self.coeffs)


class Polynomial2D:
    def __init__(self, x_poly=None, y_poly=None):
        self.x_poly = x_poly if x_poly is not None else Polynomial()
        self.y_poly = y_poly if y_poly is not None else Polynomial()

    def compute(self, t):
        return np.array([self.x_poly(t), self.y_poly(t)])

    def derivative(self, t):
        return np.array([self.x_poly.derivative(t), self.y_poly.derivative(t)])

    def normal(self, t):
        d = self.derivative(t)
        return np.array([-d[1], d[0]])

    def angle(self, t):
        d = self.derivative(t)
        return math.atan2(d[1], d[0])

    def length(self, resolution=50):
        length = 0
        for i in range(resolution):
            t0 = i / resolution
            t1 = (i + 1) / resolution
            length += np.linalg.norm(self.compute(t1) - self.compute(t0))
        return length

    def __call__(self, t):
        return self.compute(t)


class QuinticSpline:
    def __init__(self, points=None):
        self.total_length = 0
        self.segments = []
        self.points = points if points is not None else []

    def solve_spline(self, axis, ic_0, ic_1, bc_0, bc_1):
        num_vars = 6 * len(self.segments)
        data = []
        xi = []
        yi = []
        B = np.zeros((num_vars,))
        for i in range(0, len(self.segments)):
            r = i*6

            # at t=0, fi(0) = Y[i]
            data.append(1); xi.append(r); yi.append(r)
            B[r] = self.points[i][axis]

            # at t=1, fi(1) = Y[i+1]
            data.extend(differential_matrix_1[0])
            xi.extend([r+1]*6)
            yi.extend([r, r+1, r+2, r+3, r+4, r+5])
            B[r+1] = self.points[i+1][axis]

            if (i == len(self.segments)-1): continue

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
        B[-4] = ic_0

        # at t=0, f0''(0) = ic[1]
        data.extend(differential_matrix_0[2])
        xi.extend([num_vars-3]*6)
        yi.extend([0, 1, 2, 3, 4, 5])
        B[-3] = ic_1

        # at t=1, f(n-1)'(1) = fc[0]
        data.extend(differential_matrix_1[1])
        xi.extend([num_vars-2]*6)
        yi.extend([num_vars-6, num_vars-5, num_vars-4, num_vars-3, num_vars-2, num_vars-1])
        B[-2] = bc_0

        # at t=1, f(n-1)''(1) = fc[1]
        data.extend(differential_matrix_1[2])
        xi.extend([num_vars-1]*6)
        yi.extend([num_vars-6, num_vars-5, num_vars-4, num_vars-3, num_vars-2, num_vars-1])
        B[-1] = bc_1

        for x, y, z in zip(xi, yi, data):
            print(x, y, z)
        
        print("===========================")
        A = sparse.csc_matrix((data, (xi, yi)), shape=(num_vars, num_vars))
        X = spsolve(A, B)
        
        if axis == 0:
            for i in range(0, len(X), 6):
                self.segments[i//6].x_poly = Polynomial(X[i:i+6])
        else:
            for i in range(0, len(X), 6):
                self.segments[i//6].y_poly = Polynomial(X[i:i+6])

    def solve_coeffs(self, ic_theta_0, ic_theta_1, bc_theta_0, bc_theta_1):
        self.segments = [Polynomial2D() for _ in range(len(self.points) - 1)]
        self.solve_spline(0, 
                          np.cos(ic_theta_0), np.cos(ic_theta_1), 
                          np.cos(bc_theta_0), np.cos(bc_theta_1))
        self.solve_spline(1, 
                          np.sin(ic_theta_0), np.sin(ic_theta_1), 
                          np.sin(bc_theta_0), np.sin(bc_theta_1))

    def solve_length(self):
        self.total_length = 0
        for segment in self.segments:
            self.total_length += segment.length()

    def compute(self, t):
        if isinstance(t, np.ndarray):
            i = np.minimum(np.floor(t), len(self.segments)-1).astype(int)
            t = t - i
            ret = np.zeros((len(t), 2))
            for j in range(len(t)):
                ret[j, :] = self.segments[i[j]](t[j])
            return ret
        else:
            i = np.minimum(np.floor(t), len(self.segments)-1)
            t = t - i
            return self.segments[int(i)](t)

    def derivative(self, t):
        if isinstance(t, np.ndarray):
            i = np.minimum(np.floor(t), len(self.segments)-1).astype(int)
            t = t - i
            ret = np.zeros((len(t), 2))
            for j in range(len(t)):
                ret[j, :] = self.segments[i[j]].derivative(t[j])
            return ret
        else:
            i = np.minimum(np.floor(t), len(self.segments)-1)
            t = t - i
            return self.segments[int(i)].derivative(t)

    def debug_out(self):
        return str(self.points)

    def __call__(self, t):
        return self.compute(t)
    
    def __len__(self):
        return len(self.segments)

def __test():
    import matplotlib.pyplot as plt
    spline = QuinticSpline([(0, 0), (1, 0), (1, 1)])
    spline.solve_coeffs(0, 0, 0, 0)
    spline.solve_length()
    points = spline(np.linspace(0, len(spline), 100))
    plt.plot(points[:, 0], points[:, 1])
    plt.show()
    

if __name__ == "__main__":
    __test()