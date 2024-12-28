import numpy as np

n = 6
m = np.zeros((n, n))

t = 0
m[0, :] = 1
m[:, 0] = 1
for i in range(1, n):
    for j in range(1, n - i):
        m[i, j] = m[i-1, j] + m[i, j-1]

print(m)