import numpy as np
import pandas as pd

def falling_factorial_updated(i, n):
    if i < n: return 0
    result = 1
    for j in range(i, i - n, -1):
        result *= j
    return result

# Create a new 6x6 matrix using the updated falling factorial function
matrix_updated = np.zeros((6, 6), dtype=int)

for i in range(6):
    for j in range(6):
        matrix_updated[i, j] = falling_factorial_updated(i, j)

# Display the updated matrix
print("Updated Matrix:")
print(matrix_updated.T)