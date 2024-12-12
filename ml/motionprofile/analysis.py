import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

p = pd.read_csv('./ml/motionprofile/logs/log1.txt')

p['time'] /= 1e6
p['time'] -= p['time'][0]

p['left_actual_velocity'] = p['left_actual_velocity'] * 2 * np.pi / 60.0
p['right_actual_velocity'] = p['right_actual_velocity'] * 2 * np.pi / 60.0

p = p[::2].reset_index(drop=True)
p['vx'] = p['x'].diff() / p['time'].diff()
p['vy'] = p['y'].diff() / p['time'].diff()
p['v'] = np.sqrt(p['vx']**2 + p['vy']**2)

angular = np.abs((p['left_actual_velocity'] + p['right_actual_velocity']) / 2)
linear = angular * 4.125 * 0.578
vw = p['v'] / linear
vw = vw[(0 < vw) & (vw < 2)]
plt.hist(vw, 200)
plt.show()

print(vw.median())

file = open("ml/motionprofile/out.txt", "w")

buffer = "P = ["
for i in range(len(p)):
    t = p['time'][i]
    lv = p['left_actual_velocity'][i] * 4.125 * 0.6
    buffer += f"({t}, {lv}), "
buffer = buffer[:-2] + "]"

file.write(buffer)
file.close()