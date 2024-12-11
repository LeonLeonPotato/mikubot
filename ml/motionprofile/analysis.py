import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

p = pd.read_csv('./ml/motionprofile/logs/log1.txt')
p['time'] /= 1e6
p['time'] -= p['time'][0]
p['vx'] = (p['x'].diff() / p['time'].diff())[::2]
p['vy'] = (p['y'].diff() / p['time'].diff())[::2]

plt.plot(p['time'], p['vx'], label='vx')
plt.plot(p['time'], p['vy'], label='vy')
plt.legend()
plt.show()

# buffer = "P = ["
# for i in range(len(p)):
#     buffer += f"({p['time'][i]}, {p['left_actual_velocity'][i]}), "
# buffer = buffer[:-2] + "]"
# print(buffer)