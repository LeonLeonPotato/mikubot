import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

p = pd.read_csv('./ml/motionprofile/logs/log1.txt')

p['time'] /= 1e6
p['time'] -= p['time'][0]

# s = 0.05224810442586921
# m = 2 * np.pi / 60.0
# p['left_actual_velocity'] = p['left_actual_velocity'] * p['time'].diff() * m * s
# p['right_actual_velocity'] = p['right_actual_velocity'] * p['time'].diff() * m * s
# p['dtheta'] = p['theta'].diff()

# p = p.dropna()
# p = p.reset_index(drop=True)
# p['calculated_dtheta'] = p['left_actual_velocity'] - p['right_actual_velocity']
# print("Correlation:", np.polyfit(p['calculated_dtheta'], p['dtheta'], 1)[0])

# plt.scatter(
#     p['calculated_dtheta'],
#     p['dtheta']
# )
# plt.show()

p['div'] = p['right_actual_velocity'] / p['right_actual_voltage']
p = p[(p['div'] < 0.1) & (p['div'] > -0.1)]
plt.scatter(p['time'], p['div'])
plt.show()