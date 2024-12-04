import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

p = pd.read_csv('./ml/motionprofile/logs/log31.txt')
p['time'] = p['time'] - p['time'][0]
p['velocity'] = np.sqrt(p['x'].diff() ** 2 + p['y'].diff() ** 2) / (p['time'] / 1e6).diff()

plt.plot(p['time'], p['velocity'])

plt.show()