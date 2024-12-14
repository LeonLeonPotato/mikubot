import pandas as pd
import matplotlib.pyplot as plt

with open("ml/robotsim/output.txt") as f:
    data = eval(f.read())

data = pd.DataFrame(data, columns=['s', 'v'])
data['u2'] = data['v'].shift(1) ** 2
data['v2'] = data['v'] ** 2
data['ds'] = data['s'].diff()

data = data.dropna().reset_index(drop=True)

data['a'] = (data['v2'] - data['u2']) / (2 * data['ds'])

plt.plot(data['s'], data['v'], label='Velocity')
plt.plot(data['s'], data['a'], c='g', label='Accel')
plt.plot([0, data['s'][len(data)-1]], [300, 300], c='r')
plt.plot([0, data['s'][len(data)-1]], [-237, -237], c='r')
plt.show()