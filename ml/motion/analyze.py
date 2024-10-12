import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def normalize(df):
    df['left_voltage'] /= 12000
    df['right_voltage'] /= 12000
    df['dt'] /= 20000
    df['left_eff'] /= 100
    df['right_eff'] /= 100
    df['left_velo'] /= 580
    df['right_velo'] /= 580
    return df

def process(df : pd.DataFrame):
    df['dx'] = df['x'].diff()
    df['dy'] = df['y'].diff()
    df['dtheta'] = df['theta'].diff()
    df['cos'] = np.cos(df['theta'])
    df['sin'] = np.sin(df['theta'])
    return df.drop(['x', 'y', 'theta'], axis=1)

df = pd.read_csv("ml/motion/driving_logs_0.csv").dropna()
# plt.plot(df['x'], df['y'])
# df = normalize(df)
# df = process(df)

plt.ion()
plt.show()

for i in range(1, len(df)):
    plt.plot(df['x'].iloc[i-1:i+1], df['y'].iloc[i-1:i+1], 'r')
    plt.pause(0.01)
    plt.draw()
