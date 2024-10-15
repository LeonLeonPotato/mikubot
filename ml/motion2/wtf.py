import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('ml/motion2/driving_logs_9.csv')
plt.plot(df['left_velo'])
plt.show()