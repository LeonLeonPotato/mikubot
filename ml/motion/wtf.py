import models
import torch as th
import analyze
import numpy as np

import matplotlib.pyplot as plt
import constants
import pandas as pd

if __name__ == "__main__":
    raw = pd.read_csv("ml/motion/driving_logs_1.csv")
    df = analyze.load_processed_df(name = "driving_logs_1.csv").dropna()

    get_transformer = analyze.get_transformer("driving_logs_1.csv")

    X = th.tensor(df.values).float()
    dataset = analyze.RobotDataset(X, df)

    plt.ion()
    plt.show()

    i = constants.lookback
    for past, present, future in dataset:
        ax, ay = future.cpu()[0], future.cpu()[1]

        plt.cla()
        plt.scatter(ax, ay, c='b', label='Actual')

        aax = raw['x'].diff()[i]
        aay = raw['y'].diff()[i]
        plt.scatter(aax, aay, label='asdasdasd')
        plt.xlim(-3, 3)
        plt.ylim(-3, 3)

        plt.title("Predicted vs Actual")
        plt.ylabel("dy")
        plt.xlabel("dx")

        plt.legend()
        plt.grid()
        plt.show()

        plt.pause(0.01)
        i+=1