from robot_sim import *
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def test_robot(raw_df:pd.DataFrame, args, interactive = False):
    robot = Robot(0, 0, 0, args)

    if interactive:
        plt.ion()
        plt.show()
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.xlim(-4, 4)
        plt.ylim(-4, 4)
        plt.title('Predicted vs Actual Velocity')

    error = 0
    n = 0
    for idx, row in raw_df.iterrows():
        lx, ly, ltheta = robot.x, robot.y, robot.theta
        robot.update(row['left_voltage'], row['right_voltage'], row['dt'] / 1e6)
        pdx, pdy = robot.x - lx, robot.y - ly
        dtheta = robot.theta - ltheta
        robot.theta = row['theta']

        if interactive:
            plt.scatter(row['dx'], row['dy'], c='r')
            plt.scatter(pdx, pdy, c='b')

            plt.grid()
            plt.xlim(-4, 4)
            plt.ylim(-4, 4)

            plt.draw()
            plt.pause(0.02)
            plt.clf()

        error += abs(row['dtheta'] - dtheta)
        n += 1

    if interactive:
        plt.ioff()
    
    print(n)
    return error / n

if __name__ == "__main__":
    raw = pd.read_csv("ml/motion2/datasets/pure/driving_logs_9.csv")
    raw['dx'] = raw['x'].diff().shift(-1)
    raw['dy'] = raw['y'].diff().shift(-1)
    raw['dtheta'] = raw['theta'].diff().shift(-1)
    raw = raw.dropna()

    args = RobotArgs(5.08, 98.4375)
    error = test_robot(raw, args, True)
    print(error)

    # while l < h:
    #     m = (l + h) / 2
    #     args = RobotArgs(5.08, m)
    #     error = test_robot(raw, args, False)
    #     args = RobotArgs(5.08, m+0.01)
    #     error2 = test_robot(raw, args, False)
    #     print(m, f"[{l}, {h}]", error)
    #     if error < error2:
    #         h = m
    #     else:
    #         l = m