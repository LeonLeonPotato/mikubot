import models
import torch as th
import analyze
import numpy as np

import matplotlib.pyplot as plt
import constants


if __name__ == "__main__":
    df = analyze.load_processed_df(name = "driving_logs_0.csv").dropna()
    reverse_transformer = analyze.get_inverse_transformer("driving_logs_0.csv")
    X = th.tensor(df.values).float()
    dataset = analyze.RobotDataset(X, df)

    past_size = dataset[0][0].shape[-1]
    present_size = dataset[0][1].shape[-1]
    future_size = dataset[0][2].shape[-1]

    model = models.MotionModel(past_size, present_size, future_size).to(device=constants.device)
    model.load_state_dict(th.load(f"ml/motion/{constants.save_name}", map_location=constants.device))
    total_loss = 0

    plt.ion()
    plt.show()

    with th.no_grad():
        for past, present, future in dataset:
            past, present = past.to(device=constants.device), present.to(device=constants.device)
            output = model(past, present).squeeze(0)

            # print(y)
            px, py = output.cpu()[4], output.cpu()[5]
            px = reverse_transformer['left_velo'](px)
            py = reverse_transformer['right_velo'](py)

            ax, ay = future.cpu()[4], future.cpu()[5]
            ax = reverse_transformer['left_velo'](ax)
            ay = reverse_transformer['right_velo'](ay)

            plt.cla()
            plt.scatter(px, py, c='r', label='Predicted')
            plt.scatter(ax, ay, c='b', label='Actual')
            plt.xlim(-1000, 1000)
            plt.ylim(-1000, 1000)

            plt.title("Predicted vs Actual")
            plt.ylabel("dy")
            plt.xlabel("dx")

            plt.legend()
            plt.grid()
            plt.show()

            plt.pause(0.01)