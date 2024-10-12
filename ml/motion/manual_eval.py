import models
import torch as th
import analyze
import numpy as np

import matplotlib.pyplot as plt
import constants


if __name__ == "__main__":
    df = analyze.load_processed_df(name = "driving_logs_1.csv").dropna()
    reverse_transformer = analyze.get_inverse_transformer("driving_logs_1.csv")
    X = th.tensor(df.values).float()
    dataset = analyze.RobotDataset(X, df)

    past_size = dataset[0][0].shape[-1]
    present_size = dataset[0][1].shape[-1]
    future_size = dataset[0][2].shape[-1]

    model = models.MotionModel(past_size, present_size, future_size).cuda()
    model.load_state_dict(th.load(f"ml/motion/{constants.save_name}"))
    total_loss = 0

    plt.ion()
    plt.show()

    with th.no_grad():
        for past, present, future in dataset:
            past, present = past.cuda(), present.cuda()
            output = model(past.unsqueeze(0), present.unsqueeze(0)).squeeze(0)

            # print(y)
            px, py = output.cpu()[0], output.cpu()[1]
            # px = reverse_transformer['dx'](px)
            # py = reverse_transformer['dy'](py)

            ax, ay = future.cpu()[0], future.cpu()[1]

            plt.cla()
            plt.scatter(px, py, c='r', label='Predicted')
            plt.scatter(ax, ay, c='b', label='Actual')
            plt.xlim(-3, 3)
            plt.ylim(-3, 3)

            plt.title("Predicted vs Actual")
            plt.ylabel("dy")
            plt.xlabel("dx")

            plt.legend()
            plt.grid()
            plt.show()

            plt.pause(0.01)