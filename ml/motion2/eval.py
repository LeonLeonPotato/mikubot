from models import MotionModel
from dataset import RobotDataset
import torch as th
import matplotlib.pyplot as plt
import pandas as pd
from constants import *

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

df = pd.read_csv("ml/motion2/datasets/test/02.csv")
test = RobotDataset(df, 'all')

model = MotionModel(
    len(RobotDataset.PAST_COLS),
    len(RobotDataset.PRESENT_COLS),
    len(RobotDataset.FUTURE_COLS)
).to(device)

model.load_state_dict(th.load("ml/motion2/model.pth", map_location=device))
model.eval()

plt.ion()
plt.show()
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Predicted vs Actual Velocity')

last_is = []
last_preds = []
last_futures = []
losses = []

with torch.inference_mode():
    for i in range(len(test)):
        past, present, future = test[i]
        past, present, future = past.to(device), present.to(device), future.cpu().item()
        pred = model(past, present)[0].cpu().item()

        pred = RobotDataset.INVERSE_TRANSFORMER(pred, 'velo')
        future = RobotDataset.INVERSE_TRANSFORMER(future, 'velo')

        print("Loss: {:.4f}".format(abs(pred - future)))
        losses.append(abs(pred - future))

        last_is.append(i)
        last_preds.append(pred)
        last_futures.append(future)
        if len(last_is) > 100:
            last_is.pop(0)
            last_preds.pop(0)
            last_futures.pop(0)

        # plt.scatter(last_is, last_preds, label='Predicted', c='r')
        # plt.plot(last_is, last_futures, label='Actual', c='b')
        # plt.draw()
        # plt.pause(0.02)
        # plt.clf()

import numpy as np

print("Average loss: {:.4f}".format(sum(losses) / len(losses)))
print("Median:", np.median(losses))
print("25%", np.percentile(losses, 25))
print("75%", np.percentile(losses, 75))
print("Max:", max(losses))
print("Min:", min(losses))

plt.ioff(); plt.cla(); plt.clf()
plt.hist(losses, bins=100)
plt.xlabel('Time')
plt.ylabel('Loss')
plt.title('Loss over time')
plt.show()