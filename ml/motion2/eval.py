from models import MotionModel
from dataset import RobotDataset
import torch as th
import matplotlib.pyplot as plt
import pandas as pd
from constants import *

df = pd.read_csv("ml/motion2/datasets/00.csv")
test = RobotDataset(df, 'test')

model = MotionModel(
    test.total_feature_size,
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

with torch.inference_mode():
    for i in range(len(test)):
        past, present, future = test[i]
        past, present, future = past.to(device), present.to(device), future.cpu().item()
        pred = model(past, present)[0].cpu().item()

        pred = test.inverse_transformer(pred, 'velo')
        future = test.inverse_transformer(future, 'velo')

        last_is.append(i)
        last_preds.append(pred)
        last_futures.append(future)
        if len(last_is) > 100:
            last_is.pop(0)
            last_preds.pop(0)
            last_futures.pop(0)

        plt.scatter(last_is, last_preds, label='Predicted', c='r')
        plt.plot(last_is, last_futures, label='Actual', c='b')
        plt.draw()
        plt.pause(0.02)
        plt.clf()