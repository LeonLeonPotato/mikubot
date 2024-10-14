import torch.utils.data as data
from constants import *
from dataset import RobotDataset
from models import MotionModel

import torch as th
import torch.nn as nn
import os
import pandas as pd
import matplotlib.pyplot as plt

def load_dataset():
    train_datasets = []
    test_datasets = []
    for file in os.listdir("ml/motion2/datasets"):
        if file.endswith(".csv"):
            df = pd.read_csv(f"ml/motion2/datasets/{file}").dropna()
            train_datasets.append(RobotDataset(df, 'train'))
            test_datasets.append(RobotDataset(df, 'test'))
            
    return data.ConcatDataset(train_datasets), data.ConcatDataset(test_datasets)

train_dataset, eval_dataset = load_dataset()
train_loader = data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
eval_loader = data.DataLoader(eval_dataset, batch_size=batch_size, shuffle=True)

model = MotionModel(
    6,
    len(RobotDataset.PRESENT_COLS),
    len(RobotDataset.FUTURE_COLS)
).to(device)

criterion = nn.MSELoss()
optimizer = th.optim.AdamW(model.parameters(), lr=learning_rate)

def do_eval():
    model.eval()
    with th.no_grad():
        total_loss = 0
        for past, present, future in eval_loader:
            past, present, future = past.to(device), present.to(device), future.to(device)
            output = model(past, present)
            loss = criterion(output, future)
            total_loss += loss.item()
    model.train()
    return total_loss / len(eval_loader)

def do_train():
    total_steps = 0
    train_plot_x = []
    train_plot_y = []
    eval_plot_x = []
    eval_plot_y = []

    try:
        for epoch in range(1000):
            for past, present, future in train_loader:
                past, present, future = past.to(device), present.to(device), future.to(device)

                optimizer.zero_grad()
                output = model(past, present)
                loss = criterion(output, future)

                loss.backward()
                optimizer.step()
                
                train_plot_x.append(total_steps)
                train_plot_y.append(loss.item())

                if total_steps % 50 == 0:
                    eval_loss = do_eval()

                    eval_plot_x.append(total_steps)
                    eval_plot_y.append(eval_loss)

                    print(f"step {total_steps} eval loss: {eval_loss} train loss: {loss.item()}")

                total_steps += 1
    except BaseException as e:
        if isinstance(e, KeyboardInterrupt):
            print("Exiting early")
        else:
            raise e

    return train_plot_x, train_plot_y, eval_plot_x, eval_plot_y

if __name__ == "__main__":
    train_plot_x, train_plot_y, eval_plot_x, eval_plot_y = do_train()
    th.save(model.state_dict(), f"ml/motion2/{save_name}")

    plt.plot(train_plot_x, train_plot_y, label='Train Loss')
    plt.plot(eval_plot_x, eval_plot_y, label='Eval Loss')
    plt.legend()
    plt.show()