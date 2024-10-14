import torch.utils.data as data
from constants import *
from dataset import RobotDataset
from models import MotionModel

import torch as th
import torch.nn as nn
import os
import pandas as pd
import matplotlib.pyplot as plt
from typing import Tuple, List

def load_dataset() -> Tuple[data.ConcatDataset, data.ConcatDataset]:
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

th.set_printoptions(sci_mode=False)

def do_eval(model, criterion) -> float:
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

def do_train(model, criterion, optimizer, evaluate=True, output_console=True, max_steps=2000):
    total_steps = 0
    train_plot_x = []
    train_plot_y = []
    eval_plot_x = [1]
    eval_plot_y = [do_eval(model, criterion)]

    try:
        while total_steps < max_steps:
            for past, present, future in train_loader:
                total_steps += 1
                past, present, future = past.to(device), present.to(device), future.to(device)

                optimizer.zero_grad()
                output = model(past, present)
                loss = criterion(output, future)

                loss.backward()
                optimizer.step()
                
                train_plot_x.append(total_steps)
                train_plot_y.append(loss.item())

                if total_steps % 50 == 0:
                    if evaluate:
                        eval_loss = do_eval(model, criterion)

                        eval_plot_x.append(total_steps)
                        eval_plot_y.append(eval_loss)

                        if output_console:
                            print(f"step {total_steps} eval loss: {eval_loss:.4f} train loss: {loss.item():.4f}")
                    else:
                        if output_console:
                            print(f"step {total_steps} train loss: {loss.item():.4f}")

                if total_steps >= max_steps:
                    break
    except BaseException as e:
        if isinstance(e, KeyboardInterrupt):
            print("Exiting early")
        else:
            raise e

    return train_plot_x, train_plot_y, eval_plot_x, eval_plot_y

if __name__ == "__main__":
    import constants
    best_model = None
    best_eval_score = 999
    criterion = nn.L1Loss()

    for hidden_size in [1, 2, 4, 8, 16, 32, 64, 128, 256]:
        torch.manual_seed(42)

        model = MotionModel(
            4,
            len(RobotDataset.PRESENT_COLS),
            len(RobotDataset.FUTURE_COLS),
            psize=hidden_size
        ).to(device)

        optimizer = th.optim.AdamW(model.parameters(), lr=learning_rate)

        train_plot_x, train_plot_y, eval_plot_x, eval_plot_y = do_train(model, criterion, optimizer, evaluate=True, output_console=False, max_steps=2000)

        print(f"Hidden Size: {hidden_size} | Best eval score: {eval_plot_y[-1]}")

        if eval_plot_y[-1] < best_eval_score:
            best_eval_score = eval_plot_y[-1]
            best_model = model.state_dict()
            print("New best model")

    th.save(best_model, f"ml/motion2/best_{save_name}")
    print(f"Best eval score: {best_eval_score}")