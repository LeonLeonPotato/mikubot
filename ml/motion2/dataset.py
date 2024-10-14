from torch.utils import data
import pandas as pd
from constants import *

class RobotDataset(data.Dataset):
    PRESENT_COLS = ['volt', 'dt']
    FUTURE_COLS = ['velo']

    MEANS = {
        'volt': 0.000039,
        'velo': 0.000000,
        'accel': -0.000078,
        'dt': 19997.082031
    }

    STDS = {
        'volt': 8449.343750,
        'velo': 336.078247,
        'accel': 16801.458984,
        'dt': 359.323303
    }

    def __init__(self, df : pd.DataFrame, split : str):
        self.indexer = {df.columns[i] : i for i in range(len(df.columns))}
        self.transformer = lambda x, col: (x - RobotDataset.MEANS[col]) / RobotDataset.STDS[col]
        self.inverse_transformer = lambda x, col: x * RobotDataset.STDS[col] + RobotDataset.MEANS[col]

        df = df.copy().apply(lambda col: (col - RobotDataset.MEANS[col.name]) / RobotDataset.STDS[col.name])
        train_size = int(len(df) * 0.8)
        test_size = len(df) - train_size
        self.X = torch.tensor(
            df.values[:train_size] if split == 'train' else df.values[-test_size:],
            dtype=torch.float32, device=device
        )

        self.total_feature_size = len(df.columns)
        self.present_indicies = [self.indexer[col] for col in RobotDataset.PRESENT_COLS]
        self.future_indicies = [self.indexer[col] for col in RobotDataset.FUTURE_COLS]

    def __len__(self):
        return len(self.X) - lookback - 1

    def __getitem__(self, idx):
        return self.X[idx:idx + lookback], self.X[idx + lookback, self.present_indicies], self.X[idx + lookback, self.future_indicies]