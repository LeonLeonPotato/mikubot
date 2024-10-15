from torch.utils import data
import pandas as pd
from constants import *

class RobotDataset(data.Dataset):
    PAST_COLS = ['velo']
    PRESENT_COLS = ['volt']
    FUTURE_COLS = ['velo']

    MEANS = {
        'volt': 0.000016,
        'velo': 0.000001,
        'accel': -0.000078,
        'dt': 19997.082031
    }

    STDS = {
        'volt': 7721.352051,
        'velo': 303.388733,
        'accel': 16801.458984,
        'dt': 359.323303
    }

    TRANSFORMER = lambda x, col: (x - RobotDataset.MEANS[col]) / RobotDataset.STDS[col]
    INVERSE_TRANSFORMER = lambda x, col: x * RobotDataset.STDS[col] + RobotDataset.MEANS[col]

    def __init__(self, df : pd.DataFrame, split : str):
        self.indexer = {df.columns[i] : i for i in range(len(df.columns))}

        df = df.copy().apply(lambda col: RobotDataset.TRANSFORMER(col, col.name))
        train_size = int(len(df) * 0.8)
        test_size = len(df) - train_size
        self.X = torch.tensor(
            df.values[:train_size] if split == 'train'
            else df.values[-test_size:] if split == 'test'
            else df.values,
            dtype=torch.float32, device=device
        )

        self.total_feature_size = len(df.columns)
        self.past_indicies = [self.indexer[col] for col in RobotDataset.PAST_COLS]
        self.present_indicies = [self.indexer[col] for col in RobotDataset.PRESENT_COLS]
        self.future_indicies = [self.indexer[col] for col in RobotDataset.FUTURE_COLS]

    def __len__(self):
        return len(self.X) - lookback - 1

    def __getitem__(self, idx):
        return self.X[idx:idx + lookback, self.past_indicies], self.X[idx + lookback, self.present_indicies], self.X[idx + lookback, self.future_indicies]