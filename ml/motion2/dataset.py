from torch.utils import data
import pandas as pd
from constants import *

class RobotDataset(data.Dataset):
    PRESENT_COLS = ['left_voltage', 'right_voltage', 'dt']
    FUTURE_COLS = ['left_velo', 'right_velo']

    def __init__(self, df : pd.DataFrame, split : str):
        self.indexer = {df.columns[i] : i for i in range(len(df.columns))}
        self.transformer = {
            col : (lambda x : (x - df[col].mean()) / df[col].std())
            for col in df.columns
        }
        self.inverse_transformer = {
            col : (lambda x : x * df[col].std() + df[col].mean())
            for col in df.columns
        }

        df = df.copy().apply(lambda col : self.transformer[col.name](col))
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