import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import torch.utils.data as data
import constants
from collections import namedtuple

class RobotDataset(data.Dataset):
    def __init__(self, X, df):
        self.X = X
        self.indexer = get_indexer(df)
        self.present_indicies = [self.indexer[col] for col in ['left_voltage', 'right_voltage', 'dt']]
        self.future_indicies = [self.indexer[col] for col in ['dx', 'dy', 'dtheta', 'speed', 'left_velo', 'right_velo', 'left_eff', 'right_eff']]

    def __len__(self):
        return len(self.X) - constants.lookback - 1

    def __getitem__(self, idx):
        return self.X[idx:idx + constants.lookback], self.X[idx + constants.lookback, self.present_indicies], self.X[idx + constants.lookback, self.future_indicies]

def std_norm(col : pd.Series) -> pd.Series:
    return (col - col.mean()) / col.std()

def normalize(df : pd.DataFrame) -> pd.DataFrame:
    df['left_voltage'] /= 12000
    df['right_voltage'] /= 12000
    df['left_velo'] /= 580
    df['right_velo'] /= 580
    df['left_accel'] = std_norm(df['left_accel'])
    df['right_accel'] = std_norm(df['right_accel'])
    df['left_eff'] /= 100
    df['right_eff'] /= 100
    df['dt'] /= 20000
    df['dx'] = std_norm(df['dx'])
    df['dy'] = std_norm(df['dy'])
    df['dtheta'] = std_norm(df['dtheta'])
    df['speed'] = std_norm(df['speed'])
    return df

def process(df : pd.DataFrame) -> pd.DataFrame:
    df['dx'] = df['x'].diff()
    df['dy'] = df['y'].diff()
    df['dtheta'] = df['theta'].diff()
    df['speed'] = np.sqrt(df['dx']**2 + df['dy']**2)
    df['cos'] = np.cos(df['theta'])
    df['sin'] = np.sin(df['theta'])
    return df

def load_processed_df(name) -> pd.DataFrame:
    df = pd.read_csv(f"ml/motion/{name}").dropna()
    df = process(df)
    df = normalize(df)
    return df.dropna().drop(['x', 'y', 'theta'], axis=1).reset_index(drop=True)

def get_indexer(df):
    if type(df) == str:
        df = pd.read_csv(f"ml/motion/{df}").dropna()
        df = process(df)
        df = normalize(df)

    return {df.columns[i] : i for i in range(len(df.columns))}

def get_transformer(df):
    if type(df) == str:
        df = pd.read_csv(f"ml/motion/{df}").dropna()
        df = process(df)
        df = df.drop(['x', 'y', 'theta'], axis=1)
        df = df.dropna().reset_index(drop=True)

    means, stds = df.mean(), df.std()

    return {
        'left_voltage': lambda x: x / 12000,
        'right_voltage': lambda x: x / 12000,
        'left_velo': lambda x: x / 580,
        'right_velo': lambda x: x / 580,
        'left_accel': lambda x: (x - means['left_accel']) / stds['left_accel'],
        'right_accel': lambda x: (x - means['right_accel']) / stds['right_accel'],
        'left_eff': lambda x: x / 100,
        'right_eff': lambda x: x / 100,
        'dt': lambda x: x / 20000,
        'dx': lambda x: (x - means['dx']) / stds['dx'],
        'dy': lambda x: (x - means['dy']) / stds['dy'],
        'dtheta': lambda x: (x - means['dtheta']) / stds['dtheta'],
        'speed': lambda x: (x - means['speed']) / stds['speed']
    }

def get_inverse_transformer(df):
    if type(df) == str:
        df = pd.read_csv(f"ml/motion/{df}").dropna()
        df = process(df)
        df = df.drop(['x', 'y', 'theta'], axis=1)
        df = df.dropna().reset_index(drop=True)

    means, stds = df.mean(), df.std()

    return {
        'left_voltage': lambda x: x * 12000,
        'right_voltage': lambda x: x * 12000,
        'left_velo': lambda x: x * 580,
        'right_velo': lambda x: x * 580,
        'left_accel': lambda x: x * stds['left_accel'] + means['left_accel'],
        'right_accel': lambda x: x * stds['right_accel'] + means['right_accel'],
        'left_eff': lambda x: x * 100,
        'right_eff': lambda x: x * 100,
        'dt': lambda x: x * 20000,
        'dx': lambda x: x * stds['dx'] + means['dx'],
        'dy': lambda x: x * stds['dy'] + means['dy'],
        'dtheta': lambda x: x * stds['dtheta'] + means['dtheta'],
        'speed': lambda x: x * stds['speed'] + means['speed']
    }

if __name__ == "__main__":
    df = load_processed_df('driving_logs_0.csv')
    dataset = RobotDataset(df.values, df)
    
    df.hist(bins=100)
    plt.show()