import pandas as pd
import numpy as np

df_stats = pd.DataFrame(columns=['volt', 'velo'])

def create_new_data(name):
    global df_stats
    idx = 0
    df = pd.read_csv(f"ml/motion2/datasets/pure/{name}").dropna()
    df = df.drop(['x', 'y', 'theta', 'left_eff', 'right_eff'], axis=1)
    df = df.reset_index(drop=True)
    df = df.astype(np.float32)

    lognum = int(name[-5])

    copy = df.copy()[['left_voltage', 'left_velo']]
    copy.columns = ['volt', 'velo']

    copy.to_csv(f"ml/motion2/datasets/{lognum}{idx}.csv", index=False); idx += 1
    df_stats = pd.concat([df_stats, copy], axis=0)

    copy[['volt', 'velo']] *= -1

    copy.to_csv(f"ml/motion2/datasets/{lognum}{idx}.csv", index=False); idx += 1
    df_stats = pd.concat([df_stats, copy], axis=0)

    copy = df.copy()[['right_voltage', 'right_velo']]
    copy.columns = ['volt', 'velo']

    copy.to_csv(f"ml/motion2/datasets/{lognum}{idx}.csv", index=False); idx += 1
    df_stats = pd.concat([df_stats, copy], axis=0)

    copy[['volt', 'velo']] *= -1

    copy.to_csv(f"ml/motion2/datasets/{lognum}{idx}.csv", index=False); idx += 1
    df_stats = pd.concat([df_stats, copy], axis=0)

create_new_data("driving_logs_0.csv")
create_new_data("driving_logs_1.csv")

print(df_stats.describe())