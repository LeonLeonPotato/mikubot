import pandas as pd

_IDX = 0

def create_new_data(name):
    global _IDX
    df = pd.read_csv(f"ml/motion2/datasets/pure/{name}").dropna()
    df = df.drop(['x', 'y', 'theta', 'left_accel', 'right_accel', 'left_eff', 'right_eff'], axis=1)
    df = df.reset_index(drop=True)

    df.to_csv(f"ml/motion2/datasets/{name}_{_IDX}.csv")
    df[['left_voltage', 'right_voltage']] *= -1
    df[['left_velo', 'right_velo']] *= -1
    df.to_csv(f"ml/motion2/datasets/{name}_{_IDX + 1}.csv")

    _IDX += 2

create_new_data("driving_logs_0.csv")
create_new_data("driving_logs_1.csv")