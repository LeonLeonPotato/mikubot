import models
from dataset import RobotDataset
import torch as th
import constants

def main(args):
    name = args.name
    save_name = args.save_name

    model = models.MotionModel(
        len(RobotDataset.PAST_COLS),
        len(RobotDataset.PRESENT_COLS),
        len(RobotDataset.FUTURE_COLS)
    ).to(constants.device).eval()

    model.load_state_dict(th.load(f"{name}", map_location=constants.device))
    model.eval()

    model = th.compile(model)

    th.export(model, f"{save_name}.pt")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str)
    parser.add_argument("--save_name", type=str)

    args = parser.parse_args()

    main(args)