import robot
import time

r = robot.DifferentialDriveRobot(
    initial_pose=robot.Pose(0, 0, 0),
    right_drivetrain=robot.DifferentialDrivetrain(62.8, -62.8, 10000, -10000, 4.25),
    left_drivetrain=robot.DifferentialDrivetrain(62.8, -62.8, 10000, -10000, 4.25),
    track_width=40
)

print("")
while True:
    r.update(62.8, 0)
    print(r.get_velocity())