import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from robot import Pose, DifferentialDriveRobot, DifferentialDrivetrain
import bisect
import math
import bisect

class ProfileParams:
    def __init__(self, start_v:float, end_v:float, max_speed:float, accel:float, decel:float, track_width:float, ds:float=0.1, resolution:int=10000):
        self.start_v = start_v
        self.end_v = end_v
        self.max_speed = max_speed
        self.accel = accel
        self.decel = decel
        self.track_width = track_width
        self.ds = ds
        self.resolution = resolution

class ProfilePoint:
    def __init__(self, s:float, time_param:float, curvature:float, left_v:float, center_v:float, right_v:float, angular_v:float=0):
        self.s = s
        self.time_param = time_param
        self.curvature = curvature
        self.left_v = left_v
        self.center_v = center_v
        self.right_v = right_v
        self.angular_v = angular_v

    def __repr__(self):
        return f"ProfilePoint(s={self.s:.2f}, time_param={self.time_param:.2f}, curvature={self.curvature:.2f}, left_v={self.left_v:.2f}, center_v={self.center_v:.2f}, right_v={self.right_v:.2f}, angular_v={self.angular_v:.2f})"

class TwoDSpline:
    def __init__(self, poses:list[Pose]):
        self.poses:list[Pose] = poses
        self.xspline = None
        self.yspline = None
        self.distances:list[float] = []
        self.profile:list[ProfilePoint] = []
    
    def generate_spline(self, start_velo:Pose, end_velo:Pose) -> None:
        x = [pose.x for pose in self.poses]
        y = [pose.y for pose in self.poses]

        t = np.linspace(0, len(x)-1, len(x))

        bc = ((1, start_velo.x), (1, end_velo.x))
        self.xspline = CubicSpline(t, x, bc_type=bc)

        bc = ((1, start_velo.y), (1, end_velo.y))
        self.yspline = CubicSpline(t, y, bc_type=bc)

    def pose(self, t:float) -> Pose:
        return Pose(self.xspline(t), self.yspline(t), np.atan2(self.xspline(t, 1), self.yspline(t, 1)))
    
    def velocity(self, t:float) -> Pose:
        return Pose(self.xspline(t, 1), self.yspline(t, 1), np.atan2(self.xspline(t, 1), self.yspline(t, 1)))
    
    def acceleration(self, t:float) -> Pose:
        return Pose(self.xspline(t, 2), self.yspline(t, 2), np.atan2(self.xspline(t, 2), self.yspline(t, 2)))
    
    def curvature(self, t:float) -> float:
        d1 = self.velocity(t)
        d2 = self.acceleration(t)
        return (d1.x * d2.y - d1.y * d2.x) / ((d1.x ** 2 + d1.y ** 2) ** 1.5 + 1e-6)
    
    def maxt(self) -> float:
        return len(self.poses) - 1
    
    def gen_arc_list(self, res:int) -> list[float]:
        ts = np.linspace(0, self.maxt(), res)
        xs = self.xspline(ts)
        ys = self.yspline(ts)
        self.distances = [0]
        for i in range(1, len(xs)):
            self.distances.append(self.distances[-1] + np.sqrt((xs[i] - xs[i-1])**2 + (ys[i] - ys[i-1])**2))
    
    def construct_profile(self, params:ProfileParams) -> None:
        self.gen_arc_list(params.resolution)

        total_length = self.distances[-1]
        num_points = math.ceil(total_length / params.ds) + 1
        self.profile = []

        center_v = params.start_v
        i = 0
        for s in (params.ds * step for step in range(num_points)):
            if s > total_length:
                break
            i = bisect.bisect_left(self.distances, s, lo=i)
            time_param = i / (len(self.distances) - 1) * (len(self.poses) - 1)
            curve = self.curvature(time_param)
            scale = abs(curve) * params.track_width / 2.0
            self.profile.append(ProfilePoint(s, time_param, curve, 0, center_v, 0))
            center_v = min(
                math.sqrt(center_v**2 + 2 * params.accel * params.ds),
                params.max_speed / (1 + scale)
            )

        center_v = params.end_v
        for i in range(len(self.profile) - 1, -1, -1):
            p = self.profile[i]
            scale = p.curvature * params.track_width / 2.0
            p.center_v = min(p.center_v, center_v)
            p.left_v = max(min(p.center_v * (1 - scale), params.max_speed), -params.max_speed)
            p.right_v = max(min(p.center_v * (1 + scale), params.max_speed), -params.max_speed)
            p.angular_v = (p.left_v - p.right_v) / params.track_width
            center_v = min(
                math.sqrt(center_v**2 + 2 * params.decel * params.ds),
                params.max_speed / (1 + abs(scale))
            )

def ramsete(robot:DifferentialDriveRobot, desired_pose, desired_velocity, desired_angular, beta, zeta):
    error = desired_pose - robot.pose
    error = error.rotate(robot.pose.theta)
    theta_error = robot.pose.minimum_angular_diff(desired_pose.theta)

    k = 2 * zeta * np.sqrt(desired_angular ** 2 + beta * desired_velocity ** 2)
    v = desired_velocity * np.cos(theta_error) + beta*error.y
    w = desired_angular + k*theta_error + beta*desired_velocity*np.sin(theta_error) + beta*error.x
    return v, w

if __name__ == "__main__":
    r = DifferentialDriveRobot(
        initial_pose=Pose(0, 0, 0),
        right_drivetrain=DifferentialDrivetrain(40, -40, 200, -200, 4.25),
        left_drivetrain=DifferentialDrivetrain(40, -40, 200, -200, 4.25),
        track_width=40
    )
    path = TwoDSpline([
        r.pose,
        r.pose + Pose(0, 100, 0),
        r.pose + Pose(100, 100, 0),
        r.pose + Pose(100, 200, 0),
        r.pose + Pose(-200, 200, 0)
    ])
    path.generate_spline(Pose(0, 100, 0), Pose(0, 0, 0))
    path.construct_profile(ProfileParams(0, 0, 40, 200, 10, 40, 0.1))

    y = [p.center_v for p in path.profile]
    plt.plot(y, label='center_v')

    y = [p.left_v for p in path.profile]
    plt.plot(y, label='left_v')

    y = [p.right_v for p in path.profile]
    plt.plot(y, label='right_v')
    plt.legend()
    plt.show()