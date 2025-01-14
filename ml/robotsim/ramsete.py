import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from robot import Pose, DifferentialDriveRobot, DifferentialDrivetrain
import bisect
import math
import bisect

def signed_sqrt(x:float) -> float:
    return math.copysign(math.sqrt(abs(x)), x)
    # return math.sqrt(x)

def signed_square(x:float) -> float:
    return math.copysign(x**2, x)

def max_speed(curve, params) -> float:
    # double max_turn_speed = ((2 * this->max_vel / this->track_width) * this->max_vel) / (fabs(curvature) * this->max_vel + (2 * this->max_vel / this->track_width));
    return ((2 * params.max_speed / params.track_width) * params.max_speed) / (abs(curve) * params.max_speed + (2 * params.max_speed / params.track_width))

class ProfileParams:
    def __init__(self, start_v:float, end_v:float, max_speed:float, accel:float, decel:float, track_width:float, ds:float=0.1, dt:float=0.01, resolution:int=10000):
        self.start_v = start_v
        self.end_v = end_v
        self.max_speed = max_speed
        self.accel = accel
        self.decel = decel
        self.track_width = track_width
        self.ds = ds
        self.dt = dt
        self.resolution = resolution

class ProfilePoint:
    def __init__(self, s:float, t:float, time_param:float, curvature:float, left_v:float, center_v:float, right_v:float, angular_v:float=0):
        self.s = s
        self.t = t
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
        self.xspline = CubicSpline(t, x, bc_type='natural')

        bc = ((1, start_velo.y), (1, end_velo.y))
        self.yspline = CubicSpline(t, y, bc_type='natural')

    def pose(self, t:float) -> Pose:
        return Pose(
            float(self.xspline(t)), 
            float(self.yspline(t)), 
            float(np.arctan2(self.xspline(t, 1), self.yspline(t, 1)))
        )
    
    def velocity(self, t:float) -> Pose:
        return Pose(
            float(self.xspline(t, 1)), 
            float(self.yspline(t, 1)), 
            float(np.arctan2(self.xspline(t, 1), self.yspline(t, 1)))
        )
    
    def acceleration(self, t:float) -> Pose:
        return Pose(
            float(self.xspline(t, 2)), 
            float(self.yspline(t, 2)), 
            float(np.arctan2(self.xspline(t, 1), self.yspline(t, 1)))
        )
    
    def curvature(self, t:float) -> float:
        d1 = self.velocity(t)
        d2 = self.acceleration(t)
        return (d1.y * d2.x - d1.x * d2.y) / ((d1.norm() ** 3) + 1e-6)
    
    def maxt(self) -> float:
        return len(self.poses) - 1
    
    def gen_arc_list(self, res:int) -> list[float]:
        ts = np.linspace(0, self.maxt(), res)
        xs = self.xspline(ts)
        ys = self.yspline(ts)
        self.distances = [0]
        for i in range(1, len(xs)):
            self.distances.append(self.distances[-1] + np.sqrt((xs[i] - xs[i-1])**2 + (ys[i] - ys[i-1])**2))
    
    def construct_profile_time_based(self, params:ProfileParams) -> None:
        t = np.linspace(0, self.maxt(), params.resolution+1)
        computed = [self.pose(ti) for ti in t]
        self.distances = [0]
        for i in range(1, len(computed)):
            self.distances.append(self.distances[-1] + (computed[i] - computed[i-1]).norm())

        self.profile = [
            ProfilePoint(0, 0, 0, self.curvature(0), 0, params.start_v, 0)
        ]

        t = params.dt; s = 0
        time_param = 0
        while time_param <= self.maxt():
            curve = self.curvature(time_param)
            
            scale = 1 + abs(curve) * params.track_width / 2
            vel = self.profile[-1].center_v + params.accel * params.dt / scale 
            vel = min(vel, params.max_speed / scale)
            self.profile.append(ProfilePoint(s, t, time_param, curve, 0, vel, 0))

            t += params.dt
            ds = self.profile[-1].center_v * params.dt + 0.5 * params.accel / scale * params.dt ** 2
            s += ds
            time_param += ds / (self.velocity(time_param).norm() + 1e-6)

        self.profile[-1] = ProfilePoint(s, t, self.maxt(), self.curvature(self.maxt()), 0, params.end_v, 0)

        for i in range(len(self.profile) - 2, -1, -1):
            last = self.profile[i+1]
            current = self.profile[i]
            
            scale = 1 + abs(current.curvature) * params.track_width / 2
            last_scale = 1 + abs(last.curvature) * params.track_width / 2
            vel = last.center_v + params.decel * params.dt / scale
            vel = min(vel, current.center_v, params.max_speed / scale)
            left = vel * (1 + current.curvature * params.track_width / 2)
            right = vel * (1 - current.curvature * params.track_width / 2)
            self.profile[i] = ProfilePoint(current.s, current.t, current.time_param, current.curvature, left, vel, right)

def ramsete(robot:DifferentialDriveRobot, desired_pose, desired_velocity, desired_angular, beta, zeta):
    error = desired_pose - robot.pose
    error = error.rotate(robot.pose.theta)
    theta_error = robot.pose.minimum_angular_diff(desired_pose.theta)
    error = Pose(error.x/100, error.y/100, theta_error)
    safe_sinc = DifferentialDriveRobot._safe_sinc

    k = 2 * zeta * np.sqrt(desired_angular**2 + beta*desired_velocity**2)
    v = desired_velocity * np.cos(theta_error) + k*error.y
    w = desired_angular + k*theta_error + beta*desired_velocity*safe_sinc(theta_error)*error.x
    return v, w

if __name__ == "__main__":
    robot = DifferentialDriveRobot(
        initial_pose=Pose(0, 0, 0),
        right_drivetrain=DifferentialDrivetrain(10.9, 0.07, 4.25),
        left_drivetrain=DifferentialDrivetrain(10.9, 0.07, 4.25),
        track_width=40
    )
    path = TwoDSpline([
        robot.pose,
        robot.pose + Pose(0, 50, 0),
        robot.pose + Pose(48, -320, 0),
        robot.pose + Pose(150, 200, 0),
        robot.pose + Pose(-50, -99, 0)
    ])
    path.generate_spline(Pose(10, 10, 0), Pose(0, 0, 0))

    maxspeed = 100
    maxaccel = 100
    maxdecel = 100
    path.construct_profile_time_based(ProfileParams(
        0, 0, maxspeed, maxaccel, maxdecel, 39, 0.1, resolution=10000, dt=0.001
    ))

    # P = eval(open("ml/robotsim/output.txt").read())
    # plt.plot(
    #     [p[0] for p in P],
    #     [p[1] for p in P]
    # )
    x = [p.t for p in path.profile]

    y = [p.center_v for p in path.profile]
    plt.plot(x, y, label='center_v')

    y = [p.left_v for p in path.profile]
    plt.plot(x, y, label='left_v')

    y = [p.right_v for p in path.profile]
    plt.plot(x, y, label='right_v')

    # y = [(100 / (1 + abs(p.curvature) * 39/2)) for p in path.profile]
    # plt.plot(x, y)

    plt.legend()
    plt.show()

    # with open("ml/robotsim/output.txt", "w") as f:
    #     f.write(str([(p.s, float(p.left_v)) for p in path.profile]))