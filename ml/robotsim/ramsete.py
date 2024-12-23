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
        return Pose(self.xspline(t), self.yspline(t), np.arctan2(self.xspline(t, 1), self.yspline(t, 1)))
    
    def velocity(self, t:float) -> Pose:
        return Pose(self.xspline(t, 1), self.yspline(t, 1), np.arctan2(self.xspline(t, 1), self.yspline(t, 1)))
    
    def acceleration(self, t:float) -> Pose:
        return Pose(self.xspline(t, 2), self.yspline(t, 2), np.arctan2(self.xspline(t, 2), self.yspline(t, 2)))
    
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
        t = np.linspace(0, self.maxt(), params.resolution+1)
        computed = [self.pose(ti) for ti in t]
        self.distances = [0 for _ in range(params.resolution+1)]

        self.profile = [
            ProfilePoint(0, 0, self.curvature(0), 0, 0, 0)
        ]

        i = 1; j = 0; s = params.ds
        last_angular_vel = 0
        while i < params.resolution:
            while i <= params.resolution:
                self.distances[i] = self.distances[i-1] + computed[i].dist(computed[i-1])
                if self.distances[i] >= s:
                    break
                i += 1

            if i > params.resolution: i = params.resolution
            time_param = i / params.resolution * self.maxt()
            curve = self.curvature(time_param)
            ds = self.distances[i] - self.distances[j]
            
            vel = self.profile[-1].center_v
            angular_vel = vel * curve
            angular_accel = (angular_vel - last_angular_vel) * (vel / params.ds)
            last_angular_vel = angular_vel

            accel = max(0, params.accel - abs(angular_accel * params.track_width / 2.0))
            center_v = np.clip(
                signed_sqrt(signed_square(vel) + 2*accel*params.ds),
                -max_speed(curve, params),
                max_speed(curve, params)
            )
            self.profile.append(ProfilePoint(self.distances[i], time_param, curve, 0, center_v, 0))

            s += params.ds
            j = i

        __scale = self.profile[-1].curvature * params.track_width / 2.0
        self.profile[-1].center_v = 0.0001
        self.profile[-1].left_v = 0.0001
        self.profile[-1].right_v = 0
        self.profile[-1].angular_v = 0

        last_angular_vel = 0
        vel = 0.0001
        for i in range(len(self.profile)-2, 0, -1):
            lp = self.profile[i+1]
            p = self.profile[i]

            scale = p.curvature * params.track_width / 2.0
            ds = lp.s - p.s

            vel = lp.center_v
            angular_vel = vel * p.curvature
            angular_accel = (angular_vel - last_angular_vel) * (vel / params.ds)
            last_angular_vel = angular_vel

            decel = max(0, params.decel - abs(angular_accel * params.track_width / 2.0))
            vel = np.clip(
                signed_sqrt(signed_square(vel) + 2*decel*params.ds),
                -max_speed(curve, params),
                max_speed(curve, params)
            )
            p.center_v = min(p.center_v, vel)
            p.left_v = np.clip(p.center_v * (1 - scale), -params.max_speed, params.max_speed)
            p.right_v = np.clip(p.center_v * (1 + scale), -params.max_speed, params.max_speed)
            p.angular_v = p.center_v * -p.curvature * params.track_width / 2.0

    def construct_profile_2(self, params:ProfileParams) -> None:
        t = np.linspace(0, self.maxt(), params.resolution+1)
        computed = [self.pose(ti) for ti in t]
        self.distances = [0 for _ in range(params.resolution+1)]

        self.profile = [
            ProfilePoint(0, 0, self.curvature(0), 0, 1, 0)
        ]

        i = 1; j = 0; s = params.ds
        while i < params.resolution:
            while i <= params.resolution:
                self.distances[i] = self.distances[i-1] + computed[i].dist(computed[i-1])
                if self.distances[i] >= s:
                    break
                i += 1

            if i > params.resolution: i = params.resolution
            time_param = i / params.resolution * self.maxt()
            curve = self.curvature(time_param)
            ds = self.distances[i] - self.distances[j]
            center_scale = 1 + abs(curve) * params.track_width / 2.0
            left_scale = 1 - curve * params.track_width / 2.0
            right_scale = 1 + curve * params.track_width / 2.0

            vel = self.profile[-1].center_v
            lv = self.profile[-1].left_v
            rv = self.profile[-1].right_v
            vel = np.clip(
                signed_sqrt(signed_square(vel) + 2*params.accel*ds / center_scale),
                -params.max_speed / center_scale, params.max_speed / center_scale
            )
            while True:
                lv2 = np.clip(
                    signed_sqrt(signed_square(lv) + 2*params.accel*ds),
                    -vel * left_scale, vel * left_scale
                )
                rv2 = np.clip(
                    signed_sqrt(signed_square(rv) + 2*params.accel*ds),
                    -vel * right_scale, vel * right_scale
                )
                vel = min(vel, lv2 / left_scale, rv2 / right_scale)
                if lv2 == lv and rv2 == rv:
                    break
                lv = lv2
                rv = rv2
            self.profile.append(ProfilePoint(self.distances[i], time_param, curve, lv, vel, rv))

            s += params.ds
            j = i

        __scale = self.profile[-1].curvature * params.track_width / 2.0
        self.profile[-1].center_v = 0
        self.profile[-1].left_v = 0
        self.profile[-1].right_v = 0
        self.profile[-1].angular_v = 0

        for i in range(len(self.profile)-2, 0, -1):
            lp = self.profile[i+1]
            p = self.profile[i]

            ds = lp.s - p.s
            center_scale = (1 + abs(p.curvature) * params.track_width / 2.0)
            left_scale = 1 - p.curvature * params.track_width / 2.0
            right_scale = 1 + p.curvature * params.track_width / 2.0

            vel = lp.center_v
            lv = lp.left_v
            rv = lp.right_v

            vel = np.clip(
                signed_sqrt(signed_square(vel) + 2*params.decel*ds / center_scale),
                -params.max_speed / center_scale, params.max_speed / center_scale
            )
            while True:
                lv2 = np.clip(
                    signed_sqrt(signed_square(lv) + 2*params.decel*ds),
                    -vel * left_scale, vel * left_scale
                )
                rv2 = np.clip(
                    signed_sqrt(signed_square(rv) + 2*params.decel*ds),
                    -vel * right_scale, vel * right_scale
                )
                vel = min(vel, lv2 / left_scale, rv2 / right_scale)
                if lv2 == lv and rv2 == rv:
                    break
                lv = lv2
                rv = rv2


            p.center_v = min(p.center_v, vel)
            p.left_v = min(p.left_v, lv)
            p.right_v = min(p.right_v, rv)
            p.angular_v = (p.left_v - p.right_v) / 2
            # p.angular_v = p.center_v * -p.curvature * params.track_width / 2.0

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
    r = DifferentialDriveRobot(
        initial_pose=Pose(0, 0, 0),
        right_drivetrain=DifferentialDrivetrain(40, -40, 200, -200, 4.25),
        left_drivetrain=DifferentialDrivetrain(40, -40, 200, -200, 4.25),
        track_width=40
    )
    path = TwoDSpline([
        r.pose,
        r.pose + Pose(0, 100, 0),
        r.pose + Pose(100, 0, 0),
        r.pose
    ])
    path.generate_spline(Pose(0, 10, 0), Pose(0, 0, 0))

    maxspeed = 1000
    maxaccel = 300
    maxdecel = 237
    path.construct_profile_2(ProfileParams(
        0, 0, maxspeed, maxaccel, maxdecel, 39, 0.1, resolution=10000
    ))

    # P = eval(open("ml/robotsim/output.txt").read())
    # plt.plot(
    #     [p[0] for p in P],
    #     [p[1] for p in P]
    # )
    x = [p.s for p in path.profile]

    y = [p.center_v for p in path.profile]
    plt.plot(x, y, label='center_v')

    y = [p.left_v for p in path.profile]
    plt.plot(x, y, label='left_v')

    y = [p.right_v for p in path.profile]
    plt.plot(x, y, label='right_v')

    plt.legend()
    plt.show()

    with open("ml/robotsim/output.txt", "w") as f:
        f.write(str([(p.s, float(p.left_v)) for p in path.profile]))