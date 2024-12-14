import time
import math

class Pose:
    def __init__(self, x, y, theta = None):
        self.x = x
        self.y = y
        self.theta = theta
    
    def has_theta(self) -> bool:
        return self.theta is not None
    
    def forward(self, units) -> 'Pose':
        if not self.has_theta():
            return Pose(self.x, self.y, None)
        return Pose(self.x + math.cos(self.theta)*units, self.y + math.sin(self.theta)*units, self.theta)
    
    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y
    
    def rotate(self, angle) -> 'Pose':
        new_x = self.x * math.cos(angle) - self.y * math.sin(angle)
        new_y = self.x * math.sin(angle) + self.y * math.cos(angle)
        new_theta = self.theta + angle if self.has_theta() else None
        return Pose(new_x, new_y, new_theta)
    
    def minimum_angular_diff(self, angle) -> float:
        return (angle - self.theta + math.pi) % (2*math.pi) - math.pi
    
    def dist(self, other) -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __add__(self, other):
        return Pose(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, self.theta - other.theta)
    
    def __mul__(self, other):
        return Pose(self.x * other, self.y * other, self.theta * other)
    
    def __truediv__(self, other):
        return Pose(self.x / other, self.y / other, self.theta / other)
    
    def __repr__(self):
        return f'Pose(x={self.x}, y={self.y}, theta={self.theta})'

class DifferentialDrivetrain:
    def __init__(self, max_velo:float, min_velo:float, max_accel:float, min_accel:float, wheel_size:float):
        self.cur_velocity:float = 0
        self.max_velo:float = max_velo
        self.min_velo:float = min_velo
        self.max_accel:float = max_accel
        self.min_accel:float = min_accel 
        self.wheel_size:float = wheel_size
        self.angular_accel:float = 0

    def update(self, target_velo, dt) -> float:
        target_velo = min(max(target_velo, self.min_velo), self.max_velo)
        dv = target_velo - self.cur_velocity
        dv = min(max(dv, self.min_accel * dt), self.max_accel * dt)
        self.cur_velocity += dv
        if dt != 0:
            self.angular_accel = dv / dt

        return self.cur_velocity
    
    def get_angular_velocity(self) -> float:
        return self.cur_velocity
    
    def get_linear_velocity(self) -> float:
        return self.cur_velocity * self.wheel_size

class Robot:
    def __init__(self, initial_pose:Pose, dt:float=None):
        self.pose:Pose = initial_pose
        self.velocity:Pose = None
        self.acceleration:Pose = None

        self.force_dt:float = dt
        self.last_update_time:float = -1

    def _pre_update(self) -> None:
        self._pose_cache:Pose = self.pose
        self._velocity_cache:Pose = self.velocity
    
    def _post_update(self, dt) -> None:
        if dt == 0: return
        if self._pose_cache is not None:
            self.velocity = (self.pose - self._pose_cache) / dt
        if self._velocity_cache is not None:
            self.acceleration = (self.velocity - self._velocity_cache) / dt

    def get_dt(self) -> float:
        if self.force_dt is not None:
            return self.force_dt
        else:
            lut = self.last_update_time
            self.last_update_time = time.time()
            if lut == -1:
                return 0
            else:
                return time.time() - lut

    def get_pose(self) -> Pose:
        return self.pose

    def set_pose(self, pose) -> None:
        self.pose = pose

    def get_velocity(self) -> Pose:
        return self.velocity
    
    def get_acceleration(self) -> Pose:
        return self.acceleration


class DifferentialDriveRobot(Robot):
    def __init__(self, initial_pose:Pose, 
                 left_drivetrain:DifferentialDrivetrain, 
                 right_drivetrain:DifferentialDrivetrain, 
                 track_width:float, 
                 dt:float = None):
        super().__init__(initial_pose, dt)
        self.track_width = track_width
        self.left_drivetrain = left_drivetrain
        self.right_drivetrain = right_drivetrain

    def _safe_sinc(x:float) -> float:
        if abs(x) < 1e-4:
            return 1 - x**2 / 6 + x**4 / 120
        else:
            return math.sin(x) / x

    def update(self, left_velo, right_velo) -> Pose:
        dt = self.get_dt()
        self._pre_update()

        self.left_drivetrain.update(left_velo, dt)
        self.right_drivetrain.update(right_velo, dt)

        left_travel = self.left_drivetrain.get_linear_velocity() * dt
        right_travel =  self.right_drivetrain.get_linear_velocity() * dt

        dtheta = (left_travel - right_travel) / self.track_width

        u = dtheta/2
        chord = (left_travel + right_travel) * 0.5 * DifferentialDriveRobot._safe_sinc(u)
        
        dx = chord * math.sin(self.pose.theta + u)
        dy = chord * math.cos(self.pose.theta + u)

        self.pose = self.pose + Pose(dx, dy, dtheta)

        self._post_update(dt)
        return self.pose
