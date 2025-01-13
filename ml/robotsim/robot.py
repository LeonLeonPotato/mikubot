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
    
    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)
    
    def __add__(self, other):
        new_theta = (self.theta + other.theta) if self.has_theta() and other.has_theta() else None
        return Pose(self.x + other.x, self.y + other.y, new_theta)

    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, self.theta - other.theta)
    
    def __mul__(self, other):
        return Pose(self.x * other, self.y * other, self.theta * other)
    
    def __truediv__(self, other):
        return Pose(self.x / other, self.y / other, self.theta / other)
    
    def __repr__(self):
        return f'Pose(x={self.x}, y={self.y}, theta={self.theta})'

class DifferentialDrivetrain:
    def __init__(self, gain:float, time_constant:float, wheel_size:float):
        self.cur_velocity:float = 0
        self.gain:float = gain
        self.time_constant:float = time_constant
        self.wheel_size:float = wheel_size
        self.angular_accel:float = 0

    def update(self, volt, dt) -> float:
        dvdt = 1/self.time_constant * (self.gain * volt - self.cur_velocity)
        self.cur_velocity += dvdt * dt
        if dt != 0:
            self.angular_accel = dvdt

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

    def update(self, left_volt, right_volt) -> Pose:
        dt = self.get_dt()
        self._pre_update()

        self.left_drivetrain.update(left_volt, dt)
        self.right_drivetrain.update(right_volt, dt)

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

    def get_info(self):
        return {
            'x': float(self.pose.x),
            'y': float(self.pose.y),
            'theta': float(self.pose.theta),
            'left_actual_velocity': float(self.left_drivetrain.get_linear_velocity()),
            'right_actual_velocity': float(self.right_drivetrain.get_linear_velocity()),
            'left_actual_accel': float(self.left_drivetrain.angular_accel),
            'right_actual_accel': float(self.right_drivetrain.angular_accel),
            'clamp': 0,
            'intake': 0,
            'conveyor': 0
        }