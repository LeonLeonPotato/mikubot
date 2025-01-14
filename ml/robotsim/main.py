import robot
import display
import math
import pygame
import time
import numpy as np
import ramsete

linear_mult = 4.125 * 0.6

tile = 59.5
gain = 10.9097943014
time_constant = 0.685726606348

r = robot.DifferentialDriveRobot(
    initial_pose=robot.Pose(0, 0, 0),
    right_drivetrain=robot.DifferentialDrivetrain(gain, time_constant, linear_mult),
    left_drivetrain=robot.DifferentialDrivetrain(gain, time_constant, linear_mult),
    track_width=39
)

pygame.init()
pygame.font.init()
d = display.Display(42, 50)

buffer = pygame.Surface((800, 600))
screen = pygame.display.set_mode((800, 600))

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self._last_time = 0
        self._last_error = 0
        self._registered_first = False
    
    def reset(self):
        self.integral = 0
        self._last_time = 0
        self._last_error = 0
        self._registered_first = False

    def update(self, error, dt = -99999):
        if not self._registered_first:
            self._last_time = time.time() - 1
            self._last_error = error
            self._registered_first = True
        
        dt = time.time() - self._last_time
        self.integral += error * dt
        derivative = (error - self._last_error) / dt
        self._last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

linear_pid = PID(0.1, 0.0, 0.0)
angular_pid = PID(20, 0.0, 0.0)
lead = 0.5

# for angle in range(0, 420, 20):
#     angle = angle * np.pi / 180
#     print(angle, r.pose.minimum_angular_diff(angle, True))

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    desired = robot.Pose(-50, -50, -np.pi/2)
    carrot = desired + robot.Pose(np.sin(desired.theta), np.cos(desired.theta), 0) * -lead * r.pose.dist(desired)
    angle_diff = r.pose.minimum_angular_diff(carrot, True)
    linear_error = (desired - r.pose).norm()
    linear = linear_pid.update(linear_error)
    angular = angular_pid.update(angle_diff)

    print(angle_diff, linear_error, linear, angular)

    linear = -linear
    r.update((linear + angular) / 12, (linear - angular) / 12)

    buffer.fill((0, 0, 0))

    pygame.draw.circle(buffer, (255, 0, 0), (int(carrot.x) + 400, int(carrot.y) + 300), 5)
    pygame.draw.circle(buffer, (0, 255, 0), (int(desired.x) + 400, int(desired.y) + 300), 5)

    buffer = d.draw_on_buffer(buffer, r.get_info())
    screen.blit(buffer, (0, 0))
    pygame.display.flip()