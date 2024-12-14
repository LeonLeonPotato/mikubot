import robot
import display
import math
import pygame
import time
import numpy as np
import ramsete

wheelsize = 4.125
ratio = 0.6

maxspeed = 140 / (wheelsize * ratio)
maxaccel = 300 / (wheelsize * ratio)
maxdecel = 230 / (wheelsize * ratio)

r = robot.DifferentialDriveRobot(
    initial_pose=robot.Pose(0, 0, 0),
    right_drivetrain=robot.DifferentialDrivetrain(maxspeed, -maxspeed, maxaccel, -maxdecel, wheelsize * ratio),
    left_drivetrain=robot.DifferentialDrivetrain(maxspeed, -maxspeed, maxaccel, -maxdecel, wheelsize * ratio),
    track_width=39
)

pygame.init()
pygame.font.init()
d = display.Display(42, 50)

buffer = pygame.Surface((800, 600))
screen = pygame.display.set_mode((800, 600))

import random

poses1 = [
    r.pose,
    r.pose + robot.Pose(0, 100, 0),
    r.pose + robot.Pose(50, 100, 0),
    r.pose + robot.Pose(50, 200, 0)
]

poses2 = [r.pose]
for i in range(10):
    rand = random.random() * 2 * math.pi
    poses2.append(r.pose + robot.Pose(200 * math.cos(rand), 200 * math.sin(rand), 0))

path = ramsete.TwoDSpline(poses1)
path.generate_spline(robot.Pose(0, 100, 0), robot.Pose(0, 0, 0))
path.construct_profile(ramsete.ProfileParams(0, 0, 
                                             maxspeed*wheelsize*ratio, 
                                             maxaccel*wheelsize*ratio, 
                                             maxdecel*wheelsize*ratio, 
                                             39, 0.05))

def draw_path():
    xs = path.xspline(np.linspace(0, path.maxt(), 100))
    ys = path.yspline(np.linspace(0, path.maxt(), 100))
    coords = [
        (xs[i] + 400, ys[i] + 300)
        for i in range(len(xs))
    ]
    pygame.draw.lines(buffer, (255, 255, 255), False, coords, 2)

tracking_i = 0
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    while tracking_i <= len(path.profile):
        point = path.profile[tracking_i]
        profiled_pose = path.pose(point.time_param)
        profiled_deriv = path.velocity(point.time_param)
        if (profiled_pose - r.pose).dot(profiled_deriv) <= 0.01:
            tracking_i += 1
            if tracking_i >= len(path.profile):
                tracking_i = len(path.profile) - 1
                break
        else:
            break
    # tracking_i += 1
    # point = path.profile[tracking_i]
    # profiled_pose = path.pose(point.time_param)

    v, w = ramsete.ramsete(r, profiled_pose, point.center_v, point.angular_v, 2.0, 0.7)
    # v = point.center_v
    # w = point.angular_v
    v /= wheelsize * ratio
    w /= wheelsize * ratio
    r.update(v + w, v - w)

    buffer.fill((0, 0, 0))
    pygame.draw.circle(buffer, (0, 255, 0), (profiled_pose.x + 400, profiled_pose.y + 300), 2)
    draw_path()
    buffer = d.draw_on_buffer(buffer, {
        'x': r.get_pose().x,
        'y': r.get_pose().y,
        'theta': r.get_pose().theta,
        'left_actual_velocity': r.left_drivetrain.get_angular_velocity(),
        'right_actual_velocity': r.right_drivetrain.get_angular_velocity(),
        'left_actual_accel': r.left_drivetrain.angular_accel <= maxaccel,
        'right_actual_accel': r.right_drivetrain.angular_accel <= maxaccel,
        'clamp': 0,
        'intake': 0,
        'conveyor': 0
    })

    screen.blit(buffer, (0, 0))
    pygame.display.flip()