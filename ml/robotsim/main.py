import robot
import display
import math
import pygame
import time
import numpy as np
import ramsete

r = robot.DifferentialDriveRobot(
    initial_pose=robot.Pose(0, 0, 0),
    right_drivetrain=robot.DifferentialDrivetrain(62.8, -62.8, 200, -200, 4.25),
    left_drivetrain=robot.DifferentialDrivetrain(62.8, -62.8, 200, -200, 4.25),
    track_width=40
)

pygame.init()
pygame.font.init()
d = display.Display(42, 50)

buffer = pygame.Surface((800, 600))
screen = pygame.display.set_mode((800, 600))

import random
path = ramsete.TwoDSpline([robot.Pose(0, 0, 0)] + [
    robot.Pose(
        random.randrange(-300, 300),
        random.randrange(-200, 200),
        0
    )
    for i in range(10)
])
path.generate_spline(robot.Pose(0, 100, 0), robot.Pose(0, 10, 0))
path.construct_profile(ramsete.ProfileParams(0, 0, 62.8, 200, 50, 40, 0.1))

def draw_path():
    xs = path.xspline(np.linspace(0, path.maxt(), 100))
    ys = path.yspline(np.linspace(0, path.maxt(), 100))
    coords = [
        (xs[i] + 400, ys[i] + 300)
        for i in range(len(xs))
    ]
    pygame.draw.lines(buffer, (255, 255, 255), False, coords, 2)

tracking_i = 1
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    while tracking_i < len(path.profile):
        point = path.profile[tracking_i]
        profiled_pose = path.pose(point.time_param)
        profiled_deriv = path.velocity(point.time_param)
        if (profiled_pose - r.pose).dot(profiled_deriv) <= 0:
            tracking_i += 1
        else:
            break

    v, w = ramsete.ramsete(r, profiled_pose, point.center_v, point.angular_v, 1, 0)
    r.update(v + w, v - w)

    buffer.fill((0, 0, 0))
    draw_path()
    pygame.draw.circle(buffer, (0, 255, 0), (profiled_pose.x + 400, profiled_pose.y + 300), 2)
    buffer = d.draw_on_buffer(buffer, {
        'x': r.get_pose().x,
        'y': r.get_pose().y,
        'theta': r.get_pose().theta,
        'left_actual_velocity': r.left_drivetrain.get_angular_velocity(),
        'right_actual_velocity': r.right_drivetrain.get_angular_velocity(),
        'clamp': 0,
        'intake': 0,
        'conveyor': 0
    })

    screen.blit(buffer, (0, 0))
    pygame.display.flip()