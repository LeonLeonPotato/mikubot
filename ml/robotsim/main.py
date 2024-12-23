import robot
import display
import math
import pygame
import time
import numpy as np
import ramsete

wheelsize = 4.125
ratio = 0.6

tile = 59.5
maxspeed = 200 / (wheelsize * ratio)
maxaccel = 200 / (wheelsize * ratio)
maxdecel = 200 / (wheelsize * ratio)

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
    r.pose + robot.Pose(tile * 2/3, tile),
    r.pose + robot.Pose(tile, 0),
    r.pose + robot.Pose(tile+50, -tile)
]

poses2 = [r.pose]
for i in range(10):
    rand = random.random() * 2 * math.pi
    poses2.append(r.pose + robot.Pose(200 * math.cos(rand), 200 * math.sin(rand), 0))

path = ramsete.TwoDSpline(poses2)
path.generate_spline(robot.Pose(0, 100, 0), robot.Pose(0, 0, 0))
path.construct_profile_2(ramsete.ProfileParams(0, 0, 
                                             maxspeed * wheelsize * ratio, 
                                             maxaccel * wheelsize * ratio, 
                                             maxdecel * wheelsize * ratio, 
                                             39, 0.1))

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

    lookahead = tracking_i + int(point.center_v < 20)*1
    if lookahead >= len(path.profile):
        lookahead = len(path.profile) - 1
    
    lp = path.profile[lookahead-1]
    p = path.profile[lookahead]
    # tracking_i += 1
    # point = path.profile[tracking_i]
    profiled_pose = path.pose(p.time_param)
    print(profiled_pose.dist(r.pose))

    # max_angular = maxspeed * wheelsize * ratio * -np.sign(point.curvature) / 5
    max_angular = 1000000
    v, w = ramsete.ramsete(r, profiled_pose, p.center_v, min(max_angular, p.angular_v/2, key=abs), 2.0, 0.7)
    # print(-p.angular_v)
    # v = point.center_v
    # w = point.angular_v
    v /= wheelsize * ratio
    w /= wheelsize * ratio
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
        'left_actual_accel': r.left_drivetrain.angular_accel <= maxaccel,
        'right_actual_accel': r.right_drivetrain.angular_accel <= maxaccel,
        'clamp': 0,
        'intake': 0,
        'conveyor': 0
    })

    screen.blit(buffer, (0, 0))
    pygame.display.flip()