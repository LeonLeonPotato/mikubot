import cvxpy as cp
import numpy as np
import time
from robot import *
import display
import ramsete
import torch
import pygame
import random

def mpc(n, x0, poses, dt=0.1):
    u = cp.Variable((n+1, 2))
    def get_A(k):
        return np.array([
            [1, 0, u[k, 0] * np.cos(x0[2]) * dt],
            [0, 1, -u[k, 0] * np.sin(x0[2]) * dt],
            [0, 0, 1]
        ])
    B0 = np.array([
        [np.sin(x0[2]) * dt, 0],
        [np.cos(x0[2]) * dt, 0],
        [0, dt]
    ])

    Ahat = [get_A(0)]
    for i in range(n):
        Ahat.append(get_A(i) @ Ahat[-1])
    
    Bhat = np.empty((n+1, n+1, 3, 2), dtype=object)
    for i in range(n+1):
        track = np.eye(3)
        for j in range(i, -1, -1):
            Bhat[i, j] = track @ B0
            track = get_A(j) @ track

def transition(x, u, track_width, linear_mult, dt):
    ut = u * linear_mult * dt
    dtheta = (ut[0] - ut[1]) / track_width
    half = dtheta / 2
    chord = (ut[0] + ut[1]) * 0.5 * torch.sinc(half)
    x[0] = x[0] + chord * torch.sin(x[2] + half)
    x[1] = x[1] + chord * torch.cos(x[2] + half)
    x[2] = x[2] + dtheta

def nmpc(n, x0, Q, R, U, last_u, poses, track_width, linear_mult, dt=0.02, lr=5, steps=5):
    poses = [(float(pose.x), float(pose.y), float(pose.theta)) for pose in poses]
    poses = torch.tensor(poses)
    poses = poses.repeat(n, 1)
    x0 = torch.tensor((float(x0.x), float(x0.y), float(x0.theta)))
    u = torch.rand((n, 2))
    u *= 200
    u.requires_grad = True
    optimizer = torch.optim.SGD([u], lr=lr, momentum=0.9, weight_decay=0.01)

    for tick in range(steps):
        optimizer.zero_grad()
        cost = 0.0
        x = x0.clone().detach()
        for i in range(n):
            transition(x, u[i], track_width, linear_mult, dt)
            state_diff = x - poses[i]
            state_cost = state_diff @ (Q * i / n + 1) @ state_diff
            control_cost = u[i] @ R @ u[i]
            if i > 0:
                du_cost = u[i] - u[i-1]
                cost += du_cost @ U @ du_cost
            else:
                du_cost = u[i] - last_u
                cost += du_cost @ U @ du_cost
            cost += state_cost + control_cost
        cost.backward()
        optimizer.step()
        u.data = torch.clamp(u.data, -100, 100)
    print("Cost:", cost.item())
    return u

def main():
    wheel_size = 4.125
    linear_mult = wheel_size * 0.6
    max_speed = 100
    max_accel = 200
    max_decel = 200
    track_width = 40

    robot = DifferentialDriveRobot(
        Pose(-5, -5, 0),
        left_drivetrain=DifferentialDrivetrain(max_speed, -max_speed, max_accel, -max_decel, linear_mult),
        right_drivetrain=DifferentialDrivetrain(max_speed, -max_speed, max_accel, -max_decel, linear_mult),
        track_width=track_width,
        dt=0.01
    )
    random_poses = [robot.pose]
    for i in range(10):
        rand = random.random() * 2 * math.pi
        random_poses.append(robot.pose + Pose(200 * math.cos(rand), 200 * math.sin(rand), 0))
    spline = ramsete.TwoDSpline(random_poses)
    spline.generate_spline(Pose(0, 100, 0), Pose(0, 0, 0))
    spline.construct_profile(ramsete.ProfileParams(
        start_v=0, end_v=0, 
        max_speed=max_speed * linear_mult, 
        accel=max_accel * linear_mult, 
        decel=max_decel * linear_mult, 
        track_width=track_width, 
        ds=5.0
    ))

    n = 16
    Q = torch.diag(torch.tensor([1, 1, 0.0], dtype=torch.float32))
    R = torch.eye(2) * 0.0
    U = torch.eye(2) * 0.1

    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((800, 600))
    buffer = pygame.Surface((800, 600))
    disp = display.Display(track_width, 50)
    last_goal = 1

    xs = spline.xspline(np.linspace(0, spline.maxt(), 100))
    ys = spline.yspline(np.linspace(0, spline.maxt(), 100))
    coords = [
        (xs[i] + 400, ys[i] + 300)
        for i in range(len(xs))
    ]

    def draw_path():
        pygame.draw.lines(buffer, (255, 255, 255), False, coords, 2)

    u = torch.tensor([0.0, 0.0], dtype=torch.float32)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        
        while last_goal < len(spline.profile):
            point = spline.profile[last_goal]
            profiled_pose = spline.pose(point.time_param)
            profiled_deriv = spline.velocity(point.time_param)
            if (profiled_pose - robot.pose).dot(profiled_deriv) <= 0:
                last_goal += 1
            else:
                break
        
        if last_goal >= len(spline.profile):
            last_goal = len(spline.profile) - 1

        buffer.fill((0, 0, 0))

        poses = spline.profile[last_goal:last_goal+n]
        poses = [spline.pose(point.time_param) for point in poses]
        u = nmpc(n, robot.pose, Q, R, U, u, poses, track_width, linear_mult)
        u = u.detach().cpu()[0]
        robot.update(u[0], u[1])

        draw_path()
        buffer = disp.draw_on_buffer(buffer, robot.get_info())
        pygame.draw.circle(buffer, (255, 0, 0), (int(poses[0].x) + 400, 300 - int(poses[0].y)), 5)

        time.sleep(0.01)
        screen.blit(buffer, (0, 0))
        pygame.display.flip()


if __name__ == '__main__':
    main()