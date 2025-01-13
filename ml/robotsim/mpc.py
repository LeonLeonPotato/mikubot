import cvxpy as cp
import numpy as np
import time
from robot import *
import display
import ramsete
import torch
import pygame
import random
from torch.autograd.functional import jacobian
from scipy.linalg import block_diag

track_width = 39
gain = 10.9097943014
time_const = 1.07895202747021748
linear_mult = 4.125 * 0.6
dt = 0.01

def transition(x, u):
    dtheta = (x[3] - x[4]) / track_width * dt
    half = dtheta / 2
    chord = (x[3] + x[4]) * 0.5 * torch.sinc(half) * dt
    
    next_state = torch.zeros_like(x)
    next_state[0] = x[0] + chord * torch.sin(x[2] + half)
    next_state[1] = x[1] + chord * torch.cos(x[2] + half)
    next_state[2] = x[2] + dtheta
    next_state[3] = x[3] + (gain * u[0] - x[3]) / time_const * dt
    next_state[4] = x[4] + (gain * u[1] - x[4]) / time_const * dt
    return next_state

def mpc(n, xref:torch.Tensor, uref:torch.Tensor, poses:np.ndarray, Q, R, P, dt=0.1):
    a, b = jacobian(transition, (xref, uref))
    a = a.detach().numpy(); b = b.detach().numpy()
    xref = xref.numpy(); uref = uref.numpy()
    na = a.shape[1]
    nb = b.shape[1]

    A = [a]
    for i in range(n-1):
        A.append(A[-1] @ a)
    A = np.concatenate(A, axis=0)
    
    B = [[np.zeros((na, nb)) for _ in range(n)] for _ in range(n)]
    for i in range(n):
        track = np.eye(na)
        for j in range(i, -1, -1):
            B[i][j] = track @ b
            track = a @ track
    B = np.block(B)

    U = cp.Variable((nb * n, 1))
    E = poses

    Qb = block_diag(*[Q for _ in range(n-1)] + [P])
    Rb = block_diag(*[R for _ in range(n)])

    H = 2 * (B.T @ Qb @ B + Rb)
    f = 2 * B.T @ Qb @ (A @ xref - E.flatten())

    cost = cp.quad_form(U, H) + f.T @ U
    constraints = [U >= -1, U <= 1]
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()
    if prob.status == cp.OPTIMAL:
        return U.value
    else:
        return None

if __name__ == '__main__':
    pygame.init(); pygame.font.init()

    robot = DifferentialDriveRobot(
        initial_pose=Pose(0, 0, 0),
        right_drivetrain=DifferentialDrivetrain(gain, time_const, linear_mult),
        left_drivetrain=DifferentialDrivetrain(gain, time_const, linear_mult),
        track_width=track_width
    )
    disp = display.Display(track_width, 50)
    buffer = pygame.Surface((800, 600))
    screen = pygame.display.set_mode((800, 600))

    spline = ramsete.TwoDSpline([
        robot.pose,
        robot.pose + Pose(0, 50, 0),
        robot.pose + Pose(48, -320, 0),
        robot.pose + Pose(150, 200, 0),
        robot.pose + Pose(-50, -99, 0)
    ])
    spline.generate_spline(Pose(0, 100, 0), Pose(0, 0, 0))
    spline.construct_profile(ramsete.ProfileParams(
        0, 0, 
        200 * linear_mult, 
        200 * linear_mult, 
        200 * linear_mult, 
        track_width, 
        1.0
    ))

    def draw_path():
        xs = spline.xspline(np.linspace(0, spline.maxt(), 100))
        ys = spline.yspline(np.linspace(0, spline.maxt(), 100))
        coords = [
            (xs[i] + 400, ys[i] + 300)
            for i in range(len(xs))
        ]
        pygame.draw.lines(buffer, (255, 255, 255), False, coords, 2)

    n = 2
    Q = np.array([
        [3, 0, 0, 0, 0],
        [0, 3, 0, 0, 0],
        [0, 0, 100, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ])
    R = np.array([
        [0.00, 0],
        [0, 0.00]
    ])

    last_u = torch.tensor([0, 0], dtype=float)
    tracking_i = 2
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        while tracking_i < len(spline.profile):
            point = spline.profile[tracking_i]
            profiled_pose = spline.pose(point.time_param)
            profiled_deriv = spline.velocity(point.time_param)
            if (profiled_pose - robot.pose).dot(profiled_deriv) <= 0:
                tracking_i += 1
            else:
                break

        E = np.zeros((n, 5))
        for i in range(n):
            _pose = spline.pose(spline.profile[tracking_i + i].time_param)
            E[i] = np.array([
                _pose.x,
                _pose.y,
                _pose.theta,
                0,
                0
            ])
        
        xref = torch.tensor([
            float(robot.pose.x),
            float(robot.pose.y),
            float(robot.pose.theta),
            0.0,
            0.0
        ])
        # U = mpc(n, xref.flatten(), last_u.flatten(), E, Q, R, Q, 0.01)

        # if U is not None:
        #     robot.update(last_u[0] + U[0], last_u[1] + U[1])
        #     last_u = torch.from_numpy(U[:2])

        pos = np.array([100, 100])

        buffer.fill((0, 0, 0))
        draw_path()
        buffer = disp.draw_on_buffer(buffer, robot.get_info())
        pygame.draw.circle(buffer, (0, 255, 0), (profiled_pose.x + 400, 300 - profiled_pose.y), 2)
        screen.blit(buffer, (0, 0))
        pygame.display.flip()
        time.sleep(0.01)

