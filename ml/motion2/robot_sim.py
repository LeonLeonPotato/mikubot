import models
import torch as th
import pygame
import time
import numpy as np

from constants import *
from dataset import RobotDataset

def make_initial_state():
    return [th.tensor([
        RobotDataset.INVERSE_TRANSFORMER(0, 'velo')
    ], device=device) for i in range(lookback)]

class RobotArgs:
    def __init__(self, wheel_radius, width):
        self.wheel_radius = wheel_radius
        self.width = width

class Robot:
    DEFAULT_ARGS = RobotArgs(5.08, 35)

    def __init__(self, x, y, theta, robot_args):
        self.x = x
        self.y = y
        self.theta = theta
        self.args : RobotArgs = robot_args

        self.left_velo = 0
        self.right_velo = 0
        
        self.last_time = time.time()

        self.model = models.MotionModel(
            len(RobotDataset.PAST_COLS),
            len(RobotDataset.PRESENT_COLS),
            len(RobotDataset.FUTURE_COLS)
        ).to(device).eval()
        self.model.load_state_dict(th.load(f"ml/motion2/{save_name}", map_location=device))

        self.left_state = make_initial_state()
        self.right_state = make_initial_state()

    def angle_to(self, x, y, dtheta=0):
        da = np.arctan2(x - self.x, y - self.y)
        return (da + dtheta - self.theta + np.pi) % (2 * np.pi) - np.pi
    
    def dist(self, x, y):
        return np.sqrt((self.x - x) * (self.x - x) + (self.y - y) * (self.y - y))

    def _dummy_update(self, left_velo, right_velo, dt):
        if isinstance(left_velo, float):
            left_velo = th.tensor([left_velo], device='cpu')
        if isinstance(right_velo, float):
            right_velo = th.tensor([right_velo], device='cpu')

        rpm2rad = 2 * np.pi / 60
        right_travel = right_velo * rpm2rad * self.args.wheel_radius * dt
        left_travel = left_velo * rpm2rad* self.args.wheel_radius * dt

        dtheta = (left_travel - right_travel) / self.args.width
        if isinstance(dtheta, float): dtheta = th.tensor([dtheta], device='cpu')
        flags = dtheta.abs().lt(0.017)
        dtheta[dtheta == 0] = 0.00000001

        r = right_travel / dtheta + self.args.width / 2
        chord = (r * torch.sin(dtheta / 2))*~flags + left_travel*flags

        dx = chord * torch.sin(self.theta + dtheta)
        dy = chord * torch.cos(self.theta + dtheta)
        
        return dx, dy, dtheta

    def _pos_update(self, dt):
        dx, dy, dtheta = self._dummy_update(self.left_velo, self.right_velo, dt)
        self.theta += dtheta.cpu().item()
        self.x += dx.cpu().item()
        self.y += dy.cpu().item()

    @torch.inference_mode()
    def update(self, left_volt, right_volt, dt=-1):
        left_volt = min(max(-12000, left_volt), 12000)
        right_volt = min(max(-12000, right_volt), 12000)

        t_left_volt = RobotDataset.TRANSFORMER(left_volt, 'volt')
        t_right_volt = RobotDataset.TRANSFORMER(right_volt, 'volt')
        if dt == -1: dt = time.time() - self.last_time
        self.last_time = time.time()

        past = th.stack(self.left_state, dim=0)
        present = th.tensor([t_left_volt], device=device, dtype=th.float32)
        t_left_velo = self.model(past, present)[0].cpu().item()
        self.left_velo = RobotDataset.INVERSE_TRANSFORMER(t_left_velo, 'velo')
        self.left_velo = min(max(-580, self.left_velo), 580)

        past = th.stack(self.right_state, dim=0)
        present = th.tensor([t_right_volt], device=device, dtype=th.float32)
        t_right_velo = self.model(past, present)[0].cpu().item()
        self.right_velo = RobotDataset.INVERSE_TRANSFORMER(t_right_velo, 'velo')
        self.right_velo = min(max(-580, self.right_velo), 580)

        self.left_state.pop(0)
        self.left_state.append(th.tensor([
            RobotDataset.TRANSFORMER(self.left_velo, 'velo')
        ], device=device, dtype=th.float32))

        self.right_state.pop(0)
        self.right_state.append(th.tensor([
            RobotDataset.TRANSFORMER(self.right_velo, 'velo')
        ], device=device, dtype=th.float32))
        
        self._pos_update(dt)

    @torch.inference_mode()
    def best(self, func, dt, low_left=-12000, high_left=12000, low_right=-12000, high_right=12000, n_left=20, n_right=20):
        past = th.stack(self.left_state, dim=0).broadcast_to((n_left, lookback, len(RobotDataset.PAST_COLS)))
        present = th.linspace(low_left, high_left, n_left, device=device)
        present = present.view(-1, len(RobotDataset.PRESENT_COLS))

        present = RobotDataset.TRANSFORMER(present, 'volt')
        left_velos = self.model(past, present).view(n_left)
        left_velos = RobotDataset.INVERSE_TRANSFORMER(left_velos, 'velo')

        past = th.stack(self.right_state, dim=0).broadcast_to((n_right, lookback, len(RobotDataset.PAST_COLS)))
        present = th.linspace(low_right, high_right, n_right, device=device)
        present = present.view(-1, len(RobotDataset.PRESENT_COLS))

        present = RobotDataset.TRANSFORMER(present, 'volt')
        right_velos = self.model(past, present).view(n_right)
        right_velos = RobotDataset.INVERSE_TRANSFORMER(right_velos, 'velo')

        combs = th.cartesian_prod(left_velos, right_velos)
        dx, dy, dtheta = self._dummy_update(combs[:, 0], combs[:, 1], dt)

        scores = func(self, dx, dy, dtheta, left_velos, right_velos)
        best = th.argmin(scores).cpu().item()
        i, j = best // n_right, best % n_right

        best_left = (high_left - low_left) / n_left * i + low_left
        best_right = (high_right - low_right) / n_right * j + low_right
        return best_left, best_right, scores[best].item()

if __name__ == "__main__":
    R = Robot(
        0, 0, 0, RobotArgs(5.08, 98.4375)
    )

    tx, ty = 50, 100

    def func(robot, dx, dy, dtheta, left_velo, right_velo):
        tangle = robot.angle_to(tx, ty, dtheta)
        return abs(tangle)

    t = time.time()
    left, right, score = R.best(func, 0.02, n_left=100, n_right=100)

    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((800, 800))

    while True:
        left, right, score = R.best(func, 0.02, n_left=100, n_right=100)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        R.update(left, right, 0.02)
        R.x = min(max(-400, R.x), 400)
        R.y = min(max(-400, R.y), 400)

        screen.fill((0, 0, 0))

        Rx, Ry = 400 + R.x, 400 + R.y
        print(R.x, R.y, score)
        pygame.draw.circle(screen, (255, 255, 255), (Rx, Ry), 10)
        pygame.draw.line(screen, (255, 255, 255), (Rx, Ry), (Rx + 20 * np.sin(R.theta), Ry + 20 * np.cos(R.theta)))
        pygame.draw.circle(screen, (255, 255, 255), (400 + tx, 400 + ty), 10)

        surf = pygame.font.SysFont('Arial', 30).render(f"L: {left}, R: {right}", True, (255, 255, 255))
        screen.blit(surf, (0, 0))

        surf = pygame.font.SysFont('Arial', 30).render(f"Lv: {R.left_velo}, Rv: {R.right_velo}", True, (255, 255, 255))
        screen.blit(surf, (0, 30))

        sur = pygame.font.SysFont('Arial', 30).render(f"Score: {score}", True, (255, 255, 255))
        screen.blit(sur, (0, 60))

        pygame.display.update()
        time.sleep(0.02)