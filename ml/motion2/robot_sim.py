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

    def _pos_update(self, dt):
        rpm2rad = 2 * np.pi / 60
        right_travel = self.right_velo * self.args.wheel_radius * rpm2rad * dt
        left_travel = self.left_velo * self.args.wheel_radius * rpm2rad * dt

        dtheta = (left_travel - right_travel) / self.args.width
        self.theta += dtheta

        if abs(dtheta) < 0.017:
            self.x += left_travel * np.sin(self.theta) / 2 # ?
            self.y += left_travel * np.cos(self.theta) / 2 # ?
            return

        r = right_travel / dtheta + self.args.width / 2
        chord = r * np.sin(dtheta / 2)
        dx = chord * np.sin(self.theta)
        dy = chord * np.cos(self.theta)
        self.x += dx
        self.y += dy

    @torch.inference_mode()
    def update(self, left_volt, right_volt, dt=-1):
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

if __name__ == "__main__":
    R = Robot(
        0, 0, 0, RobotArgs(5.08, 98.4375)
    )

    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((800, 800))

    while True:
        left, right = 0, 0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        if pygame.key.get_pressed()[pygame.K_UP]:
            left = 12000
            right = 12000

        if pygame.key.get_pressed()[pygame.K_DOWN]:
            left = -12000
            right = -12000

        if pygame.key.get_pressed()[pygame.K_RIGHT]:
            left = 0
        
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            right = 0

        left = min(max(-12000, left), 12000)
        right = min(max(-12000, right), 12000)

        R.update(left, right)
        R.x = min(max(-400, R.x), 400)
        R.y = min(max(-400, R.y), 400)

        screen.fill((0, 0, 0))

        Rx, Ry = 400 + R.x, 400 + R.y
        print(Rx, Ry)
        pygame.draw.circle(screen, (255, 255, 255), (Rx, Ry), 10)
        pygame.draw.line(screen, (255, 255, 255), (Rx, Ry), (Rx + 20 * np.sin(R.theta), Ry + 20 * np.cos(R.theta)))

        surf = pygame.font.SysFont('Arial', 30).render(f"L: {left}, R: {right}", True, (255, 255, 255))
        screen.blit(surf, (0, 0))

        surf = pygame.font.SysFont('Arial', 30).render(f"Lv: {R.left_velo}, Rv: {R.right_velo}", True, (255, 255, 255))
        screen.blit(surf, (0, 90))

        pygame.display.update()
        time.sleep(0.02)