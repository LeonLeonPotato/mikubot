import pygame
import numpy as np
import random
import time
import gymnasium

import pygame.ftfont
import pygame.gfxdraw
import quintic_spline as qs
import matplotlib.pyplot as plt

class Args:
    screen_space = [500, 500]
    robot_spawn_x = [25, 475]
    robot_spawn_y = [25, 475]
    robot_spawn_angle = [-np.pi, np.pi]
    robot_width = 50
    wp_radius = 15
    robot_radius = 35
    pts = 20
    num_targets = 5
    dt = [1/20, 1/40]

    def getrand(x):
        return np.random.random() * (x[1] - x[0]) + x[0]

class Robot:
    def __init__(self, width, x, y, theta, accel_factor=10, accel_noise=0.2):
        self.width = width
        self.x = x; self.velocity_x = 0; self.accel_x = 0
        self.y = y; self.velocity_y = 0; self.accel_y = 0
        self.theta = theta; self.angular_velo = 0; self.angular_accel = 0

        self.accel_factor = accel_factor
        self.accel_noise = accel_noise
        self.set_left_velo = 0
        self.set_right_velo = 0
        self.left_velo = 0; self.left_accel = 0
        self.right_velo = 0; self.right_accel = 0
        self._prev_angular_velo = 0

    def set_velo(self, left, right):
        right = max(min(right, 100), -100)
        left = max(min(left, 100), -100)
        self.set_left_velo = left
        self.set_right_velo = right

    def update(self, dt):
        dleft = self.set_left_velo - self.left_velo
        dright = self.set_right_velo - self.right_velo
        self.left_accel = dleft * random.gauss(1, self.accel_noise) * self.accel_factor
        self.right_accel = dright * random.gauss(1, self.accel_noise) * self.accel_factor
        self.left_velo += self.left_accel * dt
        self.right_velo += self.right_accel * dt

        if abs(self.left_velo - self.right_velo) < 1e-3:
            self.velocity_x = self.left_velo * np.sin(self.theta)
            self.velocity_y = self.left_velo * np.cos(self.theta)
            self.accel_x = self.left_accel * np.sin(self.theta)
            self.accel_y = self.left_accel * np.cos(self.theta)
            self.x += self.velocity_x * dt
            self.y += self.velocity_y * dt
            return
        
        r = self.width * (self.left_velo + self.right_velo) / (2 * (self.right_velo - self.left_velo))
        self.angular_velo = (self.left_velo - self.right_velo) / (self.width)
        if self._prev_angular_velo == 0:
            self._prev_angular_velo = self.angular_velo
        self.angular_accel = (self.angular_velo - self._prev_angular_velo) / dt
        self._prev_angular_velo = self.angular_velo

        self.theta += self.angular_velo * dt
        chord = 2 * r * np.sin(-self.angular_velo * dt / 2)
        dx = chord * np.sin(self.theta)
        dy = chord * np.cos(self.theta)
        self.x += dx
        self.y += dy
        prev_x_vel = self.velocity_x
        prev_y_vel = self.velocity_y
        self.velocity_x = dx / dt
        self.velocity_y = dy / dt
        self.accel_x = (dx / dt - prev_x_vel) / dt
        self.accel_y = (dy / dt - prev_y_vel) / dt

    def angle_to(self, x, y):
        norm_angle = self.theta % (2 * np.pi)
        dx = x - self.x
        dy = y - self.y
        return (np.arctan2(dx, dy) - norm_angle + np.pi) % (2 * np.pi) - np.pi

    def dist_to(self, x, y):
        return np.sqrt((self.x - x)**2 + (self.y - y)**2)

    def pos(self):
        return np.array([self.x, self.y])

    def speed(self):
        return np.sqrt(self.velocity_x ** 2 + self.velocity_y ** 2)

    def draw(self, screen):
        rotation_matrix = np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])
        rect = np.array([
            (-self.width/2, 5),
            (self.width/2, 5),
            (self.width/2, -5),
            (-self.width/2, -5)
        ]) @ rotation_matrix

        pygame.draw.polygon(screen, (0, 255, 0), rect + (self.x, self.y))
        pygame.draw.line(screen, (255, 0, 0), (self.x, self.y), (self.x + 20 * np.sin(self.theta), self.y + 20 * np.cos(self.theta)))

class PurePursuitEnv(gymnasium.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.observation_space = gymnasium.spaces.Box(low=0, high=1, shape=(24,), dtype=np.float32)
        self.action_space = gymnasium.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.render_mode = render_mode

        if render_mode == 'human':
            pygame.init()
            pygame.font.init()
            self.font = pygame.font.SysFont('Arial', 20)
            self.screen = pygame.display.set_mode(Args.screen_space)
        elif render_mode is not None:
            print(f"Warning: Rendering mode {render_mode} is not supported")

    def _generate_obs(self):
        obs = np.zeros(8)
        obs[0] = self.stx / Args.screen_space[0]
        obs[1] = self.sty / Args.screen_space[1]
        obs[2] = self.robot.x / Args.screen_space[0]
        obs[3] = self.robot.y / Args.screen_space[1]
        obs[4] = self.robot.velocity_x / 100
        obs[5] = self.robot.velocity_y / 100
        obs[6] = self.robot.theta / (2 * np.pi)
        obs[7] = self.robot.angular_velo / 100
        return obs

    def _intersection(self, guess, iterations=10, threshold=1e-1, bounds=None):
        if bounds is None:
            bounds = [0, len(self.spline)]

        for i in range(iterations):
            f_guess = self.spline(guess) - self.robot.pos()
            numerator = np.sum(f_guess * f_guess, axis=1) - Args.robot_radius ** 2
            denom = 2 * np.sum(f_guess * self.spline.derivative(guess), axis=1)
            guess -= numerator / (denom + 1e-6)
            guess = np.clip(guess, bounds[0], bounds[1])

        guess = np.sort(guess)
        f_guess = np.linalg.norm(self.spline(guess) - self.robot.pos(), axis=1) - Args.robot_radius

        for i in range(len(guess)-1, -1, -1):
            if f_guess[i] < threshold:
                return guess[i]
            
        return None

    def reset(self, seed=0):
        self.robot = Robot(Args.robot_width, 
                           Args.getrand(Args.robot_spawn_x), 
                           Args.getrand(Args.robot_spawn_y), 
                           Args.getrand(Args.robot_spawn_angle))
        self.targets = [
            np.random.random(2) * Args.screen_space
            for _ in range(Args.num_targets)
        ]

        self.spline = qs.QuinticSpline([[self.robot.x, self.robot.y]] + self.targets)
        self.spline.solve_coeffs(np.cos(self.robot.theta), np.sin(self.robot.theta), 0, 0)
        self.points = self.spline(np.linspace(0, len(self.spline), Args.pts * len(self.spline)))

        self.steps = 0
        self.total_rwd = 0
        self.last_rwd = np.zeros(3)

        self.last_intersection = self._intersection(np.linspace(0.1, len(self.spline), 10))
        self.stx, self.sty = self.spline(self.last_intersection)
        self.last_obs = [self._generate_obs(), np.zeros(9), np.zeros(9)]
        obs = np.concatenate(self.last_obs)

        return obs, {}
    
    def step(self, action):
        dt = Args.getrand(Args.dt)
        self.robot.set_velo(action[0], action[1])
        self.robot.update(dt)

        self.last_intersection = self._intersection(
            np.array([self.last_intersection]), iterations=2,
            bounds=[self.last_intersection, len(self.spline)]
        )
        if self.last_intersection is None:
            self.spline = qs.QuinticSpline([[self.robot.x, self.robot.y]] + self.targets)
            self.spline.solve_coeffs(np.cos(self.robot.theta), np.sin(self.robot.theta), 0, 0)
            self.points = self.spline(np.linspace(0, len(self.spline), Args.pts * len(self.spline)))
            self.last_intersection = self._intersection(np.linspace(0.1, len(self.spline), 15))

        self.stx, self.sty = self.spline(self.last_intersection)
        
        self.last_obs = [self._generate_obs(), self.last_obs[0], self.last_obs[1]]
        obs = np.concatenate(self.last_obs)
        return obs, 0, False, False, {"dt": dt}

    def render(self):
        if self.render_mode is None:
            return

        self.screen.fill((0, 0, 0))

        for p in self.targets:
            pygame.draw.circle(self.screen, (255, 0, 0), p, Args.wp_radius, 1)
        for p in self.points:
            pygame.draw.circle(self.screen, (0, 150, 0), p, 1)
    
        self.robot.draw(screen=self.screen)

        renderstr = ", ".join([
            f"Left: {self.robot.left_velo:.2f}",
            f"Right: {self.robot.right_velo:.2f}",
            f"Speed: {np.sqrt(self.robot.velocity_x ** 2 + self.robot.velocity_y ** 2):.2f}"
        ])
        surf = self.font.render(renderstr, True, (255, 255, 255))
        self.screen.blit(surf, (10, 10))

        stx, sty = self.spline(self.last_intersection)
        pygame.draw.circle(self.screen, (0, 0, 255), (stx, sty), 5)
        pygame.draw.circle(self.screen, (255, 0, 0), (self.robot.x, self.robot.y), Args.robot_radius, 1)

        pygame.display.flip()

class AnySplineEnv(gymnasium.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.observation_space = gymnasium.spaces.Box(low=0, high=1, shape=(24,), dtype=np.float32)
        self.action_space = gymnasium.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.render_mode = render_mode

        if render_mode == 'human':
            pygame.init()
            pygame.font.init()
            self.font = pygame.font.SysFont('Arial', 20)
            self.screen = pygame.display.set_mode(Args.screen_space)
        elif render_mode is not None:
            print(f"Warning: Rendering mode {render_mode} is not supported")

    def _generate_obs(self):
        obs = np.zeros(8)
        obs[0] = self.stx / Args.screen_space[0]
        obs[1] = self.sty / Args.screen_space[1]
        obs[2] = self.robot.x / Args.screen_space[0]
        obs[3] = self.robot.y / Args.screen_space[1]
        obs[4] = self.robot.velocity_x / 100
        obs[5] = self.robot.velocity_y / 100
        obs[6] = self.robot.theta / (2 * np.pi)
        obs[7] = self.robot.angular_velo / 100
        return obs

    def _closest(self, guess, iterations=10, threshold=1e-1, bounds=None):
        if bounds is None:
            bounds = [0, len(self.spline)]

        for i in range(iterations):
            f_guess = self.spline(guess) - self.robot.pos()
            numerator = f_guess ** 2
            denom = 2 * np.sum(f_guess * self.spline.derivative(guess), axis=1)
            guess -= numerator / (denom + 1e-6)
            guess = np.clip(guess, bounds[0], bounds[1])

        dist = np.linalg.norm(self.spline(guess) - self.robot.pos(), axis=1)
        return guess, dist

    def reset(self, seed=0):
        self.robot = Robot(Args.robot_width, 
                           Args.getrand(Args.robot_spawn_x), 
                           Args.getrand(Args.robot_spawn_y), 
                           Args.getrand(Args.robot_spawn_angle))
        self.targets = [
            np.random.random(2) * Args.screen_space
            for _ in range(Args.num_targets)
        ]

        self.spline = qs.QuinticSpline([[self.robot.x, self.robot.y]] + self.targets)
        self.spline.solve_coeffs(np.cos(self.robot.theta), np.sin(self.robot.theta), 0, 0)
        self.points = self.spline(np.linspace(0, len(self.spline), Args.pts * len(self.spline)))

        self.steps = 0
        self.total_rwd = 0
        self.last_rwd = np.zeros(3)

        self.last_intersection = self._intersection(np.linspace(0.1, len(self.spline), 10))
        self.stx, self.sty = self.spline(self.last_intersection)
        self.last_obs = [self._generate_obs(), np.zeros(9), np.zeros(9)]
        obs = np.concatenate(self.last_obs)

        return obs, {}
    
    def step(self, action):
        dt = Args.getrand(Args.dt)
        self.robot.set_velo(action[0], action[1])
        self.robot.update(dt)

        self.last_intersection = self._intersection(
            np.array([self.last_intersection]), iterations=2,
            bounds=[self.last_intersection, len(self.spline)]
        )
        if self.last_intersection is None:
            self.spline = qs.QuinticSpline([[self.robot.x, self.robot.y]] + self.targets)
            self.spline.solve_coeffs(np.cos(self.robot.theta), np.sin(self.robot.theta), 0, 0)
            self.points = self.spline(np.linspace(0, len(self.spline), Args.pts * len(self.spline)))
            self.last_intersection = self._intersection(np.linspace(0.1, len(self.spline), 15))

        self.stx, self.sty = self.spline(self.last_intersection)
        
        self.last_obs = [self._generate_obs(), self.last_obs[0], self.last_obs[1]]
        obs = np.concatenate(self.last_obs)
        return obs, 0, False, False, {"dt": dt}

    def render(self):
        if self.render_mode is None:
            return

        self.screen.fill((0, 0, 0))

        for p in self.targets:
            pygame.draw.circle(self.screen, (255, 0, 0), p, Args.wp_radius, 1)
        for p in self.points:
            pygame.draw.circle(self.screen, (0, 150, 0), p, 1)
    
        self.robot.draw(screen=self.screen)

        renderstr = ", ".join([
            f"Left: {self.robot.left_velo:.2f}",
            f"Right: {self.robot.right_velo:.2f}",
            f"Speed: {np.sqrt(self.robot.velocity_x ** 2 + self.robot.velocity_y ** 2):.2f}"
        ])
        surf = self.font.render(renderstr, True, (255, 255, 255))
        self.screen.blit(surf, (10, 10))

        stx, sty = self.spline(self.last_intersection)
        pygame.draw.circle(self.screen, (0, 0, 255), (stx, sty), 5)
        pygame.draw.circle(self.screen, (255, 0, 0), (self.robot.x, self.robot.y), Args.robot_radius, 1)

        pygame.display.flip()

if __name__ == "__main__":
    # 17668.1092365258, 13.0, 0.2, 0.9333333333333333
    # 8527.547100731887, 10.0, 0.225, 1.0
    # 9369.687273226324, 10.0, 0.175, 0.8
    env = RobotEnvironment(render_mode='human')
    obs, info = env.reset()

    while True:
        if pygame.event.get(pygame.QUIT):
            break

        stx, sty = obs[:2] * Args.screen_space
        x, y = obs[2:4] * Args.screen_space
        angle = env.robot.angle_to(stx, sty)
        dist = env.robot.dist_to(stx, sty)
        dist = min(dist ** 2, 100)
        angle = (angle * (210 / np.pi))
        left = dist + angle
        right = dist - angle
        obs, rwd, done, truncated, info = env.step([left, right])
        env.render()
        pygame.time.wait(int(info['dt'] * 1000))
        if done:
            break