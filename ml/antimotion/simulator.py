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
    robot_width = 40
    wp_radius = 15
    robot_radius = 35
    pts = 20
    num_targets = 5
    dt = [1/20, 1/40]

class Robot:
    def __init__(self, width, x, y, theta, accel_factor=0.8, accel_noise=0.02):
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

class RobotEnvironment(gymnasium.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.observation_space = gymnasium.spaces.Box(low=0, high=1, shape=(27,), dtype=np.float32)
        self.action_space = gymnasium.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.render_mode = render_mode

        if render_mode == 'human':
            pygame.init()
            pygame.font.init()
            self.font = pygame.font.SysFont('Arial', 20)
            self.screen = pygame.display.set_mode((800, 600))
        elif render_mode is not None:
            print(f"Warning: Rendering mode {render_mode} is not supported")

    def _generate_obs(self):
        obs = np.zeros(9)
        obs[0] = self.robot.x / Args.screen_space[0]
        obs[1] = self.robot.y / Args.screen_space[1]
        obs[2] = self.robot.velocity_x / 100
        obs[3] = self.robot.velocity_y / 100
        obs[4] = self.robot.accel_x / 100
        obs[5] = self.robot.accel_y / 100
        obs[6] = self.robot.theta / (2 * np.pi)
        obs[7] = self.robot.angular_velo / 100
        obs[8] = self.robot.angular_accel / 100
        return obs

    def _intersection(self, guess, iterations=7, threshold=1e-1):
        for i in range(iterations):
            f_guesses = self.spline(guess) - Args.robot_radius
            dfdt_guesses = self.spline.derivative(guess)
            guesses -= f_guesses / (dfdt_guesses + 1e-6)
            guesses = np.clip(guesses, 0, len(self.spline))

        guesses = np.sort(guesses)

        for i in range(len(guesses)-1, -1, -1):
            if self.spline(guess) - self.robot_radius < threshold:
                return guesses[i]
            
        return None

    def reset(self, seed=0):
        rx = np.random.random() * (Args.robot_spawn_x[1] - Args.robot_spawn_x[0]) + Args.robot_spawn_x[0]
        ry = np.random.random() * (Args.robot_spawn_y[1] - Args.robot_spawn_y[0]) + Args.robot_spawn_y[0]
        rt = np.random.random() * (Args.robot_spawn_angle[1] - Args.robot_spawn_angle[0]) + Args.robot_spawn_angle[0]
        self.robot = Robot(Args.robot_width, rx, ry, rt)
        self.targets = [
            np.random.random(2) * Args.screen_space
            for _ in range(Args.num_targets)
        ]

        self.spline = qs.QuinticSpline([[self.robot.x, self.robot.y]] + self.targets)
        self.spline.solve_coeffs(np.cos(self.robot.theta), np.sin(self.robot.theta), 0, 0)
        self.points = self.spline(np.linspace(0, len(self.spline), self.pts * len(self.spline)))

        self.intersections = [self._intersection(np.array([0, 0.1, 0.2])), 0, 0]
        self.last_obs = [self._generate_obs(), np.zeros(9), np.zeros(9)]

        return self.last_obs, {}
    
    def step(self, action):
        obs = np.concatenate(self.last_obs)
        return obs, 0, False, False, {}

    def render(self):
        if self.render_mode is None:
            return

        self.screen.fill((0, 0, 0))

        for p in self.targets:
            pygame.draw.circle(self.screen, (255, 0, 0), p, self.wp_radius, 1)
        
        for i in range(len(self.Xpoints) - 1):
            pygame.draw.line(self.screen, (150, 150, 150), 
                             (self.Xpoints[i], self.Ypoints[i]), 
                             (self.Xpoints[i + 1], self.Ypoints[i + 1])
                             )
    
        self.robot.draw(screen=self.screen)

        renderstr = ", ".join([
            f"Left: {self.robot.left_velo:.2f}",
            f"Right: {self.robot.right_velo:.2f}",
            f"Action: {self.last_action:.2f}",
            f"Speed: {np.sqrt(self.robot.velocity_x ** 2 + self.robot.velocity_y ** 2):.2f}"
        ])
        surf = self.font.render(renderstr, True, (255, 255, 255))
        self.screen.blit(surf, (10, 10))

        pygame.draw.circle(self.screen, (0, 0, 255), (self.stx, self.sty), 5)
        pygame.draw.circle(self.screen, (255, 0, 0), (self.robot.x, self.robot.y), self.robot_radius, 1)

        pygame.display.flip()

if __name__ == "__main__":
    env = RobotEnvironment(render_mode='human')
    obs, info = env.reset()

    dangles = []
    while True:
        if env.screen is not None:
            env.screen.fill((0, 0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                break
            if event.type == pygame.MOUSEBUTTONDOWN:
                env.target.append(event.pos)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    env.robot.set_velo(0, 0)
                if event.key == pygame.K_BACKSPACE:
                    env.target.clear()
                if event.key == pygame.K_r:
                    plt.plot(dangles)
                    plt.show()
                    dangles = []

        obs, rwd, done, truncated, info = env.step([0.5])
        dangles.append(rwd)

        env.render()