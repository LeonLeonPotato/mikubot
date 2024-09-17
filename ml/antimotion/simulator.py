import pygame
import numpy as np
import random
import time
import gymnasium

import pygame.ftfont
import pygame.gfxdraw
import quintic_spline as qs
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, width, x, y, theta, accel_factor=0.8):
        self.width = width
        self.x = x; self.velocity_x = 0; self.accel_x = 0
        self.y = y; self.velocity_y = 0; self.accel_y = 0
        self.theta = theta; self.angular_velo = 0; self.angular_accel = 0

        self.accel_factor = accel_factor
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
        self.left_accel = dleft * random.gauss(1, 0.02) * self.accel_factor
        self.right_accel = dright * random.gauss(1, 0.02) * self.accel_factor
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
    def __init__(self, robot_width=40, render_mode=None, 
                 wp_radius=5, robot_radius=30, pts=50, num_targets=5):
        super().__init__()
        self.observation_space = gymnasium.spaces.Box(low=0, high=1, shape=(10,), dtype=np.float32)
        self.action_space = gymnasium.spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.render_mode = render_mode

        if render_mode == 'human':
            pygame.init()
            pygame.font.init()
            self.font = pygame.font.SysFont('Arial', 20)
            self.screen = None
        elif render_mode is not None:
            print(f"Warning: Rendering mode {render_mode} is not supported")

        self.robot_width = robot_width
        self.wp_radius = wp_radius
        self.robot_radius = robot_radius
        self.pts = pts
        self.num_targets = num_targets

    def reset(self, seed=0):
        self.robot = Robot(self.robot_width, random.randint(0, 800), random.randint(0, 600), random.random() * 2 * np.pi)
        self.last_update = -1
        self.last_error = -1
        self.steps = 0
        self.total_rwd = 0
        self.targets = []
        self.Xpoly = None
        for i in range(self.num_targets):
            self.targets.append((random.randint(0, 800), random.randint(0, 600)))
        return self.step([0])[0], {"newtarg": True}
    
    def _compute_spline(self, speed):
        Xwaypoints = [self.robot.x] + [x for x, y in self.targets]
        Ywaypoints = [self.robot.y] + [y for x, y in self.targets]
        self.Xpoly = qs.compute_spline(Xwaypoints, np.sin(self.robot.theta) * speed, 0, 0, 0)
        self.Ypoly = qs.compute_spline(Ywaypoints, np.cos(self.robot.theta) * speed, 0, 0, 0)
        self.Xpoints = [qs.compute(self.Xpoly, i) for i in np.linspace(0, len(self.targets), len(self.targets) * self.pts)]
        self.Ypoints = [qs.compute(self.Ypoly, i) for i in np.linspace(0, len(self.targets), len(self.targets) * self.pts)]

    def _compute_intersection(self, guesses, iterations=10, tolerance=1e-2):
        X = self.Xpoly[0]
        Y = self.Ypoly[0]

        if (self.targets[0][0] - self.robot.x) ** 2 + (self.targets[0][1] - self.robot.y) ** 2 < self.robot_radius ** 2:
            return 1.0

        dXdt = X.deriv()
        dYdt = Y.deriv()

        for i in range(iterations):
            f_guesses = (X(guesses) - self.robot.x) ** 2 + (Y(guesses) - self.robot.y) ** 2 - self.robot_radius ** 2
            dfdt_guesses = 2*(X(guesses) - self.robot.x)*dXdt(guesses) + 2*(Y(guesses) - self.robot.y)*dYdt(guesses)
            guesses -= f_guesses / (dfdt_guesses + 1e-6)
            guesses = np.clip(guesses, 0, 1)

        guesses = np.sort(guesses)
        for i in range(len(guesses)-1, -1, -1):
            if abs((X(guesses[i]) - self.robot.x) ** 2 + (Y(guesses[i]) - self.robot.y) ** 2 - self.robot_radius ** 2) < tolerance:
                return np.clip(guesses[i], 0, 1)
            
        return None

    def step(self, action):
        action = action[0] * 200
        self.last_action = action
        self.steps += 1
        if self.render_mode == 'human':
            if self.last_update == -1:
                self.last_update = time.time()
                dt = 1 / 50
            else:
                dt = time.time() - self.last_update
                self.last_update = time.time()
        else:
            dt = 1 / 50

        speed = np.sqrt(self.robot.velocity_x ** 2 + self.robot.velocity_y ** 2)

        if self.Xpoly is None:
            self._compute_spline(speed)

        t_surrogate = self._compute_intersection(np.linspace(0, 1, 10))
        recomputed = False
        if t_surrogate is None:
            self._compute_spline(speed)
            recomputed = True
            t_surrogate = self._compute_intersection(np.linspace(0, 1, 10))

        assert t_surrogate is not None, "Robot is stuck"

        self.stx = self.Xpoly[0](t_surrogate)
        self.sty = self.Ypoly[0](t_surrogate)
        rtx, rty = self.targets[0]
        sdx = self.stx - self.robot.x
        sdy = self.sty - self.robot.y
        rdx = rtx - self.robot.x
        rdy = rty - self.robot.y
        
        norm_angle = self.robot.theta % (2 * np.pi)
        dangle_surrogate = (np.arctan2(sdx, sdy) - norm_angle + np.pi) % (2 * np.pi) - np.pi
        dangle_target = (np.arctan2(rdx, rdy) - norm_angle + np.pi) % (2 * np.pi) - np.pi

        self.robot.set_velo(100 - action, 100 + action)
        self.robot.update(dt)

        dist_surrogate = np.sqrt(sdx ** 2 + sdy ** 2)
        dist_target = np.sqrt((self.targets[0][0] - self.robot.x) ** 2 + (self.targets[0][1] - self.robot.y) ** 2)

        self.error = abs(dangle_surrogate)
        if self.last_error == -1:
            self.last_error = self.error
        rwd = self.last_error - self.error
        if recomputed:
            rwd -= 0.1
        if speed < 50:
            rwd -= 0.01

        self.last_error = self.error
        self.total_rwd += rwd

        s_dxdt = self.Xpoly[0].deriv()(t_surrogate)
        s_dydt = self.Ypoly[0].deriv()(t_surrogate)
        dangle_at_target = (np.arctan2(s_dxdt, s_dydt) - norm_angle + np.pi) % (2 * np.pi) - np.pi

        obs = []
        obs.append(dist_surrogate / 1000)
        obs.append(dist_target / 1000)
        obs.append(dangle_surrogate / np.pi)
        obs.append(dangle_target / np.pi)
        obs.append(self.robot.angular_velo / 100)
        obs.append(self.robot.angular_accel / 100)
        obs.append(dangle_at_target / np.pi)
        obs.append((self.robot.theta / (2 * np.pi)) % 1)
        obs.append(speed / 100)
        obs.append(rwd)
        obs = np.array(obs, dtype=np.float32)

        done = self.steps > 5000
        info = {'newtarg': False}
        
        if np.sqrt((self.targets[0][0] - self.robot.x) ** 2 + (self.targets[0][1] - self.robot.y) ** 2) < self.wp_radius:
            self.targets.pop(0)
            info['newtarg'] = True
            # done = not self.render_mode
            done = len(self.targets) == 0
            if not done:
                self._compute_spline(np.sqrt(s_dxdt ** 2 + s_dydt ** 2))
        
        if done:
            info['episode'] = {'r': self.total_rwd, 'l': self.steps}

        return obs, rwd, done, False, info

    def render(self):
        if self.render_mode is None:
            return

        if self.screen == None:
            self.screen = pygame.display.set_mode((800, 600))

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