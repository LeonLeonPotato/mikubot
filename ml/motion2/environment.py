import gymnasium as gym
import numpy as np
import torch as th
import matplotlib as plt
from robot_sim import Robot, RobotArgs
from dataset import RobotDataset
import quintic_spline as qs

class Args:
    def __init__(self, robot_radius,
                 dt, max_steps):
        self.robot_radius = robot_radius
        self.dt = dt
        self.max_steps = max_steps

class PathFollowingEnv(gym.Env):
    def __init__(self, args):
        self.action_space = gym.spaces.Box(-1, 1, (2,))
        self.observation_space = gym.spaces.Box(-1, 1, (9,))

        self.args = args
        self.robot = Robot(0, 0, 0, Robot.DEFAULT_ARGS)

        self.target_locs = [
            np.array((0, 0), dtype=float) for _ in range(9)
        ] + [np.random.rand(2) * 800 - 400]
        self.target_theta = 0
        self.target_theta_velo = 0
        self.target_theta_acc = 0
        self.steps = 0
        self.last_score = self.compute_score()
        self.total_reward = 0

    def compute_score(self):
        a = self.robot.dist(*self.target_locs[-1])
        c = abs(self.robot.angle_to(*self.target_locs[-1]))
        return -(a + c)
    
    def make_obs(self, action):
        return np.array([
            (self.target_locs[-3][0] - self.robot.x) / 400,
            (self.target_locs[-3][1] - self.robot.y) / 400,
            (self.target_locs[-2][0] - self.robot.x) / 400,
            (self.target_locs[-2][1] - self.robot.y) / 400,
            (self.target_locs[-1][0] - self.robot.x) / 400,
            (self.target_locs[-1][1] - self.robot.y) / 400,
            self.robot.angle_to(*self.target_locs[-1]) / np.pi,
            action[0], action[1]
        ])

    def reset(self, seed=0):
        self.robot = Robot(0, 0, 0, Robot.DEFAULT_ARGS)
        self.target = np.random.rand(2) * 800 - 400
        self.steps = 0
        self.last_score = self.compute_score()
        self.total_reward = 0

        obs = self.make_obs([0, 0])

        return obs, {}

    def step(self, action):
        left_velo, right_velo = action * 24000

        dt = RobotDataset.TRANSFORMER(self.args.dt * 1e6, 'dt')
        self.robot.update(12000 + left_velo, 12000 + right_velo, dt)

        self.steps += 1
        done = self.steps >= self.args.max_steps
        done = done or self.robot.dist(*self.target_locs[-1]) < self.args.robot_radius

        cur_score = self.compute_score()
        reward = cur_score - self.last_score - 0.5
        self.total_reward += reward
        self.last_score = cur_score

        self.target_theta_acc = np.random.random() * 0.02 - 0.01
        self.target_theta_velo += self.target_theta_acc
        self.target_theta += self.target_theta_velo

        ntx = self.target_locs[-1][0] + np.sin(self.target_theta) * 2
        nty = self.target_locs[-1][1] + np.cos(self.target_theta) * 2
        ntx = np.clip(ntx, -400, 400)
        nty = np.clip(nty, -400, 400)
        self.target_locs.append(np.array((ntx, nty), dtype=float))
        if len(self.target_locs) > 10:
            self.target_locs.pop(0)

        obs = self.make_obs(action)

        info = {
            "episode": {
                'r': self.total_reward,
                'l': self.steps
            },
            "total_rwd": self.total_reward
        }

        return obs, reward, done, False, info
    

if __name__ == "__main__":
    env = PathFollowingEnv(Args(10, 0.02, 1000))

    import pygame

    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((800, 800))

    done = False
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        action = env.robot.angle_to(*env.target_locs[-1])
        print(env.robot.left_velo, env.robot.right_velo)
        obs, reward, done, trunc, info = env.step(np.array([action, -action]) * 0.5)

        screen.fill((0, 0, 0))
        tx, ty = env.target_locs[-1]
        pygame.draw.circle(screen, (255, 0, 0), (int(env.robot.x) + 400, int(env.robot.y) + 400), 10)
        pygame.draw.line(screen, (255, 0, 0), (int(env.robot.x) + 400, int(env.robot.y) + 400), (int(env.robot.x + 20 * np.sin(env.robot.theta)) + 400, int(env.robot.y + 20 * np.cos(env.robot.theta)) + 400))
        pygame.draw.circle(screen, (0, 255, 0), (int(tx) + 400, int(ty) + 400), 10)
        pygame.display.flip()
        pygame.time.delay(20)