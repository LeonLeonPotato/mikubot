import gymnasium as gym
import numpy as np
import torch as th
import matplotlib as plt

class PathFollowingEnv(gym.Env):
    def __init__(self):
        self.robot = Robot(0, 0, 0, RobotArgs(0.1, 0.1))
        self.target = (1, 1)
        self.dt = 0.1
        self.max_steps = 100
        self.steps = 0

    def reset(self):
        self.robot = Robot(0, 0, 0, RobotArgs(0.1, 0.1))
        self.target = (1, 1)
        self.steps = 0
        return self.robot.x, self.robot.y, self.robot.theta

    def step(self, action):
        left_velo, right_velo = action
        self.robot._dummy_update(left_velo, right_velo, self.dt)
        self.steps += 1
        done = self.steps >= self.max_steps
        reward = -self.robot.angle_to(*self.target)
        return self.robot.x, self.robot.y, self.robot.theta, reward, done

    def render(self):
        plt.plot(self.robot.x, self.robot.y, 'ro')
        plt.plot(*self.target, 'bo')
        plt.show()