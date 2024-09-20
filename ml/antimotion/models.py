import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import gymnasium as gym

class Actor(nn.Module):
    def __init__(self, env):
        super().__init__()
        if isinstance(env, gym.vector.SyncVectorEnv):
            self.observation_space = env.single_observation_space
            self.action_space = env.single_action_space
        else:
            self.observation_space = env.observation_space
            self.action_space = env.action_space

        self.fc1 = nn.Linear(self.observation_space.shape[0], 32)
        self.fc2 = nn.Linear(32, 8)
        self.fc3 = nn.Linear(8, self.action_space.shape[0])

        self.register_buffer(
            "action_scale", torch.tensor((self.action_space.high - self.action_space.low) / 2)
        )

        self.register_buffer(
            "action_bias", torch.tensor((self.action_space.high + self.action_space.low) / 2)
        )

    def forward(self, x):
        x = F.leaky_relu(self.fc1(x))
        x = F.leaky_relu(self.fc2(x))
        x = torch.tanh(self.fc3(x)) * 100
        return x * self.action_scale + self.action_bias
    
class Critic(nn.Module):

    def __init__(self, env):
        super().__init__()
        if isinstance(env, gym.vector.SyncVectorEnv):
            self.observation_space = env.single_observation_space
            self.action_space = env.single_action_space
        else:
            self.observation_space = env.observation_space
            self.action_space = env.action_space
        n = self.observation_space.shape[0] + self.action_space.shape[0]
        self.fc1 = nn.Linear(n, 256)
        self.fc2 = nn.Linear(256, 32)
        self.fc3 = nn.Linear(32, 1)

    def forward(self, x, a):
        x = torch.cat([x, a], 1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x