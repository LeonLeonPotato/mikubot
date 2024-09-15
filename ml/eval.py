from simulator import *
import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, env):
        super().__init__()
        self.fc1 = nn.Linear(np.array(env.observation_space.shape).prod(), 32)
        self.fc2 = nn.Linear(32, 16)
        self.fc_mu = nn.Linear(16, np.prod(env.action_space.shape))
        # action rescaling
        self.register_buffer(
            "action_scale", torch.tensor((env.action_space.high - env.action_space.low) / 2.0, dtype=torch.float32).unsqueeze(0)
        )
        self.register_buffer(
            "action_bias", torch.tensor((env.action_space.high + env.action_space.low) / 2.0, dtype=torch.float32).unsqueeze(0)
        )

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = torch.tanh(self.fc_mu(x))
        return x * self.action_scale + self.action_bias

model = Actor(RobotEnvironment())
model.load_state_dict(torch.load("done3.pt")[0])
model.eval()

env = RobotEnvironment(render_mode='human')
obs, info = env.reset()
done = False

while True:
    with torch.no_grad():
        action = model(torch.tensor(obs, dtype=torch.float32)).numpy()[0]
    obs, _, done, _, info = env.step(action)
    env.render()
    pygame.time.wait(20)