from simulator import *
import torch
import torch.nn as nn
import torch.nn.functional as F
from models import Actor
import os

maxt, name = 0, ""
for file in os.listdir("runs"):
    t = int(file.split("_")[-1])
    if t > maxt:
        maxt = t
        name = file

model = Actor(RobotEnvironment(num_targets=10))
model.load_state_dict(torch.load("runs/" + name + "/trainer.pt")[0])
model.eval()
model = model.cuda()

env = RobotEnvironment(render_mode='human', num_targets=5)
obs, info = env.reset()
done = False

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()

    with torch.no_grad():
        action = model(torch.tensor(obs, dtype=torch.float32, device='cuda')).cpu().numpy()[0]
    obs, _, done, _, info = env.step([action])
    env.render()
    pygame.time.wait(20)