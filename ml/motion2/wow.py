import environment
import models
import robot_sim
import torch
import numpy as np
import pygame

env = environment.PathFollowingEnv(environment.Args(10, 0.02, 1000))
model = models.RLPolicyModel(env).cpu()
model = model.eval()
model.load_state_dict(torch.load("ml/motion2/ddpg.cleanrl_model", map_location='cpu')[0])

pygame.init()
pygame.font.init()

screen = pygame.display.set_mode((800, 800))

obs, _ = env.reset()
for i in range(1000):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    action = model(torch.tensor(obs, dtype=torch.float32)).detach().numpy()[0]
    obs, reward, done, trunc, _ = env.step(action)
    if done:
        break

    screen.fill((0, 0, 0))
    tx, ty = env.target_locs[-1]
    pygame.draw.circle(screen, (255, 0, 0), (int(env.robot.x) + 400, int(env.robot.y) + 400), 10)
    pygame.draw.line(screen, (255, 0, 0), (int(env.robot.x) + 400, int(env.robot.y) + 400), (int(env.robot.x + 20 * np.sin(env.robot.theta)) + 400, int(env.robot.y + 20 * np.cos(env.robot.theta)) + 400))
    pygame.draw.circle(screen, (0, 255, 0), (int(tx) + 400, int(ty) + 400), 10)
    pygame.display.flip()
    pygame.time.delay(20)