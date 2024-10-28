import subprocess
import json
import pygame
import numpy as np
import time
from collections import namedtuple

circle = namedtuple("circle", ["angle", "x", "y", "entrytime"])

process = subprocess.Popen([
    "/home/leon/.config/Code/User/globalStorage/sigbots.pros/install/pros-cli-linux/pros", 
    "terminal"
], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

pygame.init()
screen = pygame.display.set_mode((800, 800))
circles = []

def lerp_alpha(circle, fade_time = 1):
    return 255 - (time.time() - circle.entrytime) / fade_time * 255

while True:
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, 800, 800))

    pygame.draw.circle(screen, (255, 255, 255), (400, 400), 400, 1)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    output = process.stdout.readline().decode("utf-8").strip()
    if output.startswith("{"):
        data = json.loads(output)
        x, y = np.sin(data["angle"]) * data['distance'], np.cos(data["angle"]) * data['distance']
        circles.append(circle(data["angle"], x, y, time.time()))

    i = 0
    while i < len(circles):
        c = circles[i]
        alpha = lerp_alpha(c)
        if alpha <= 0:
            circles.pop(i)
            continue
        else:
            i += 1

        surf = pygame.Surface((20, 20), pygame.SRCALPHA)
        pygame.draw.circle(surf, (255, 255, 255, int(alpha)), (10, 10), 3)

        screen.blit(surf, (c.x + 400, c.y + 400))

    if len(circles) > 0:
        maxangle = max([c.angle for c in circles])
        pygame.draw.line(screen, (255, 0, 0), (400, 400), (np.sin(maxangle) * 400 + 400, np.cos(maxangle) * 400 + 400))

    pygame.display.flip()
    pygame.time.wait(10)
