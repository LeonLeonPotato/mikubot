import subprocess
import json
import pygame
import numpy as np
import time
import sys

prospath = {
    "macos": "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
    "win32": "C:/Users/leon/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros"
}

proc = subprocess.Popen([
    prospath[sys.platform],
    "terminal"
], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

pygame.init()
screen = pygame.display.set_mode((800, 800))
x, y, conf = -1, -1, 0

while True:
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, 800, 800))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    output = proc.stdout.readline().decode("utf-8").strip()
    if output.startswith("{"):
        data = json.loads(output)
        x, y = np.sin(data["angle"]) * data['distance'], np.cos(data["angle"]) * data['distance']
        conf = data['confidence']

    if conf > 10:
        surf = pygame.Surface((20, 20), pygame.SRCALPHA)
        pygame.draw.circle(surf, (255, 255, 255, 255), (10, 10), 3)

        screen.blit(surf, (x, 800 - y))

    pygame.display.flip()
    pygame.time.wait(10)
