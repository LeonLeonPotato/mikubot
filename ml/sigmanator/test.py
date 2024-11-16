import filters
import time
import pygame
import numpy as np
from collections import namedtuple

distrib = namedtuple("Distrib", ["mean", "std", "decay_factor", "tick"])

pygame.init()
screen = pygame.display.set_mode((800, 600))

distribs = []

working = True
ticks = 0
start_time = time.time()
errors = []
while working and time.time() - start_time < 5:
    screen.fill((0, 0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            working = False

    ticks += 1
    true_val = np.sin(time.time()) * 5 + 12.5
    observed_val = true_val + np.random.normal(scale=1)
    distribs.append(distrib(
        observed_val,
        1,
        0,
        ticks
    ))

    while len(distribs) > 50:
        distribs.pop(0)

    X = np.linspace(0, 25, 800)
    Y = np.zeros_like(X)
    for d in distribs:
        Y += 1 / np.sqrt(2 * np.pi * (d.std ** 2 + (ticks - d.tick) * d.decay_factor)) * np.exp(-0.5 * (X - d.mean) ** 2 / (d.std ** 2 + (ticks - d.tick) * d.decay_factor))

    X = X / 25 * 800
    Y = Y * 10
    argmax = np.argmax(Y)
    min_y = Y[argmax]
    min_x = X[argmax]
    
    for i in range(1, len(X)):
        pygame.draw.line(screen, (255, 255, 255), 
                         (X[i - 1], Y[i - 1]), 
                         (X[i], Y[i]), 
                         1)
    
    if "true_val" in locals():
        pygame.draw.line(screen, (255, 0, 0), (true_val / 25 * 800, 0), (true_val / 25 * 800, 600), 1)
        pygame.draw.line(screen, (0, 255, 0), (observed_val / 25 * 800, 0), (observed_val / 25 * 800, 600), 1)
        errors.append(abs(true_val - min_x / 800 * 25))

    pygame.draw.circle(screen, (255, 255, 255), (min_x, min_y), 5)

    pygame.display.flip()

print("Samples per second:", ticks)
print("Average error:", np.mean(errors))
print("STD:", np.std(errors))