import pandas as pd
import bisect
import pygame
import time
import numpy as np

p = pd.read_csv('./ml/motionprofile/logs/log32.txt')
p['time'] = p['time'] - p['time'][0]
p['velocity'] = np.sqrt(p['x'].diff() ** 2 + p['y'].diff() ** 2) / (p['time'] / 1e6).diff()

pygame.init()
pygame.font.init()
arial = pygame.font.SysFont('Arial', 20)

screen = pygame.display.set_mode((800, 600))
buffer = pygame.Surface((800, 600))

robot_base = pygame.Surface((40, 50), pygame.SRCALPHA)
pygame.draw.rect(robot_base, (150, 150, 150), (0, 0, 40, 50))
pygame.draw.rect(robot_base, (0, 0, 0), (19, 40, 2, 10))

def draw_axes():
    pygame.draw.line(buffer, (255, 255, 255), (400, 0), (400, 600))
    pygame.draw.line(buffer, (255, 255, 255), (0, 300), (800, 300))

def get_transformed_robot(theta):
    return pygame.transform.rotate(robot_base, theta * 180 / np.pi)

def draw_robot(x, y, theta):
    robot = get_transformed_robot(theta)
    robot_rect = robot.get_rect(center=(x + 400, y + 300))
    buffer.blit(robot, robot_rect)

running = True
start_t = time.time()

paused = False
pause_start_t = -1
last_i = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
                if paused:
                    pause_start_t = time.time()
                else:
                    start_t += time.time() - pause_start_t
            if event.key == pygame.K_LEFT:
                start_t += 1
            if event.key == pygame.K_RIGHT:
                start_t -= 1

    buffer.fill((0, 0, 0))
    draw_axes()

    if paused:
        pygame.display.flip()
        continue

    i = bisect.bisect_left(p['time'], (time.time() - start_t) * 1e6)
    if (i > len(p['time']) - 1):
        break

    draw_robot(p['x'][i], p['y'][i], p['theta'][i])

    pygame.draw.circle(buffer, (255, 0, 0), (p['x'][i]+400, p['y'][i]+300), 2)

    buffer_flipped = pygame.transform.flip(buffer, False, True)
    screen.blit(buffer_flipped, (0, 0))

    xydisplay = arial.render(f'x: {p["x"][i]:.3f}, y: {p["y"][i]:.3f}', True, (255, 255, 255))
    timedisplay = arial.render(f'time: {time.time() - start_t:.3f}', True, (255, 255, 255))
    velocities = arial.render(f'Left: {p["left_actual_velocity"][i]:.3f}, Right: {p["right_actual_velocity"][i]:.3f}', True, (255, 255, 255))
    screen.blit(xydisplay, (0, 0))
    screen.blit(timedisplay, (0, 20))
    screen.blit(velocities, (0, 40))

    pygame.display.flip()