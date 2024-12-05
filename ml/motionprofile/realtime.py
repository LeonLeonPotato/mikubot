import subprocess
import threading
import sys
import bisect
import time
import numpy as np
import pygame
import collections

cur_info = collections.defaultdict(float)

def run():
    prospath = {
        "macos": "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
        "win32": "C:/Users/leon/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros"
    }

    proc = subprocess.Popen([
        prospath[sys.platform],
        "terminal"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def updater():
        while True:
            line = proc.stdout.readline().decode("utf-8").strip()
            line = line.split(",")
            if len(line) == 12:
                cur_info['time'] = float(line[0] / 1e6)
                cur_info["x"] = float(line[1])
                cur_info["y"] = float(line[2])
                cur_info['theta'] = float(line[3])
                cur_info['left_voltage'] = float(line[4])
                cur_info['right_voltage'] = float(line[5])
                cur_info['left_velocity'] = float(line[6])
                cur_info['right_velocity'] = float(line[7])
                cur_info['left_actual_voltage'] = float(line[8])
                cur_info['right_actual_voltage'] = float(line[9])
                cur_info['left_actual_velocity'] = float(line[10])
                cur_info['right_actual_velocity'] = float(line[11])

    lidarthread = threading.Thread(target=updater, daemon=True)
    lidarthread.start()

if __name__ == "__main__":
    run()

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
        pygame.draw.circle(buffer, (255, 100, 255), (x + 400, y + 300), 2)

    running = True
    start_t = time.time()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        buffer.fill((0, 0, 0))

        draw_axes()
        draw_robot(cur_info['x'], cur_info['y'], cur_info['theta'])

        buffer_flipped = pygame.transform.flip(buffer, False, True)
        screen.blit(buffer_flipped, (0, 0))

        xydisplay = arial.render(f'x: {cur_info['x']:.3f}, y: {cur_info['y']:.3f}', True, (255, 255, 255))
        timedisplay = arial.render(f'time: {time.time() - start_t:.2f}', True, (255, 255, 255))
        velocities = arial.render(f'Left: {cur_info["left_actual_velocity"]:.3f}, Right: {cur_info["right_actual_velocity"]:.3f}', True, (255, 255, 255))
        screen.blit(xydisplay, (0, 0))
        screen.blit(timedisplay, (0, 20))
        screen.blit(velocities, (0, 40))

        pygame.display.flip()