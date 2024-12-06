import sys
import time
import numpy as np
import pygame

class Display:
    def __init__(self):
        pygame.init()
        pygame.font.init()

        self.arial = pygame.font.SysFont('Arial', 20)

        self.screen = pygame.display.set_mode((800, 600))
        self.buffer = pygame.Surface((800, 600))

        self.robot_base = pygame.Surface((40, 50), pygame.SRCALPHA)
        pygame.draw.rect(self.robot_base, (150, 150, 150), (0, 0, 40, 50))
        pygame.draw.line(self.robot_base, (255, 255, 255), (20, 25), (20, 50), 1)

    def draw_axes(self):
        for x in range(100, 800, 100):
            for y in range(100, 600, 100):
                pygame.draw.line(self.buffer, (100, 100, 100), (x, 0), (x, 600)) 
                pygame.draw.line(self.buffer, (100, 100, 100), (0, y), (800, y))

        pygame.draw.line(self.buffer, (255, 255, 255), (400, 0), (400, 600), 2)
        pygame.draw.line(self.buffer, (255, 255, 255), (0, 300), (800, 300), 2)

    def get_transformed_robot(self, theta):
        return pygame.transform.rotate(self.robot_base, theta * 180 / np.pi)

    def draw_robot(self, x, y, theta):
        robot = self.get_transformed_robot(theta)
        robot_rect = robot.get_rect(center=(x + 400, y + 300))
        self.buffer.blit(robot, robot_rect)
        pygame.draw.circle(self.buffer, (255, 100, 255), (x + 400, y + 300), 2)

    def start(self):
        self.start_t = time.time()

    def update(self, cur_info):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        self.buffer.fill((0, 0, 0))

        self.draw_axes()
        self.draw_robot(cur_info['x'], cur_info['y'], cur_info['theta'])

        buffer_flipped = pygame.transform.flip(self.buffer, False, True)
        self.screen.blit(buffer_flipped, (0, 0))

        xydisplay = self.arial.render(f'x: {cur_info["x"]:.3f}, y: {cur_info["y"]:.3f}', True, (255, 255, 255))
        timedisplay = self.arial.render(f'time: {time.time() - self.start_t:.2f}', True, (255, 255, 255))
        velocities = self.arial.render(f'Left: {cur_info["left_actual_velocity"]:.3f}, Right: {cur_info["right_actual_velocity"]:.3f}', True, (255, 255, 255))
        clampdisplay = self.arial.render(f'Clamp: {cur_info["clamp"]:.3f}', True, (255, 255, 255))
        intakedisplay = self.arial.render(f'Intake: {cur_info["intake"]:.3f}', True, (255, 255, 255))
        conveyordisplay = self.arial.render(f'Conveyor: {cur_info["conveyor"]:.3f}', True, (255, 255, 255))
        self.screen.blit(xydisplay, (0, 0))
        self.screen.blit(timedisplay, (0, 20))
        self.screen.blit(velocities, (0, 40))
        self.screen.blit(clampdisplay, (0, 60))
        self.screen.blit(intakedisplay, (0, 80))
        self.screen.blit(conveyordisplay, (0, 100))


        pygame.display.flip()