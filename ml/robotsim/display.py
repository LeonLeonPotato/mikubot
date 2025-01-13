import sys
import time
import numpy as np
import pygame

class Display:
    def __init__(self, robot_width, robot_height):
        self.arial = pygame.font.SysFont('Arial', 20)

        self.robot_base = pygame.Surface((robot_width, robot_height), pygame.SRCALPHA)
        pygame.draw.rect(self.robot_base, (150, 150, 150), (0, 0, robot_width, robot_height))
        pygame.draw.line(self.robot_base, (255, 255, 255), (robot_width/2, robot_height/2), (robot_width/2, robot_height), 2)
        pygame.draw.circle(self.robot_base, (255, 100, 255), (robot_width/2, robot_height/2), 5)

    def draw_axes(self, buffer):
        for x in range(100, 800, 100):
            pygame.draw.line(buffer, (100, 100, 100), (x, 0), (x, 600), 1)
        for y in range(100, 600, 100):
            pygame.draw.line(buffer, (100, 100, 100), (0, y), (800, y), 1)

        pygame.draw.line(buffer, (255, 255, 255), (400, 0), (400, 600), 2)
        pygame.draw.line(buffer, (255, 255, 255), (0, 300), (800, 300), 2)

    def get_transformed_robot(self, theta):
        return pygame.transform.rotate(self.robot_base, theta * 180 / np.pi)

    def draw_robot(self, buffer, x, y, theta):
        robot = self.get_transformed_robot(theta)
        robot_rect = robot.get_rect(center=(x + 400, y + 300))
        buffer.blit(robot, robot_rect)

    def draw_on_buffer(self, buffer, cur_info):
        self.draw_axes(buffer)
        self.draw_robot(buffer, cur_info['x'], cur_info['y'], cur_info['theta'])

        buffer = pygame.transform.flip(buffer, False, True)

        xydisplay = self.arial.render(f'x: {cur_info["x"]:.3f}, y: {cur_info["y"]:.3f}', True, (255, 255, 255))
        velocities = self.arial.render(f'VL: {cur_info["left_actual_velocity"]:.3f}, VR: {cur_info["right_actual_velocity"]:.3f}', True, (255, 255, 255))
        accelerations = self.arial.render(f'AL: {cur_info["left_actual_accel"]:.3f}, AR: {cur_info["right_actual_accel"]:.3f}', True, (255, 255, 255))

        buffer.blit(xydisplay, (0, 0))
        buffer.blit(velocities, (0, 20))
        buffer.blit(accelerations, (0, 40))

        return buffer