import models
import analyze
import torch as th
import pygame
import constants
import time
import numpy as np

def make_initial_state(transformer):
    return [th.tensor([
        transformer['left_voltage'](0),
        transformer['right_voltage'](0),
        transformer['left_velo'](0),
        transformer['right_velo'](0),
        transformer['left_accel'](0),
        transformer['right_accel'](0),
        transformer['left_eff'](0),
        transformer['right_eff'](0),
        1,
        transformer['dx'](0),
        transformer['dy'](0),
        transformer['dtheta'](0),
        transformer['speed'](0),
        1, 0
    ], device='cuda') for i in range(constants.lookback)]

def get_sizes(df):
    df = analyze.load_processed_df(name = df).dropna()
    dummy_set = analyze.RobotDataset(th.tensor(df.values).float(), df)
    past_size = dummy_set[0][0].shape[-1]
    present_size = dummy_set[0][1].shape[-1]
    future_size = dummy_set[0][2].shape[-1]
    return past_size, present_size, future_size

class Robot:
    def __init__(self, x, y, theta, df):
        self.x = x
        self.y = y
        self.theta = theta
        self.transformer = analyze.get_transformer(df)
        self.inverse_transformer = analyze.get_inverse_transformer(df)

        self.left_velo = 0
        self.right_velo = 0
        
        self.last_time = time.time()

        self.model = models.MotionModel(*get_sizes(df)).cuda().eval()
        self.model.load_state_dict(th.load(f"ml/motion/{constants.save_name}"))
        self.last_states = make_initial_state(self.transformer)

    def update(self, left_volt, right_volt):
        left_volt = self.transformer['left_voltage'](left_volt)
        right_volt = self.transformer['right_voltage'](right_volt)
        dt = 1

        present = th.tensor([left_volt, right_volt, dt], device='cuda')
        past = th.stack(self.last_states, dim=0).to('cuda', dtype=th.float32)
        partial_future = self.model(past, present)[0]

        dx, dy, dtheta, speed, left_velo, right_velo, left_eff, right_eff = partial_future.cpu().detach().numpy()

        last_left_velo = self.left_velo
        last_right_velo = self.right_velo
        self.left_velo = self.inverse_transformer['left_velo'](left_velo)
        self.right_velo = self.inverse_transformer['right_velo'](right_velo)
        left_accel = (self.inverse_transformer['left_velo'](left_velo) - last_left_velo) / dt
        right_accel = (self.inverse_transformer['right_velo'](right_velo) - last_right_velo) / dt

        self.x += self.inverse_transformer['dx'](dx)
        self.y += self.inverse_transformer['dy'](dy)
        self.theta += self.inverse_transformer['dtheta'](dtheta)

        self.last_states.pop(0)
        self.last_states.append(th.tensor([
            left_volt,
            right_volt,
            left_velo,
            right_velo,
            self.transformer['left_accel'](left_accel),
            self.transformer['right_accel'](right_accel),
            left_eff,
            right_eff,
            dt,
            dx,
            dy,
            dtheta,
            speed,
            np.cos(self.theta),
            np.sin(self.theta)
        ], device='cuda'))

if __name__ == "__main__":
    R = Robot(400, 400, 0, "driving_logs_1.csv")

    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((800, 800))
    left, right = 0, 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        if pygame.key.get_pressed()[pygame.K_UP]:
            left += 100

        if pygame.key.get_pressed()[pygame.K_DOWN]:
            left -= 100

        if pygame.key.get_pressed()[pygame.K_w]:
            right += 100
        
        if pygame.key.get_pressed()[pygame.K_s]:
            right -= 100

        left = min(max(-12000, left), 12000)
        right = min(max(-12000, right), 12000)

        R.update(left, right)
        print(R.x, R.y)

        screen.fill((0, 0, 0))

        pygame.draw.circle(screen, (255, 255, 255), (R.x, R.y), 10)
        pygame.draw.line(screen, (255, 255, 255), (R.x, R.y), (R.x + 20 * np.sin(R.theta), R.y + 20 * np.cos(R.theta)))

        surf = pygame.font.SysFont('Arial', 48).render(f"L: {left}, R: {right}", True, (255, 255, 255))
        screen.blit(surf, (0, 0))

        surf = pygame.font.SysFont('Arial', 48).render(f"Lv: {R.left_velo}, Rv: {R.right_velo}", True, (255, 255, 255))
        screen.blit(surf, (0, 90))
        pygame.display.update()
        time.sleep(0.02)