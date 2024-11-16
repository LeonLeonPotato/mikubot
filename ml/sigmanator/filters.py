import numpy as np
import matplotlib.pyplot as plt
import time

class NaiveParticleFilter:
    def __init__(self, num_particles:int, state_dim:int, transition_model, observation_model):
        self.num_particles = num_particles
        self.state_dim = state_dim
        self.transition_model = transition_model
        self.observation_model = observation_model
        self.particles = np.random.normal(size=(num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles

    def update(self, observation):
        self.weights = self.observation_model(self.particles, observation)
        self.weights /= self.weights.sum()
        indices = np.random.choice(self.num_particles, size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]

    def predict(self):
        self.transition_model(self.particles)

    def get_state(self):
        return self.particles.mean(axis=0)
    
class GojoParticleFilter:
    # Guess why I named it Gojo
    class NormalMeasurementCloud:
        def __init__(self, mean, std, tick, decay_factor=1):
            self.mean = mean
            self.std = std
            self.decay_factor = decay_factor
            self.init_tick = tick

        def calculate(self, x, current_tick):
            dt = (current_tick - self.init_tick) * self.decay_factor
            scale = 1 / np.sqrt(2 * np.pi * (self.std ** 2 + dt))
            power = -0.5 * (x - self.mean) ** 2 / (self.std ** 2 + dt)
            return scale * np.exp(power)
        
        def gradient(self, x, current_tick):
            dt = (current_tick - self.init_tick) * self.decay_factor
            scale = (x - self.mean) / (self.std ** 2 + dt)
            return -scale * self.calculate(x, current_tick)

    def __init__(self, num_distribs:int, state_dim:int, guess:np.ndarray, steps:int, step_size:float, transition_model, remove_predicate):
        self.num_distribs = num_distribs
        self.state_dim = state_dim
        self.transition_model = transition_model
        self.remove_predicate = remove_predicate
        self.guess = guess
        self.steps = steps
        self.step_size = step_size
        self.tick = 0
        self.distribs = []

        if state_dim != 1:
            raise ValueError("Only 1-dimensional state space is supported for now")

    def __update_guess(self):
        # performs gradient ascent on the distributions, based on the last guess
        for step in range(self.steps):
            av = 0
            for distrib in self.distribs:
                av += distrib.gradient(self.guess, self.tick).item()
            av = av / len(self.distribs)
            self.guess += av * self.step_size
            
    def update(self, observation, cov, decay_factor=1):
        self.tick += 1
        self.distribs.append(self.NormalMeasurementCloud(observation, cov, self.tick, decay_factor))

        # remove distributions that are no longer relevant
        i = 0
        while i < len(self.distribs):
            if self.remove_predicate(self.distribs[i]):
                self.distribs.pop(i)
            else:
                i += 1

        # remove excess distributions (limit computation and memory)
        while len(self.distribs) > self.num_distribs:
            self.distribs.pop(0)

        self.__update_guess()

    def predict(self):
        self.transition_model(self.distribs, self.tick)

    def get_state(self):
        return self.guess
    
def make_dataset(seed = 0):
    np.random.seed(seed)

    true_x = np.linspace(0, 20, 2000)
    true_y = true_x
    measured_y = true_y + np.random.normal(size=true_y.size, scale=1)
    return true_x, true_y, measured_y
    
def __test_gojo(dataset):
    def transitioner(distribs, tick):
        # for distrib in distribs:
        #     distrib.mean += np.cos(tick * (20 / 2000)) * (20 / 2000)
        pass

    def remover(distrib):
        return False

    distribs = 5
    gojo = GojoParticleFilter(
        distribs, 1, 0, 20, 1,
        transitioner, remover
    )

    true_x, true_y, measured_y = dataset
    predicted_y = []

    start_t = time.time()
    for y in measured_y:
        gojo.update(y, 2, decay_factor=1)
        gojo.predict()
        predicted_y.append(gojo.get_state())
    end_t = time.time()
    print("Time taken (Gojo):", end_t - start_t)

    return predicted_y

def __test_naive(dataset):
    def transitioner(particles):
        particles += np.random.normal(size=particles.shape, scale=1)
        pass

    naive = NaiveParticleFilter(
        10000, 1,
        transitioner, lambda x, y: np.exp(-0.5 * (x - y) ** 2).sum(axis=1)
    )

    true_x, true_y, measured_y = dataset
    predicted_y = []

    start_t = time.time()
    for y in measured_y:
        naive.predict()
        naive.update(np.array([y]))
        predicted_y.append(naive.get_state()[0])
    end_t = time.time()

    print("Time taken (Naive):", end_t - start_t)

    return predicted_y

if __name__ == "__main__":
    dataset = make_dataset(2)
    predicted_y_naive = __test_naive(dataset)
    predicted_y_gojo = __test_gojo(dataset)
    true_x, true_y, measured_y = dataset

    print("Naive error:", np.abs(true_y - predicted_y_naive).mean())
    print("Gojo error:", np.abs(true_y - predicted_y_gojo).mean())

    plt.plot(true_x, true_y, label="True")
    plt.scatter(true_x, measured_y, label="Measured", c="red", s=1)
    plt.plot(true_x, predicted_y_naive, label="Naive")
    plt.plot(true_x, predicted_y_gojo, label="Gojo")
    plt.legend()
    plt.grid()
    plt.show()
