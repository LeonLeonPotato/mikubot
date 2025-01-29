import matplotlib.pyplot as plt
import numpy as np

gain = 40.2; time_constant = 0.13; noise = lambda x: 5 + abs(x) * 0.02
dt = 0.01

times = [0]
output_volts = [0]
true_readings = [0]
noisy_readings = [0]
filtered = [0]

def output(i):
    return np.sin(i / 100) * 12

class LinearKalmanFilter:
    def __init__(self, x0, P0, Q, R, T, G):
        # System parameters
        self.T = T      # System time constant
        self.G = G      # System gain
        
        # Initialize state (1D vector)
        self.state = np.array([[x0]])          # x
        
        # Initialize covariance matrices
        self.error_cov = np.array([[P0]])      # P
        self.process_noise = np.array([[Q]])   # Q
        self.meas_noise = R     # R
        
        # Measurement matrix (scalar 1)
        self.H = np.array([[1.0]])

    def predict(self, u, dt):
        # Calculate system matrices
        F = np.array([[1.0 - (dt / self.T)]])
        B = np.array([[(dt * self.G) / self.T]])
        
        # Predict state
        self.state = F @ self.state + B * u
        
        # Predict error covariance
        self.error_cov = F @ self.error_cov @ F.T + self.process_noise

    def update(self, z):
        # Calculate innovation
        y = np.array([[z]]) - self.H @ self.state
        
        # Innovation covariance
        S = self.H @ self.error_cov @ self.H.T + self.meas_noise(self.state)
        
        # Kalman gain
        K = self.error_cov @ self.H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.state = self.state + K @ y
        
        # Update error covariance
        I = np.eye(1)  # Identity matrix
        self.error_cov = (I - K @ self.H) @ self.error_cov

    @property
    def position(self):
        return self.state[0, 0]

    @property
    def uncertainty(self):
        return self.error_cov[0, 0]

kf = LinearKalmanFilter(
    0, 0, 1, noise, time_constant, gain
)

for i in range(1, 200):
    output_volt = output(i)
    kf.predict(u=output_volt, dt=dt)
    true = true_readings[-1] + (gain * output_volt - true_readings[-1]) * dt / time_constant
    noise_i = np.random.normal(0, noise(true))
    noisy = true + noise_i

    output_volts.append(output_volt)
    true_readings.append(true)
    noisy_readings.append(noisy)
    kf.update(noisy)
    filtered.append(kf.position)
    times.append(i * dt)

plt.plot(times, true_readings, label='True')
plt.plot(times, filtered, label='Filtered')
plt.scatter(times, noisy_readings, label='Noisy', s=1)
plt.legend()
plt.show()