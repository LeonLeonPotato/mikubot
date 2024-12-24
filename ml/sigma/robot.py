import numpy as np

class DrivetrainArgs:
    def __init__(self,
                 num_motors:int,
                 motor_backemf:float,
                 motor_resistance:float,
                 voltage_maxchange:float,
                 max_voltage:float,
                 torque_constant:float,
                 moment_of_inertia:float,
                 wheel_radius:float,
                 gear_ratio:float,
                 ):
        self.num_motors:int = num_motors
        self.motor_backemf:float = motor_backemf
        self.motor_resistance:float = motor_resistance
        self.voltage_maxchange:float = voltage_maxchange
        self.max_voltage:float = max_voltage
        self.torque_constant:float = torque_constant
        self.moment_of_inertia:float = moment_of_inertia
        self.wheel_radius:float = wheel_radius
        self.gear_ratio:float = gear_ratio

class Drivetrain:
    def __init__(self, args:DrivetrainArgs):
        self.args = args
        self.set_voltage = 0
        self.voltage = 0
        self.current = 0

        self.angular_position = 0
        self.angular_velocity = 0
        self.angular_acceleration = 0
        self.torque = 0

    def linear_velocity(self):
        return self.angular_velocity * self.args.wheel_radius * self.args.gear_ratio
    
    def linear_acceleration(self):
        return self.angular_acceleration * self.args.wheel_radius * self.args.gear_ratio
    
    def rpm(self):
        return self.angular_velocity * 60 / (2*np.pi)
    
    def apm(self):
        return self.angular_acceleration * 60 / (2*np.pi)

    def update(self, set_voltage:float, dt:float):
        self.set_voltage = np.clip(set_voltage, -self.args.max_voltage, self.args.max_voltage)
        self.voltage = np.clip(
            self.set_voltage,
            self.voltage - self.args.voltage_maxchange * dt,
            self.voltage + self.args.voltage_maxchange * dt
        )
        backemf = self.args.motor_backemf * self.angular_velocity
        self.current = (self.voltage - backemf) / self.args.motor_resistance
        self.torque = self.current * self.args.torque_constant * self.args.num_motors
        
        self.angular_acceleration = self.torque / self.args.moment_of_inertia
        self.angular_position += self.angular_velocity*dt + 0.5*self.angular_acceleration*dt**2
        self.angular_velocity += self.angular_acceleration * dt

if __name__ == "__main__":
    args = DrivetrainArgs(
        num_motors=3,
        motor_backemf=0.1894483 * 520 / 600,
        motor_resistance=3.84,
        voltage_maxchange=12,
        max_voltage=12,
        torque_constant=0.198,
        moment_of_inertia=1,
        wheel_radius=4.125,
        gear_ratio=0.6
    )
    drivetrain = Drivetrain(args)
    times = []
    angular_velocities = []
    angular_accelerations = []
    for i in range(1000):
        drivetrain.update(12, 0.01)
        times.append(i*0.01)
        angular_velocities.append(drivetrain.rpm())
        angular_accelerations.append(drivetrain.apm())
    import matplotlib.pyplot as plt
    plt.plot(times, angular_velocities)
    plt.plot(times, angular_accelerations)
    plt.show()