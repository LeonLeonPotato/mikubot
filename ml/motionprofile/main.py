import numpy as np

def path(u):
    return (
        u**3 - 2*(u ** 2) + 3*u,
        u**2 + u
    )

def path_deriv(u):
    return (
        3*(u**2) - 4*u + 3,
        2*u + 1
    )

def path_second_deriv(u):
    return (
        6*u - 4,
        2
    )

def curvature(u):
    p = path(u)
    dp = path_deriv(u)
    ddp = path_second_deriv(u)
    return (dp[0] * ddp[1] - dp[1] * ddp[0]) / (np.hypot(dp[0], dp[1]) ** 3)

class TrapezoidalProfile:
    def __init__(self, mv:float, ma:float, d:float):
        self.mv = mv
        self.ma = ma
        self.d = d
        self.v_eff = min(np.sqrt(d * ma), mv)
        self.t1 = self.v_eff / ma
        self.t2 = d / self.v_eff
        self.t_max = self.t1 + self.t2
        self.__c_s_coast = 0.5 * self.ma * self.t1**2
        self.__c_s_decel = 0.5 * self.ma * self.t1**2 - self.v_eff * self.t1
        self.__c_v_decel = self.v_eff + self.ma * self.t2
        self.__discretized_s = None
        self.__discretized_v = None
    
    def distance(self, t):
        if 0 <= t < self.t1:
            return 0.5 * self.ma * t**2
        elif self.t1 <= t < self.t2:
            return self.v_eff * (t - self.t1) + self.__c_s_coast
        elif self.t2 <= t <= self.t_max:
            return self.v_eff * t - 0.5 * self.ma * (t - self.t2)**2 + self.__c_s_decel
        return 0
    
    def velocity(self, t):
        if 0 <= t < self.t1:
            return self.ma * t
        elif self.t1 <= t < self.t2:
            return self.v_eff
        elif self.t2 <= t <= self.t_max:
            return self.__c_v_decel - self.ma * t
        return 0
    
    def acceleration(self, t):
        if 0 <= t < self.t1:
            return self.ma
        elif self.t2 <= t <= self.t_max:
            return -self.ma
        return 0
    
    def discretized_s(self, dt):
        if self.discretized_s != None and self.discretized_s[1] == dt:
            return self.discretized_s[0]
        t = 0
        distances = []
        while t <= self.t_max:
            distances.append(self.distance(t))
            t += dt
        self.discretized_s = (distances, dt)
        return distances
    
    def discretized_v(self, dt):
        if self.discretized_v != None and self.discretized_v[1] == dt:
            return self.discretized_v[0]
        t = 0
        velocities = []
        while t <= self.t_max:
            velocities.append(self.velocity(t))
            t += dt
        self.discretized_v = (velocities, dt)
        return velocities

def main():
    import matplotlib.pyplot as plt

    profile = TrapezoidalProfile(1, 1, 4)
    X = np.linspace(0, profile.t_max, 100)
    Y = [profile.distance(x) for x in X]
    dY = [profile.velocity(x) for x in X]
    ddY = [profile.acceleration(x) for x in X]

    plt.plot(X, Y)
    plt.plot(X, dY)
    plt.plot(X, ddY)
    plt.show()


if __name__ == "__main__":
    main()