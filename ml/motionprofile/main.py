import numpy as np
import bisect

def path(u):
    return (
        u**3 - 3*(u ** 2) + 3*u,
        u**2 + u
    )

def path_deriv(u):
    return (
        3*(u**2) - 6*u + 3,
        2*u + 1
    )

def path_second_deriv(u):
    return (
        6*u - 6,
        2
    )

def curvature(u):
    dp = path_deriv(u)
    ddp = path_second_deriv(u)
    return (dp[0] * ddp[1] - dp[1] * ddp[0]) / (np.hypot(dp[0], dp[1]) ** 3 + 1e-6)

def gen_arc_list(res:int):
    distances = [0]
    for ui in range(res):
        uf = (ui + 1) / res; ui = ui / res
        pui = path(ui); puf = path(uf)
        distances.append(np.hypot(puf[0] - pui[0], puf[1] - pui[1]) + distances[-1])
    distances.append(distances[-1])
    return distances

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
    
class TwoDProfile:
    def __init__(self, mv:float, ma:float, res:int, ds:float, track_width:float):
        self.mv = mv
        self.ma = ma
        self.res = res
        self.track_width = track_width
        self.ds = ds
        self.velocities = []
        self.us = []

    def construct_profile(self, start_v=0, end_v=0):
        self.velocities = [start_v]
        self.us = [0]

        distances = gen_arc_list(self.res)
        curvature_cache = [-1]
            
        u = start_v; d = 0
        while d <= distances[-1]:
            arc_t = bisect.bisect_left(distances, d) / self.res
            dsLdsC = 1 + abs(curvature(arc_t)) * self.track_width
            v = max(0.01, min(self.mv / dsLdsC, np.sqrt(u**2 + 2*self.ma*self.ds)))

            self.velocities.append(v)
            self.us.append(arc_t)
            curvature_cache.append(dsLdsC)

            u = v
            d += self.ds

        u = end_v
        for i in range(len(self.velocities)-1, 0, -1):
            dsLdsC = curvature_cache[i]
            v = max(0.01, min(self.mv / dsLdsC, np.sqrt(u**2 + 2*self.ma*self.ds)))
            self.velocities[i] = min(self.velocities[i], v)

            u = v
            d -= self.ds

def main():
    import matplotlib.pyplot as plt

    profile = TwoDProfile(0.1, 0.1, 100000, 0.001, 9)
    profile.construct_profile(start_v=0)

    X = np.linspace(0, gen_arc_list(profile.res)[-1], len(profile.velocities))
    print(X)
    Y = profile.velocities
    CY = [1/(1+abs(curvature(u)) * profile.track_width) for u in profile.us]

    print(len(Y))

    plt.plot(X, Y)
    plt.plot(X, CY)
    plt.show()

if __name__ == "__main__":
    main()