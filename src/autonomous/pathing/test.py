import math

# Example class with maxSpeed method (to simulate your Constraints class)
class Constraints:
    def __init__(self, max_vel, track_width, friction_coef):
        self.max_vel = max_vel
        self.track_width = track_width
        self.friction_coef = friction_coef
    
    # maxSpeed function based on given code
    def maxSpeed(self, curvature):
        max_turn_speed = ((2 * self.max_vel / self.track_width) * self.max_vel) / (abs(curvature) * self.max_vel + (2 * self.max_vel / self.track_width))
        if curvature == 0:
            return max_turn_speed
        max_slip_speed = math.sqrt(self.friction_coef * (1 / abs(curvature)) * 9.81 * 39.3701)
        return min(max_slip_speed, max_turn_speed)

# Initialize values
v_i = 10  # Initial velocity
t_i = 0
t_f = 0.05
ds = 0.01  # Distance step
L = 0.2  # Distance parameter
a_max = 20  # Maximum acceleration
max_iter = 10  # Maximum iterations to prevent infinite loops
tolerance = 1e-5  # Convergence tolerance

# Initialize the constraints object (you can set your own values here)
constraints = Constraints(max_vel=20, track_width=1.5, friction_coef=0.9)

# Define the curvature function C(t), for simplicity, we can assume C(t) = t here
def C(t):
    return math.sin(t ** 2) / (1 + t**1.5)  # Example curvature as a function of time (this can be replaced with actual function)

# Initial guess
v_f = v_i - 1

# Simulation loop
for iteration in range(max_iter):
    # Step 1: Compute alpha
    alpha = (C(t_f) * v_f - C(t_i) * v_i) * v_i / ds
    
    # Step 2: Compute a
    a = a_max - abs(alpha * (L / 2))
    a = max(0.01, a)  # Ensure a >= 0
    print(a)
    
    # Step 3: Compute the maximum speed using maxSpeed function
    curvature_at_tf = C(t_f)  # The curvature at t_f
    max_speed = constraints.maxSpeed(curvature_at_tf)  # S(C(t_f))+
    
    # Step 4: Update v_f
    v_f_new = min(max_speed, (v_i**2 + 2 * a * ds)**0.5)

    print(f"Iteration {iteration + 1}: v_f = {v_f_new}")

    # Check for convergence
    # if abs(v_f_new - v_f) < tolerance:
    #     print(f"Converged to v_f = {v_f_new} after {iteration + 1} iterations")
    #     break
    
    # Update v_f for the next iteration
    v_f = v_f_new
else:
    print("Did not converge within max iterations")
