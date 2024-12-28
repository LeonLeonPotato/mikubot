import numpy as np

# Given constants
radius = 0.5 / 100  # radius in meters (0.5 cm)
area = np.pi * radius ** 2  # cross-sectional area in m^2
k_cu = 398  # thermal conductivity of copper (W/m·K)
k_fe = 73   # thermal conductivity of iron (W/m·K)
T_initial = 25  # initial temperature in degrees Celsius
T_source = 80  # temperature of heating element in degrees Celsius
L_values = np.linspace(0, 3, 100)  # 10 different values of L from 0 to 3 cm

# Function to calculate temperature at the other end
def calculate_temperature(L, k_cu, k_fe, area, T_initial, T_source):
    # Convert lengths from cm to m
    L_cu = L / 100  # copper length in meters
    L_fe = (3 - L) / 100  # iron length in meters
    
    # Thermal resistances
    R_cu = L_cu / (k_cu * area)
    R_fe = L_fe / (k_fe * area)
    
    # Total thermal resistance
    R_total = R_cu + R_fe
    
    # Temperature at the far end
    T_end = T_initial + (R_fe / R_total) * (T_source - T_initial)
    
    return T_end

# Calculate temperatures for each L
temperatures = [calculate_temperature(L, k_cu, k_fe, area, T_initial, T_source) for L in L_values]

# Display the result as a table
import pandas as pd

df = pd.DataFrame({
    "L (cm)": L_values,
    "Temperature at far end (C)": temperatures
})

print(df)