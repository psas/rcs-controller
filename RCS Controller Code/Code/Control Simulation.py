import numpy as np
import matplotlib.pyplot as plt

# System parameters
I = 0.015  # Moment of inertia (kg*m^2)
dt = 0.0001  # Time step (s)
total_time = 5.0  # Simulation duration (s)
steps = int(total_time / dt)

# PID gains
Kp = 0.1
Ki = 0.0
Kd = 0.1

# Desired angle
theta_desired = np.pi / 2  # 45 degrees

# State variables
theta = 0.0      # Current angle (rad)
omega = 0.0      # Angular velocity (rad/s)
integral_error = 0.0
previous_error = 0.0

# Data storage for plotting
theta_history = []
time_history = []

for step in range(steps):
    time = step * dt
    error = theta_desired - theta
    integral_error += error * dt
    derivative_error = (error - previous_error) / dt

    # PID control law (torque command)
    torque = Kp * error + Ki * integral_error + Kd * derivative_error

    # System dynamics (Euler integration)
    alpha = torque / I  # Angular acceleration
    omega += alpha * dt
    theta += omega * dt

    previous_error = error

    # Store for plotting
    print(theta, omega)
    theta_history.append(theta)
    time_history.append(time)

# Plot results
plt.plot(time_history, theta_history, label='Angle (rad)')
plt.axhline(y=theta_desired, color='r', linestyle='--', label='Desired angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('PID Control of Rotational System')
plt.legend()
plt.grid(True)
plt.show()