import numpy as np
import matplotlib.pyplot as plt

# System parameters (adjust based on your RCS)
Izz = 0.1436 # Moment of inertia of entire Rocket (kg·m²)
# Izz = 0.01491745  # Moment of inertia of RCS (kg·m²)
tau_rcs = 17.64*2*0.1  # Max torque (N·m, from RCS_F * RCS_ARM)
dt = 0.1  # Control loop time (s)
min_fire = 0.06  # Minimum thruster fire time (s)

# PID parameters (initial guesses, to be tuned)
Kp = 0.1  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.2  # Derivative gain
# integral_max = 10.0  # Integral clamping limits
# integral_min = -10.0

# Simulation parameters
t_max = 10  # Simulation duration (s)
target_heading = 90.0  # Target heading (degrees)
initial_heading = 0.0  # Initial heading (degrees)
initial_omega = 0.0  # Initial angular velocity (deg/s)

# Initialize arrays for simulation
t = np.arange(0, t_max, dt)
n_steps = len(t)
theta = np.zeros(n_steps)  # Heading (degrees)
omega = np.zeros(n_steps)  # Angular velocity (deg/s)
error = np.zeros(n_steps)  # Heading error (degrees)
tau_output = np.zeros(n_steps)  # PID torque output (N·m)
duty_cycle = np.zeros(n_steps)  # Thruster duty cycle

# PID state variables
integral = 0.0
prev_error = 0.0

# Set initial conditions
theta[0] = initial_heading
omega[0] = initial_omega

# Simulation loop
for k in range(n_steps):
    # if t[k] >=5:
    #     target_heading = 180
    # Compute error (with rollover handling)
    error[k] = target_heading - theta[k]
    if error[k] > 180:
        error[k] -= 360
    elif error[k] < -180:
        error[k] += 360

    # PID controller
    P = Kp * error[k]
    integral += error[k] * dt
    # if integral > integral_max:
    #     integral = integral_max
    # elif integral < integral_min:
    #     integral = integral_min
    I = Ki * integral
    D = Kd * (error[k] - prev_error) / dt if k > 0 else 0.0
    
    prev_error = error[k]
    tau_output[k] = P + D

    # Compute duty cycle and apply thruster logic
    duty_cycle[k] = tau_output[k] / tau_rcs
    if duty_cycle[k] > 1.0:
        duty_cycle[k] = 1.0
    elif duty_cycle[k] < -1.0:
        duty_cycle[k] = -1.0

    fire_time = abs(duty_cycle[k]) * dt
    if fire_time < min_fire:
        duty_cycle[k] = 0.0
        if abs(error[k]) < 2:  # Deadband reset
            integral = 0.0

    # Apply control torque (positive = CW, negative = CCW)
    tau_control = duty_cycle[k] * tau_rcs

    # Update system dynamics (Euler integration, in degrees)
    if k < n_steps - 1:
        # print(P, I, D, omega, error, theta, omega[k], tau_control / I, (tau_control / I) * dt)
        omega[k+1] = omega[k] + (tau_control / Izz) * dt * (180 / np.pi)  # Convert rad/s² to deg/s²
        theta[k+1] = theta[k] + omega[k] * dt
        # print(P, I, D, omega, error, theta)
        # print(theta[k] + omega[k] * dt, theta[k], omega[k])

        # Handle heading rollover
        if theta[k+1] > 360:
            theta[k+1] -= 360
        elif theta[k+1] < 0:
            theta[k+1] += 360

# Plot results
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t, theta, label='Heading (deg)')
plt.axhline(target_heading, color='r', linestyle='--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading (deg)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(t, error, label='Error (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Error (deg)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(t, tau_output, label='PID Torque (N·m)')
plt.plot(t, duty_cycle * tau_rcs, label='Applied Torque (N·m)')
plt.xlabel('Time (s)')
plt.ylabel('Torque (N·m)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

