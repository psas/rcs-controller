import numpy as np
import matplotlib.pyplot as plt

# Simulate a noisy signal
np.random.seed(42)
time = np.linspace(0, 10, 100)
signal = np.sin(time) + np.random.normal(0, 0.5, time.shape)  # Signal with noise

# Running average (simple moving average with window size 5)
window_size = 10
running_avg = np.convolve(signal, np.ones(window_size)/window_size, mode='same')

# Exponential smoothing with alpha = 0.1
alpha = 0.4
exp_smooth = np.zeros_like(signal)
exp_smooth[0] = signal[0]  # Initialize with the first value
for i in range(1, len(signal)):
    exp_smooth[i] = alpha * signal[i] + (1 - alpha) * exp_smooth[i-1]

# Plotting the signals
plt.figure(figsize=(10, 6))
plt.plot(time, signal, label="Noisy Signal", alpha=0.5)
plt.plot(time, running_avg, label="Running Average (Window 5)", linewidth=2)
plt.plot(time, exp_smooth, label="Exponential Smoothing (alpha=0.1)", linewidth=2, linestyle='--')
plt.title("Comparison of Low-Pass Filtering Methods")
plt.xlabel("Time")
plt.ylabel("Signal Value")
plt.legend(loc="upper right")
plt.grid(True)
plt.show()