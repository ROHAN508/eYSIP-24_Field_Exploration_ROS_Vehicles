import numpy as np
import matplotlib.pyplot as plt

# Initial conditions
x = np.array([[0.0], [0.0]])
P = np.array([[1.0, 0.0], [0.0, 1.0]])
Q = np.array([[0.001, 0.0], [0.0, 0.003]])
R = np.array([[0.5, 0.0], [0.0, 0.5]])
H = np.array([[1.0, 0.0], [0.0, 1.0]])
I = np.eye(2)

# Process simulation parameters
acc_x = 1.2  # m/s² (after bias correction)
pwm_val = np.array([-40, -35, -30, -25, -20, 0, 20, 25, 30, 35, 40])
speed = np.array([-0.945, -0.79, -0.656, -0.508, -0.35, 0, 0.33, 0.529, 0.7165, 0.8675, 1.028])
dt = 0.1  # time step
iterations = 500

# Variables to store results
distance_estimates = []
speed_estimates = []

# Simulation loop
for i in range(iterations):
    # Simulate changing PWM every 100 iterations
    if i % 100 == 0:
        pwm = pwm_val[(i // 100) % len(pwm_val)]
    
    # State transition matrix
    F = np.array([[1, dt], [0, 1]])
    
    # Control input matrix
    B = np.array([[0.5 * dt**2], [dt]])
    
    # Control input (acceleration)
    u = np.array([[acc_x]])
    
    # Prediction step
    x_pred = F @ x + B @ u
    P_pred = F @ P @ F.T + Q
    
    # Simulated measurement (position and velocity)
    measured_speed = np.interp(pwm, pwm_val, speed)
    z = np.array([[x[0, 0] + measured_speed * dt], [measured_speed]])
    
    # Measurement update (correction) step
    y = z - (H @ x_pred)
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # Update state estimate
    x = x_pred + K @ y
    
    # Update state covariance matrix
    P = (I - K @ H) @ P_pred
    
    # Store results
    distance_estimates.append(x[0, 0])
    speed_estimates.append(x[1, 0])

# Plotting the results
plt.figure(figsize=(12, 6))

# Distance plot
plt.subplot(2, 1, 1)
plt.plot(distance_estimates, label='Estimated Distance')
plt.xlabel('Iteration')
plt.ylabel('Distance (m)')
plt.legend()

# Speed plot
plt.subplot(2, 1, 2)
plt.plot(speed_estimates, label='Estimated Speed', color='r')
plt.xlabel('Iteration')
plt.ylabel('Speed (m/s)')
plt.legend()

plt.tight_layout()
plt.show()
