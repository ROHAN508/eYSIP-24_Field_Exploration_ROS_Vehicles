import numpy as np
import matplotlib.pyplot as plt

# Constants
a = 3 / np.sqrt(2)
num_points_initial = 1000  # initial number of points to ensure smooth curve

# Parametric equations for lemniscate of Bernoulli
t = np.linspace(0, 2 * np.pi, num_points_initial)
x = a * np.sqrt(2) * np.cos(t) / (1 + np.sin(t)**2)
y = a * np.sqrt(2) * np.cos(t) * np.sin(t) / (1 + np.sin(t)**2)

# Calculate the angles between points
angles = np.arctan2(np.diff(y), np.diff(x))

# Function to reduce the number of points while maintaining the angle constraint
def reduce_points(x, y, max_points, max_angle_diff):
    selected_indices = [0]
    last_angle = angles[0]
    for i in range(1, len(x) - 1):
        angle = angles[i]
        if np.abs(angle - last_angle) <= max_angle_diff:
            selected_indices.append(i)
            last_angle = angle
        if len(selected_indices) >= max_points:
            break
    selected_indices.append(len(x) - 1)
    return x[selected_indices], y[selected_indices]

# Reduce points
max_angle_diff = np.pi / 4  # 45 degrees in radians
x_reduced, y_reduced = reduce_points(x, y, 60, max_angle_diff)

# Plot the reduced lemniscate
plt.plot(x_reduced, y_reduced, marker='o')
plt.title('Reduced Lemniscate of Bernoulli')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.grid(True)
plt.show()

# Create numpy arrays for x and y coordinates
x_coordinates = np.array(x_reduced)
y_coordinates = np.array(y_reduced)

x_coordinates, y_coordinates
