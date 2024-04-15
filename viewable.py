import numpy as np
import matplotlib.pyplot as plt
positions = []  # To store ship positions
headings = []   # To store ship headings
velocities = [] # To store ship velocities
tug_forces = [] # To store tug forces and directions
for t in range(0, total_simulation_time, dt):
    # Existing simulation code...

    # Collect data for visualization
    positions.append(ship_state['position'].copy())
    headings.append(ship_state['heading'])
    velocities.append(ship_state['velocity'].copy())
    tug_forces.append((tug_force, ship_state['heading']))

    # Continue with the simulation...
# Convert collected data to numpy arrays for easier manipulation
positions = np.array(positions)
velocities = np.array(velocities)

# Plotting the ship trajectory with heading direction
plt.figure(figsize=(10, 8))
plt.quiver(positions[:, 0], positions[:, 1], np.cos(headings), np.sin(headings), scale=10, color='r', width=0.002, headwidth=3, headlength=4)
plt.title('Ship Trajectory with Heading')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.grid(True)
plt.axis('equal')

# Plotting the velocity vectors
plt.figure(figsize=(10, 8))
plt.quiver(positions[:, 0], positions[:, 1], velocities[:, 0], velocities[:, 1], scale=5, color='b', width=0.002, headwidth=3, headlength=4)
plt.title('Ship Velocity Vectors')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.grid(True)
plt.axis('equal')

# Plotting tug forces
tug_force_magnitudes, tug_force_angles = zip(*tug_forces)
tug_force_magnitudes = np.array(tug_force_magnitudes)
plt.figure(figsize=(10, 8))
plt.quiver(positions[:, 0], positions[:, 1], tug_force_magnitudes * np.cos(tug_force_angles), tug_force_magnitudes * np.sin(tug_force_angles), scale=1, color='g', width=0.005, headwidth=3, headlength=4)
plt.title('Tug Forces Over Time')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.grid(True)
plt.axis('equal')

# Show all plots
plt.show()
