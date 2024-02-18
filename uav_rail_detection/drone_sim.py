import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def simulate_drone_attitude(duration, dt):
    # Initial conditions
    pitch, roll, yaw = 0.0, 0.0, 0.0  # Initial attitude angles in radians
    angular_velocity = np.array([0.1, 0.05, 0.03])  # Angular velocity in rad/s for pitch, roll, yaw

    # Time vector
    time = np.arange(0, duration, dt)

    # Lists to store attitude angles
    pitch_list, roll_list, yaw_list = [], [], []

    # Simulation loop
    for t in time:
        # Update attitude angles based on angular velocity
        pitch += angular_velocity[0] * dt
        roll += angular_velocity[1] * dt
        yaw += angular_velocity[2] * dt

        # Append current attitude angles to lists
        pitch_list.append(pitch)
        roll_list.append(roll)
        yaw_list.append(yaw)


    return time, pitch_list, roll_list, yaw_list

def plot_attitude(time, pitch, roll, yaw):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(time, pitch, roll, label='Attitude')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch (rad)')
    ax.set_zlabel('Roll (rad)')
    ax.set_title('Drone Attitude Simulation')
    ax.legend()
    plt.show()

def update_plot(num, time, pitch, roll, yaw, line):
    line.set_data(time[:num], pitch[:num])
    line.set_3d_properties(roll[:num])
    return line,

def animate_drone_attitude(time, pitch, roll, yaw):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch (rad)')
    ax.set_zlabel('Roll (rad)')
    ax.set_title('Drone Attitude Animation')

    # Plot initial line
    line, = ax.plot([], [], [], lw=2)
    
    # Set limits for better visualization
    ax.set_xlim([0, max(time)])
    ax.set_ylim([min(pitch), max(pitch)])
    ax.set_zlim([min(roll), max(roll)])

    # Set anime style
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(False)

    # Animation
    ani = FuncAnimation(
        fig, update_plot, len(time),
        fargs=(time, pitch, roll, yaw, line),
        interval=50, blit=True
    )

    plt.show()
# Simulation parameters
duration = 10.0  # Total simulation time in seconds
dt = 0.1  # Time step in seconds

# Run simulation
time, pitch, roll, yaw = simulate_drone_attitude(duration, dt)

# Animate results
animate_drone_attitude(time, pitch, roll, yaw)

