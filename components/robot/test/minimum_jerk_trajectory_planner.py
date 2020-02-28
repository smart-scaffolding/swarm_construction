import matplotlib.pyplot as plt
import numpy as np


def mjtg(current, setpoint, frequency, move_time):
    trajectory = []
    trajectory_derivative = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq):
        trajectory.append(
            current + (setpoint - current) *
            (10.0 * (time/timefreq)**3
             - 15.0 * (time/timefreq)**4
             + 6.0 * (time/timefreq)**5))

        trajectory_derivative.append(
            frequency * (1.0/timefreq) * (setpoint - current) *
            (30.0 * (time/timefreq)**2.0
             - 60.0 * (time/timefreq)**3.0
             + 30.0 * (time/timefreq)**4.0))

    return trajectory, trajectory_derivative

def get_minimum_jerk_trajectory(points, average_velocity=5.0, frequency=100):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    x_points = []
    y_points = []
    z_points = []

    x_points_derivative = []
    y_points_derivative = []
    z_points_derivative = []

    for i in range(1, len(x)):
        x_initial = x[i-1] #initial position
        x_final = x[i]   #final position
        time_x = (x_final - x_initial) / average_velocity

        y_initial = y[i - 1]  # initial position
        y_final = y[i]  # final position
        time_y = (y_final - y_initial) / average_velocity

        z_initial = z[i - 1]  # initial position
        z_final = z[i]  # final position
        time_z = (z_final - z_initial) / average_velocity

        # print(time_x, time_y, time_z)
        time = max(abs(time_x), abs(time_y), abs(time_z))
        # print(time)

        traj_x, traj_vel_x = mjtg(x_initial, x_final, frequency, time)


        traj_y, traj_vel_y = mjtg(y_initial, y_final, frequency, time)


        traj_z, traj_vel_z = mjtg(z_initial, z_final, frequency, time)

        x_points.append(traj_x)
        y_points.append(traj_y)
        z_points.append(traj_z)

        x_points_derivative.append(traj_vel_x)
        y_points_derivative.append(traj_vel_y)
        z_points_derivative.append(traj_vel_z)


    final_values = []
    for x_l, y_l, z_l in zip(x_points, y_points, z_points):
        for x, y, z in zip(x_l, y_l, z_l):
            final_values.append((x, y, z))

    print(final_values)

    final_values_vel = []
    for x_l, y_l, z_l in zip(x_points_derivative, y_points_derivative, z_points_derivative):
        for x, y, z in zip(x_l, y_l, z_l):
            final_values_vel.append((x, y, z))
    return final_values, [x_points, y_points, z_points], final_values_vel, [x_points_derivative, y_points_derivative,
                                                                            z_points_derivative]


def plot_mjtg(initial_points, average_velocity=5.0, frequency=100):
    points, values, vel, vel_val = get_minimum_jerk_trajectory(initial_points, average_velocity, frequency)
    plt.plot(values[0][0], label="X values")
    plt.plot(values[1][0], label="Y values")
    plt.plot(values[2][0], label="Z values")

    plt.plot(vel_val[0][0], label="X vel")
    plt.plot(vel_val[1][0], label="Y vel")
    plt.plot(vel_val[2][0], label="Z vel")
    plt.ylabel('Position')
    plt.xlabel('Time')
    plt.title("Minimum Jerk Trajectory Generator")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    initial_points = np.array([[0, 0, 0], [80, 80, 80]])
    plot_mjtg(initial_points)
    points, values, vel, vel_val = get_minimum_jerk_trajectory(initial_points, average_velocity=10, frequency=100)
    print(points)