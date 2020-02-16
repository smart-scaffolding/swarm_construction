import numpy as np


def quintic_polynomial(initial_conditions):
    initial_position = initial_conditions[0]
    final_position = initial_conditions[1]
    initial_velocity = initial_conditions[2]
    final_velocity = initial_conditions[3]
    initial_time = initial_conditions[4]
    final_time = initial_conditions[5]
    initial_acceleration = initial_conditions[6]
    final_acceleration = initial_conditions[7]

    coefficients = np.array(
        [
          [1,       initial_time,     initial_time**2,    initial_time**3,     initial_time**4,     initial_time**5],
          [0,       1,                2*initial_time,     3*initial_time**2,   4*initial_time**3,   5*initial_time**4],
          [0,       0,                2,                  6*initial_time,      12*initial_time**2,  20*initial_time**3],
          [1,       final_time,       final_time**2,      final_time**3,       final_time**4,       final_time**5],
          [0,       1,                2*final_time,       3 * final_time**2,   4*final_time**4,     5*final_time**4],
          [0,       0,                2,                  6 * final_time,      12*final_time**2,    20*final_time**3],
        ]
    )

    result = np.array([initial_position, initial_velocity, initial_acceleration, final_position, final_velocity,
                       final_acceleration]).transpose()

    return np.dot(np.linalg.inv(coefficients),result)

def create_quintic_trajectory(initial_conditions, setpoints):
    initial_position = initial_conditions[0]
    final_position = initial_conditions[1]
    initial_velocity = initial_conditions[2]
    final_velocity = initial_conditions[3]
    initial_time = initial_conditions[4]
    final_time = initial_conditions[5]
    initial_acceleration = initial_conditions[6]
    final_acceleration = initial_conditions[7]

    coefficients = quintic_polynomial(initial_conditions)

    #Create time matrix
    time = np.linspace(initial_time, final_time, setpoints)

    #Quintic polynomial
    quintic = coefficients[0] + coefficients[1] * time + coefficients[2] * time ** 2 + coefficients[3] * \
        time ** 3 + coefficients[4] * time ** 4 + coefficients[5] * time ** 5

    # quintic = coefficients[1] * time
    return quintic






def get_quintic_trajectory(points, set_points=30, start_time=0, end_time=1):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    x_points = []
    y_points = []
    z_points = []

    for i in range(1, len(x)):
        values_x = np.array(
                    [
                        x[i-1], #initial position
                        x[i],   #final position
                        0,      #initial velocity
                        0,      #final velocity
                        start_time,
                        end_time,
                        0,      #initial acceleration
                        0,      #final acceleration
                       ]).transpose()

        values_y = np.array(
            [
                y[i - 1],  # initial position
                y[i],  # final position
                0,  # initial velocity
                0,  # final velocity
                start_time,
                end_time,
                0,  # initial acceleration
                0,  # final acceleration
            ]).transpose()

        values_z = np.array(
            [
                z[i - 1],  # initial position
                z[i],  # final position
                0,  # initial velocity
                0,  # final velocity
                start_time,
                end_time,
                0,  # initial acceleration
                0,  # final acceleration
            ]).transpose()


        x_points.append(create_quintic_trajectory(values_x, set_points))
        y_points.append(create_quintic_trajectory(values_y, set_points))
        z_points.append(create_quintic_trajectory(values_z, set_points))

    final_values = []
    for x_l, y_l, z_l in zip(x_points, y_points, z_points):
        for x, y, z in zip(x_l, y_l, z_l):
            final_values.append((x, y, z))

    return final_values, [x_points, y_points, z_points]

def plot_quintic(initial_points, set_points=30):
    points, values = get_quintic_trajectory(initial_points, set_points=set_points)
    import matplotlib.pyplot as plt
    plt.plot(values[0][0], label="X values")
    plt.plot(values[1][0], label="Y values")
    plt.plot(values[2][0], label="Z values")
    plt.ylabel('Position')
    plt.xlabel('Time')
    plt.title("Quintic Trajectory Generator")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot_quintic(np.array([[0, 0, 0], [2, 5, 3]]))
    # print(values)

