import math
import os
from pprint import pprint
import multiprocessing as mp
import numpy as np
import matplotlib.pyplot as plt
import pickle


class DifferentialDriveRobot:
    def __init__(self, length, v_left, v_right, dt=0.1):
        self.length = length  # Distance between the wheels
        self.v_left = v_left  # Velocity of the left wheel
        self.v_right = v_right  # Velocity of the right wheel
        self.dt = dt
        self.x = 0
        self.y = 0
        self.theta = 0  # Direction of the robot
        self.total_distance = 0  # Total distance traveled

    def step(self):
        # Calculate the velocity of the robot and the angular velocity
        v = (self.v_right + self.v_left) / 2  # Linear velocity
        omega = (self.v_right - self.v_left) / self.length  # Angular velocity

        # Update the robot's orientation
        self.theta = self.theta + omega * self.dt

        # Update the robot's position
        new_x = self.x + v * np.cos(self.theta) * self.dt
        new_y = self.y + v * np.sin(self.theta) * self.dt

        # Calculate the distance traveled in this step
        distance = np.sqrt((new_x - self.x) ** 2 + (new_y - self.y) ** 2)
        self.total_distance += distance

        # Update position
        self.x = new_x
        self.y = new_y

        return self.x, self.y, self.theta

    def set_wheel_velocities(self, v_left, v_right):
        # Update the velocities of the wheels
        self.v_left = v_left
        self.v_right = v_right





def run_simulation(wheelbase, v_left, v_right, total_time):
    dt = 0.01
    robot = DifferentialDriveRobot(wheelbase, v_left, v_right, dt)
    times = np.arange(0, total_time, dt)

    for t in times:
        x, y, theta = robot.step()

    final_position = (robot.x, robot.y, np.rad2deg(robot.theta))
    total_distance = robot.total_distance
    total_displacement = np.sqrt(robot.x ** 2 + robot.y ** 2)
    return final_position, total_distance, total_displacement


def create_lookup_table(wheelbases, steering_angles, velocities, total_times):
    lookup_table = {}
    for wheelbase in wheelbases:
        for steering_angle in steering_angles:
            for velocity in velocities:
                for total_time in total_times:
                    result = run_simulation(wheelbase, steering_angle, velocity, total_time)
                    lookup_table[(wheelbase, steering_angle, velocity, total_time)] = result
    return lookup_table


def create_lookup_table_parallel_v2(wheelbases, vls, vrs, total_times):
    # Create a multiprocessing Pool object
    with mp.Pool(mp.cpu_count()) as pool:
        # Use a list comprehension to create a list of tasks
        tasks = [(wheelbase, vl, vr, total_time)
                 for wheelbase in wheelbases
                 for vl in vls
                 for vr in vrs
                 for total_time in total_times]

        # Run the tasks in the pool and get the results
        results = pool.starmap(run_simulation, tasks)

    # Combine the results and tasks into a dictionary
    # Here, we use the final position as the key and exclude the velocity from the value
    lookup_table = {result[0]: (task[1], task[2], result[1]) for task, result in zip(tasks, results)}
    return lookup_table



def find_nearest_key(lookup_table, position):
    nearest_key = None
    min_position_difference = float('inf')
    

    for key in lookup_table.keys():
        # Calculate the Euclidean distance between the positions
        position_difference = np.sqrt((key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2)

        # Check if this difference is the smallest we've seen so far
        if position_difference < min_position_difference:
            min_position_difference = position_difference
            nearest_key = key

    print(f"Minimum position difference: {min_position_difference}")
    return nearest_key


def plot_positions_orientations(lookup_table):
    plt.figure(figsize=(10, 10))
    for final_position, _ in lookup_table.items():
        x, y, theta = final_position
        # if abs(theta - 90) <= :
        plt.quiver(x, y, np.cos(np.deg2rad(theta)), np.sin(np.deg2rad(theta)), angles='xy', scale_units='xy',
                       scale=100)

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Final Positions and Orientations of DD in Each Simulation')
    plt.grid(True)
    plt.show()

for wheelbase in [1.542]:
    print(wheelbase)
    wheelbases = [wheelbase] # Example values
    # steering_angles = np.deg2rad(np.arange(-80, 81, 1.0))  # Angles from -90 to 90 degrees, in increments of 1 degree # Example values
    # vl = vr = [i / 20 for i in range(-20, 21)]
    # vl = vr = [i / 200 for i in range(-60, 241)]
    vl = vr = [i / 100 for i in range(-60, 121)]

    # vr = [0.1, 0.0, -0.1]
    # velocities = [0.2, -0.2]  # Example values
    # total_times = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Example values
    time_array = [i / 10 for i in range(2, 11)]
    print(time_array, vl)
    total_times = time_array

    lookup_table_parallel_v2 = create_lookup_table_parallel_v2(wheelbases, vl, vr, total_times)
    # plot_positions_orientations(lookup_table_parallel_v2)
    pprint(lookup_table_parallel_v2)

    # Store
    with open('mpc_lookup_table_dd_v2_'+str(wheelbase)+'.pkl', 'wb') as f:
        pickle.dump(lookup_table_parallel_v2, f)
    print('filesize',os.path.getsize('mpc_lookup_table_dd_'+str(wheelbase)+'.pkl') / (1024 * 1024))
