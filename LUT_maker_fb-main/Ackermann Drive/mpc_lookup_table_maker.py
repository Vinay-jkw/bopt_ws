import math
from pprint import pprint
import multiprocessing as mp
import numpy as np
import matplotlib.pyplot as plt
import pickle


class Car:
    def __init__(self, length, velocity, dt=0.1):
        self.length = length
        self.velocity = velocity
        self.dt = dt
        self.x = 0
        self.y = 0
        self.theta = 0  # Direction of car
        self.total_distance = 0  # Total distance traveled

    def step(self, steering_angle):
        # Update car state
        lv = self.velocity * np.cos(steering_angle)
        self.theta = self.theta + (self.velocity * np.sin(steering_angle) * self.dt) / self.length
        new_x = self.x + lv * np.cos(self.theta) * self.dt
        new_y = self.y + lv * np.sin(self.theta) * self.dt

        # Calculate the distance traveled in this step
        distance = np.sqrt((new_x - self.x) ** 2 + (new_y - self.y) ** 2)
        self.total_distance += distance
        # Update position
        self.x = new_x
        self.y = new_y

        return self.x, self.y, self.theta


def run_simulation(wheelbase, steering_angle, velocity, total_time):
    dt = 0.01
    car = Car(wheelbase, velocity, dt)
    times = np.arange(0, total_time, dt)
    for t in times:
        x, y, theta = car.step(steering_angle)

    final_position = (car.x, car.y, np.rad2deg(car.theta))
    total_distance = car.total_distance
    total_displacement = np.sqrt(car.x * car.x + car.y * car.y)
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


def create_lookup_table_parallel_v2(wheelbases, steering_angles, velocities, total_times):
    # Create a multiprocessing Pool object
    with mp.Pool(mp.cpu_count()) as pool:
        # Use a list comprehension to create a list of tasks
        tasks = [(wheelbase, steering_angle, velocity, total_time)
                 for wheelbase in wheelbases
                 for steering_angle in steering_angles
                 for velocity in velocities
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
                       scale=10)

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Final Positions and Orientations of Car in Each Simulation')
    plt.grid(True)
    plt.savefig('plot.png')
    plt.show()

for wheelbase in [1.542]:
    print(wheelbase)
    wheelbases = [wheelbase] # Example values
    steering_angles = np.deg2rad(np.arange(-90, 90, 1.0))  # Angles from -90 to 90 degrees, in increments of 1 degree # Example values
    velocities = [0.2, -0.2]  # Example values
    # total_times = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Example values
    time_array = [i / 10 for i in range(2, 151)]
    print(time_array)
    total_times = time_array

    lookup_table_parallel_v2 = create_lookup_table_parallel_v2(wheelbases, steering_angles, velocities, total_times)

    pprint(lookup_table_parallel_v2)
    plot_positions_orientations(lookup_table_parallel_v2)

    # Store
    with open('mpc_lookup_table_'+str(wheelbase)+'.pkl', 'wb') as f:
        pickle.dump(lookup_table_parallel_v2, f)
