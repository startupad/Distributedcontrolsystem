import os
import numpy as np


class Drone:
    def __init__(self, sim, id, starting_config):
        self.sim = sim
        self.id = id
        self.starting_config = starting_config
        self.velocity = 0.5
        self.posAlongPath = 0
        self.previousSimulationTime = 0

        # Get the base path of the current file
        base_path = os.path.dirname(os.path.abspath(__file__))

        # Construct the full path to the model
        path_drone = os.path.join(base_path, 'Quadcopter.ttm')

        # Check if the file exists
        if not os.path.exists(path_drone):
            print(f"Error: the file {path_drone} does not exist.")
        else:
            self.handle_drone = self.sim.loadModel(path_drone)

        # Check if the model was loaded successfully
        if self.handle_drone == -1:
            print(f"Error loading model for Drone {self.id}: ", self.handle_drone)
        else:
            print(f"Successfully loaded model for Drone {self.id}: ", self.handle_drone)
            self.sim.setObjectPosition(self.handle_drone, starting_config[0:3], self.sim.handle_world)

        # Set an alias for the drone
        self.sim.setObjectAlias(self.handle_drone, f"Drone_{self.id}")

        # Configure the visual sensor
        self.sensor = VisualSensor(self.sim)
        self.sensor.create_sensor()
        self.sim.setObjectQuaternion(self.sensor.handle_sensor, [1, 0, 0, 0], self.sim.handle_world)
        self.sim.setObjectParent(self.sensor.handle_sensor, self.handle_drone, False)

        # Set the target
        self.target_handle = self.sim.getObject(":/target", {'index': int(self.id) - 1, 'noError': True})
        self.sim.setObjectPosition(self.target_handle, -1, starting_config[0:3])

    def get_position(self):
        # Get the current position of the drone
        return self.sim.getObjectPosition(self.handle_drone, self.sim.handle_world)

    def next_animation_step(self):
        # Update the simulation time
        self.t = self.sim.getSimulationTime()
        self.posAlongPath += self.velocity * (self.t - self.previousSimulationTime)
        config = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, self.posAlongPath)

        # Check if the configuration is valid
        if config:
            if len(config) >= 3:
                self.sim.setObjectPosition(self.target_handle, config[0:3], self.sim.handle_world)
                if len(config) >= 7:
                    self.sim.setObjectQuaternion(self.target_handle, config[3:7], self.sim.handle_world)
                else:
                    print(f"Warning: config does not contain enough elements for quaternion: {config}")
            else:
                print(f"Error: config does not contain enough elements for position: {config}")
        else:
            print("Error: config is None")

        self.previousSimulationTime = self.t

    def calculate_new_path(self, new_config):
        # Calculate a new path for the drone
        self.config_to_reach = new_config
        actual_pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        actual_orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        self.path = actual_pos + actual_orientation + self.config_to_reach[0:3] + self.config_to_reach[3:7]
        self.pathLengths, self.path_total_length = self.sim.getPathLengths(self.path, 7)
        self.posAlongPath = 0

    def set_target_position(self, position):
        # Set the target position for the drone
        self.sim.setObjectPosition(self.target_handle, -1, position)

    def read_sensor(self):
        # Read data from the visual sensor
        return self.sensor.read_sensor()

    def get_drone_config_info(self):
        # Get the current position and orientation of the drone
        pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        return pos, orientation

    def has_reached_target(self):
        # Get the current position of the drone
        current_pos = self.get_position()
        target_pos = self.config_to_reach[0:3]

        # Check if the drone is close enough to the target position
        tolerance = 1  # Define a tolerance for reaching the target
        distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        return distance < tolerance
