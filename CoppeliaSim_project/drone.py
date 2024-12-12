import os
import numpy as np
import logging

from visual_sensor import VisualSensor


class Drone:
    def __init__(self, sim, drone_id, starting_config, wait_time=3.0):
        """Initialize the drone with its simulation environment, ID, starting configuration, and wait time."""
        self.sim = sim

        self.id = drone_id
        self.starting_config = starting_config
        self.velocity = 0.5

        self.posAlongPath = 0
        self.previousSimulationTime = 0
        self.wait_time = wait_time
        self.wait_start_time = None

        # Load the drone model
        self.handle_drone = self.load_drone_model()

        # Set the initial position and alias for the drone
        self.set_initial_position()
        self.sim.setObjectAlias(self.handle_drone, f"Drone_{self.id}")

        # Configure the visual sensor
        self.sensor = VisualSensor(self.sim)
        self.sensor.create_sensor()
        self.sim.setObjectQuaternion(self.sensor.handle_sensor, [1, 0, 0, 0], self.sim.handle_world)
        self.sim.setObjectParent(self.sensor.handle_sensor, self.handle_drone, False)

        # Set the target handle
        self.target_handle = self.sim.getObject(":/target", {'index': int(self.id) - 1, 'noError': True})
        self.sim.setObjectPosition(self.target_handle, -1, starting_config[0:3])

    def load_drone_model(self):
        """Load the drone model from the file."""
        base_path = os.path.dirname(os.path.abspath(__file__))
        path_drone = os.path.join(base_path, 'Quadcopter.ttm')

        if not os.path.exists(path_drone):
            logging.error(f"The file {path_drone} does not exist.")
            return -1
        else:
            handle_drone = self.sim.loadModel(path_drone)
            if handle_drone == -1:
                logging.error(f"Error loading model for Drone {self.id}: {handle_drone}")
            else:
                logging.info(f"Successfully loaded model for Drone {self.id}: {handle_drone}")
            return handle_drone

    def set_initial_position(self):
        """Set the initial position of the drone."""
        if self.handle_drone != -1:
            self.sim.setObjectPosition(self.handle_drone, self.starting_config[0:3], self.sim.handle_world)

    def get_position(self):
        """Get the current position of the drone."""
        return self.sim.getObjectPosition(self.handle_drone, self.sim.handle_world)

    def next_animation_step(self):
        """Perform the next animation step for the drone."""
        self.t = self.sim.getSimulationTime()
        if self.wait_start_time is not None:
            if self.t - self.wait_start_time < self.wait_time:
                return  # Wait until the wait time has passed
            else:
                self.wait_start_time = None  # Reset wait start time

        self.posAlongPath += self.velocity * (self.t - self.previousSimulationTime)

        config = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, self.posAlongPath)

        if config:
            if len(config) >= 3:
                self.sim.setObjectPosition(self.target_handle, config[0:3], self.sim.handle_world)
                if len(config) >= 7:
                    self.sim.setObjectQuaternion(self.target_handle, config[3:7], self.sim.handle_world)
                else:
                    logging.warning(f"Config does not contain enough elements for quaternion: {config}")
            else:
                logging.error(f"Config does not contain enough elements for position: {config}")
        else:
            logging.error("Config is None")

        self.previousSimulationTime = self.t

        if self.has_reached_target():
            self.wait_start_time = self.t  # Start the wait time

    def calculate_new_path(self, new_config):
        """Calculate a new path for the drone."""
        self.config_to_reach = new_config
        actual_pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        actual_orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        self.path = actual_pos + actual_orientation + self.config_to_reach[0:3] + self.config_to_reach[3:7]
        self.pathLengths, self.path_total_length = self.sim.getPathLengths(self.path, 7)
        self.posAlongPath = 0

    def set_target_position(self, position):
        """Set the target position for the drone."""
        self.sim.setObjectPosition(self.target_handle, -1, position)

    def read_sensor(self):
        return self.sensor.read_sensor()

    def get_drone_config_info(self):
        """Get the current position and orientation of the drone."""
        pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        return pos, orientation

    def has_reached_target(self):
        """Check if the drone has reached its target position."""
        current_pos = self.get_position()
        target_pos = self.config_to_reach[0:3]
        # Check if the drone is close enough to the target position
        tolerance = 0.75  # Define a tolerance for reaching the target

        distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        return distance < tolerance
