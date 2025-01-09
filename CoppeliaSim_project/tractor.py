from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import numpy as np
import logging
from config import TOLERANCE

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class Tractor:
    def __init__(self, sim):

        self.sim = sim
        self.tractor_handle = self.load_tractor_model()

        self.velocity = 1

        self.starting_config = [0.5, 0.5, 0.4, 0.00013, -0.7, 0.00036, 0.7]
        self.sim.setObjectPosition(self.tractor_handle, self.starting_config, self.sim.handle_world)
        self.config_to_reach = []
        self.previousSimulationTime = 0

        self.posAlongPath = 0
        self.pathLengths = None
        self.path_total_length = None
        self.path = None

    def load_tractor_model(self):
        """Load the drone model from the file."""
        base_path = os.path.dirname(os.path.abspath(__file__))
        path_tract = os.path.join(base_path, 'dr12.ttm')

        if not os.path.exists(path_tract):
            logging.error(f"The file {path_tract} does not exist.")
            return -1
        else:
            self.tractor_handle = self.sim.loadModel(path_tract)
            if self.tractor_handle == -1:
                logging.error(f"Error loading model for the Tractor: {self.tractor_handle}")
            else:
                logging.info(f"Successfully loaded model for the Tractor: {self.tractor_handle}")
            return self.tractor_handle

    def next_animation_step(self, t_step):
        # """Perform the next animation step for the drone."""

        self.posAlongPath += self.velocity * t_step

        config = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, self.posAlongPath,
                                                    {'type': 'quadraticBezier', 'strength': 1.0, 'forceOpen': False})
        if config:
            if len(config) >= 3:
                self.sim.setObjectPosition(self.tractor_handle, config[0:3], self.sim.handle_world)
                if len(config) >= 7:
                    self.sim.setObjectQuaternion(self.tractor_handle, config[3:7], self.sim.handle_world)
                else:
                    logging.warning(f"Config does not contain enough elements for quaternion: {config}")
            else:
                logging.error(f"Config does not contain enough elements for position: {config}")
        else:
            logging.error("Config is None")

    def calculate_new_path(self, config_to_reach):
        """Calculate a new path for the drone."""
        actual_pos = self.sim.getObjectPosition(self.tractor_handle, self.sim.handle_world)
        actual_orientation = self.sim.getObjectQuaternion(self.tractor_handle, self.sim.handle_world)
        self.config_to_reach = config_to_reach
        self.path = actual_pos + actual_orientation + self.config_to_reach
        self.pathLengths, self.path_total_length = self.sim.getPathLengths(self.path, 7)
        self.posAlongPath = 0

    #  def unicycle_model(self): 
    #     theta = 0
    #     lin_vel = 1
    #     angular_vel = 2
    #     r= 0.5
    #     matrix_point_vel = np.array([[lin_vel* cos(theta), - r* sin(theta) * angular_vel ],
    #                                   [lin_vel* sin(theta), r* cos(theta) * angular_vel]])

    def has_reached_target(self):

        """Check if the tractor has reached its target position."""
        current_pos = self.sim.getObjectPosition(self.tractor_handle, self.sim.handle_world)
        target_pos = self.config_to_reach[0:3]

        # Check if the drone is close enough to the target position
        distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
        print("distance: ", distance)
        print("tolerance: ", TOLERANCE)
        if distance < TOLERANCE:
            return True
        else:
            return False

def run_tractor_simulation():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)

    sim.startSimulation()

    # create the tractor obj
    my_tract = Tractor(sim)

    prev_time = 0

    mypath = [[1, 1, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [2, 1, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [3, 1, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [3, 2, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [3, 3, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [2, 3, 0.4, 0.00013, -0.7, 0.00036, 0.7],
                  [0.1, 5, 0.4, 0.00013, -0.7, 0.00036, 0.7]]
    path_to_follow = mypath

    for j in range(len(path_to_follow)):
        my_tract.calculate_new_path(path_to_follow[j])

        has_reached_target = my_tract.has_reached_target()
        while has_reached_target == False:
            t_step = (sim.getSimulationTime() - prev_time)
            prev_time = t_step
            my_tract.next_animation_step(t_step)
            sim.step()
            has_reached_target = my_tract.has_reached_target()

    sim.stopSimulation()

if __name__ == "__main__":
    run_tractor_simulation()
