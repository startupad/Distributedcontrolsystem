import math
import Visual_sensor_script

class Drone:
    def __init__(self, sim, starting_config: list[7]):

        # simulation environment setup
        self.sim = sim
        self.t = 0
        self.previousSimulationTime = 0

        # drones variables setup
        self.velocity = 0.05  # m/s

        # path variables setup
        self.posAlongPath = 0
        self.path_total_length = 0  # ??? it's an output of a foundamental function bau what it supposed to be used for?  ???
        self.pathLengths = []
        self.path = []
        self.target_handle = 0
        self.config_to_reach = []
        self.target_pos = []

        # importing drone model
        path_drone = "models/robots/mobile/Quadcopter.ttm"
        self.handle_drone = sim.loadModel(path_drone)
        if self.handle_drone == -1:
            print("Error loading model: ", self.handle_drone)
        else:
            print("Successfully loaded model: ", self.handle_drone)
            self.sim.setObjectPosition(self.handle_drone, starting_config[0:3],  self.sim.handle_world)

        # importing Visual sensor model using my sensor class
        self.sensor = Visual_sensor_script.Visual_sens(self.sim)
        self.sensor.create_sensor()
        # make the sensor face the soil
        self.sim.setObjectQuaternion(self.sensor.handle_sensor, [1, 0, 0, 0], self.sim.handle_world)
        # attach sensor to drone
        self.sim.setObjectParent(self.sensor.handle_sensor, self.handle_drone, False)

    def next_animation_step(self):
        # this code must be run continuously to simulate the animation of the drones' movements!!
        self.t = self.sim.getSimulationTime()

        self.posAlongPath = self.posAlongPath + self.velocity * (self.t - self.previousSimulationTime)
        # calculate next intermediate config to reach the next point of the path starting from the point of the path where we are
        config = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, self.posAlongPath)

        # move the target to the next step
        self.sim.setObjectPosition(self.target_handle, config[0:3], self.sim.handle_world)
        self.sim.setObjectQuaternion(self.target_handle, config[3:7], self.sim.handle_world)

        # update time
        self.previousSimulationTime = self.t

    def calculate_new_path(self, new_config: list[7]):
        # save new objective configuration
        self.config_to_reach = new_config

        # get actual pos
        actual_pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)

        # get actual orientation
        actual_orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)

        # create a list containing the actual and the new point
        self.path = actual_pos + actual_orientation + self.config_to_reach[0:3] + self.config_to_reach[3:7]

        # calculate path length = distance between the origin of the path and the other path's points
        self.pathLengths, self.path_total_length = self.sim.getPathLengths(self.path, 7)

        # put the drone at the starting position of the new calculated path
        self.posAlongPath = 0

