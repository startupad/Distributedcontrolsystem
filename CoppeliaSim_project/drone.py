from visual_sensor import VisualSensor


class Drone:
    def __init__(self, sim, id, starting_config):
        # Setup iniziale
        self.sim = sim
        self.id = id
        self.velocity = 0.05  # Velocità in m/s
        self.t = 0
        self.previousSimulationTime = 0
        self.posAlongPath = 0
        self.path_total_length = 0
        self.pathLengths = []
        self.path = []
        self.config_to_reach = []

        # Caricamento del modello del drone
        path_drone = "models/robots/mobile/Quadcopter.ttm"
        self.handle_drone = self.sim.loadModel(path_drone)
        if self.handle_drone == -1:
            print(f"Error loading model for Drone {self.id}: ", self.handle_drone)
        else:
            print(f"Successfully loaded model for Drone {self.id}: ", self.handle_drone)
            self.sim.setObjectPosition(self.handle_drone, starting_config[0:3], self.sim.handle_world)

        # Imposta un alias per il drone
        self.sim.setObjectAlias(self.handle_drone, f"Drone_{self.id}")

        # Configurazione del sensore visivo
        self.sensor = VisualSensor(self.sim)
        self.sensor.create_sensor()
        self.sim.setObjectQuaternion(self.sensor.handle_sensor, [1, 0, 0, 0], self.sim.handle_world)
        self.sim.setObjectParent(self.sensor.handle_sensor, self.handle_drone, False)

        # Imposta il target
        self.target_handle = self.sim.getObject(":/target", {'index': int(self.id) - 1, 'noError': True})
        self.sim.setObjectPosition(self.target_handle, -1, starting_config[0:3])

    def get_position(self):
        return self.sim.getObjectPosition(self.handle_drone, self.sim.handle_world)

    def next_animation_step(self):
        self.t = self.sim.getSimulationTime()
        self.posAlongPath += self.velocity * (self.t - self.previousSimulationTime)
        # print("path: ", self.path)
        # print("pos_along: ", self.posAlongPath)
        config = self.sim.getPathInterpolatedConfig(self.path, self.pathLengths, self.posAlongPath)
        # print("config: ", config)
        if config:
            self.sim.setObjectPosition(self.target_handle, config[0:3], self.sim.handle_world)
            self.sim.setObjectQuaternion(self.target_handle, config[3:7], self.sim.handle_world)
        self.previousSimulationTime = self.t

    def calculate_new_path(self, new_config):
        self.config_to_reach = new_config
        actual_pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        actual_orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        self.path = actual_pos + actual_orientation + self.config_to_reach[0:3] + self.config_to_reach[3:7]
        self.pathLengths, self.path_total_length = self.sim.getPathLengths(self.path, 7)
        self.posAlongPath = 0

    def set_target_position(self, position):
        self.sim.setObjectPosition(self.target_handle, -1, position)

    def read_sensor(self):
        # Legge i dati dal sensore visivo
        return self.sensor.read_sensor()

    def get_drone_config_info(self):
        pos = self.sim.getObjectPosition(self.target_handle, self.sim.handle_world)
        orientation = self.sim.getObjectQuaternion(self.target_handle, self.sim.handle_world)
        return pos, orientation
