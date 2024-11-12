class VisualSensor:
    def __init__(self, sim):
        self.sim = sim
        self.bit_coded_options = [
            1,  # Sensor will be explicitly handled (data measured only when manually requested)
            1,  # Sensor is in perspective operative mode
            0,  # Sensed volume of space will be shown
            0,  # Reserved for CoppeliaSim
            0,  # Sensor uses external inputs (videos, photos, etc.)
            1,  # Sensor will use local lights
            1,  # Sensor will not render any fog
            0   # Sensor will not use a specific color for default background
        ]

        self.sensor_options = 0
        # Convert the bit-coded options to an integer value
        for i, bit in enumerate(self.bit_coded_options):
            self.sensor_options += bit * (2 ** i)

        # Sensor parameters (resolution, clipping planes, etc.)
        self.sensor_int_param = [2**4, 2**4, 0, 0]  # Sensor resolution (x, y) in pixels
        self.red_color = 0
        self.blue_color = 0
        self.green_color = 0
        self.sensor_float_param = [
            0.05,   # Near clipping plane (minimum detection distance in meters)
            5,      # Far clipping plane (maximum detection distance in meters)
            0.785398,  # View angle (rad)
            0.05,   # Size of the body-part of the sensor along x (m)
            0,      # Reserved for CoppeliaSim
            0,      # Reserved for CoppeliaSim
            self.red_color,   # Red value for null pixel
            self.green_color, # Green value for null pixel
            self.blue_color,  # Blue value for null pixel
            0,      # Reserved for CoppeliaSim
            0       # Reserved for CoppeliaSim
        ]

        self.handle_sensor = 0  # Sensor handle, initialized to 0

    def create_sensor(self):
        # Create the sensor and save its handle
        self.handle_sensor = self.sim.createVisionSensor(self.sensor_options, self.sensor_int_param, self.sensor_float_param)

    def read_sensor(self):
        # Read the state of the vision sensor and return the detected data
        detection_count, aux_packet, aux_packet2 = self.sim.handleVisionSensor(self.handle_sensor)

        # Check for sensor detection
        read_result, aux_packet, aux_packet2 = self.sim.handleVisionSensor(self.handle_sensor)
        if read_result == 1:
            print("Nothing has been detected.")
        elif read_result == -1:
            print("Sensor initialization error.")
        else:
            if aux_packet[11] >= 0.9 and aux_packet[12] <= 0.1 and aux_packet[13] <= 0.1:
                return 3  # red
            elif aux_packet[11] >= 0.9 and aux_packet[12] >= 0.9 and aux_packet[13] <= 0.1:
                return 2  # yellow
            elif aux_packet[11] <= 0.1 and aux_packet[12] >= 0.9 and aux_packet[13] <= 0.1:
                return 1  # green
            else:
                print("too strange color found: ")
                return aux_packet[11:14]
