from colormath.color_objects import sRGBColor, HSVColor
from colormath.color_conversions import convert_color


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
            0  # Sensor will not use a specific color for default background
        ]

        self.sensor_options = 0
        # Convert the bit-coded options to an integer value
        for i, bit in enumerate(self.bit_coded_options):
            self.sensor_options += bit * (2 ** i)

        # Sensor parameters (resolution, clipping planes, etc.)
        self.sensor_int_param = [2 ** 4, 2 ** 4, 0, 0]  # Sensor resolution (x, y) in pixels
        self.red_color = 0
        self.blue_color = 0
        self.green_color = 0
        self.sensor_float_param = [
            0.05,  # Near clipping plane (minimum detection distance in meters)
            5,  # Far clipping plane (maximum detection distance in meters)
            0.785398,  # View angle (rad)
            0.05,  # Size of the body-part of the sensor along x (m)
            0,  # Reserved for CoppeliaSim
            0,  # Reserved for CoppeliaSim
            self.red_color,  # Red value for null pixel
            self.green_color,  # Green value for null pixel
            self.blue_color,  # Blue value for null pixel
            0,  # Reserved for CoppeliaSim
            0  # Reserved for CoppeliaSim
        ]

        self.handle_sensor = 0  # Sensor handle, initialized to 0

    def create_sensor(self):
        # Create the sensor and save its handle
        self.handle_sensor = self.sim.createVisionSensor(self.sensor_options, self.sensor_int_param,
                                                         self.sensor_float_param)

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

            # Convert RGB to HSV
            # Hue Saturation Brightness/Value = tonalità saturazione luminosità/valore
            rgb_color = sRGBColor(aux_packet[11], aux_packet[12], aux_packet[13])
            hsv_color = convert_color(rgb_color, HSVColor)

            # the type color read depend only on Hue, so if the drone takes a picture with multiple colors, then the
            # resulting color's hue is a weighted average of the hue of the colors captured

            # the color intervals have been chosen such that if a read color hue is more than 50% similar to the one
            # of the basic color than the related if-statement triggers itself

            # light green = HSVColor (hsv_h:102.6000 hsv_s:0.6803 hsv_v:0.5765)
            # dark green = HSVColor (hsv_h:102.1782 hsv_s:0.4856 hsv_v:0.8157)
            if 93 <= hsv_color.hsv_h <= 110:
                return 1
            # yellow = HSVColor(hsv_h: 83.7736 hsv_s: 0.4953 hsv_v: 0.8392)
            elif 67 <= hsv_color.hsv_h <= 93:
                return 2
            # brown= HSVColor( hsv_h: 52.7273 hsv_s: 0.7059 hsv_v: 0.7333)
            elif 37 <= hsv_color.hsv_h <= 67:
                return 3
            else:
                print("too strange color found: ")
                return aux_packet[11:14]
