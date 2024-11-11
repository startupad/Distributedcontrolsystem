class Visual_sens: 
    def __init__ (self,sim):
        # the sysytem param = sys is a unique param of each coppelia scene --> i need to pass the sys param of my scene to the code
        self.sim =sim
        self.bit_coded_options = [ 1 , #sensor will be eplicited handled = it will measure data only when they will be manually requested
                            1, #the sensor is in perspective operative mode --> it has fixed angle of view and more aways is from obj more things it views
                            0, #sensed volume of space will be shown
                            0, #reserved to coppelia --> must be  = 0
                            0, #sensor use external inputs (videos, photo, etc...)
                            1, #sensor will use local lights
                            1, #sensor will not render any fog
                            0] #sensor will not use a specific color for default background 

        self.sensor_options = 0
        #converted the wanted sensor configuration from bit code to int value
        for i, bit in enumerate(self.bit_coded_options):
            self.sensor_options += bit * (2 ** i)
        
        self.sensor_int_param =[2**4, #sensor resolution x [pixel]
                                2**4, #sensor resolution y [pixel]
                                0,  #reserved to coppelia --> must be  = 0
                                0 ] #reserved to coppelia --> must be  = 0
        
        self.red_color=0
        self.blue_color= 0
        self.green_color=0
        self.sensor_float_param =[ 0.05, # near clipping plane = min detection distance [m]
                              5,   # far clipping plane = max detection distance [m]
                              0.785398,  # view angle / ortho view size [rad]
                              0.05, # size of the body-part of the vision sensor along x [m]
                              0,   # reserved to coppelia --> must be  = 0
                              0,   # reserved to coppelia --> must be  = 0
                              self.red_color,    # null pixel red-value
                              self.green_color,  # null pixel green-value
                              self.blue_color,   # null pixel blue-value
                              0,   # reserved to coppelia --> must be  = 0
                              0]   # reserved to coppelia --> must be  = 0

        self.handle_sensor = 0  #prepare the sensor handle

    def create_sensor(self):
        #create the sensor and save its handle value
        self.handle_sensor = self.sim.createVisionSensor(self.sensor_options,self.sensor_int_param,self.sensor_float_param)

    def read_sensor(self):

        # detection_count= number of detections (number of vision sensors that triggered a detection)
        #
        # aux_packet = default auxiliary packet = list of 15 auxiliary values:
        #       -the minimum of [intensity red green blue depth] = [0,1,2,3,4]
        #       -the maximum of [intensity red green blue depth] = [5,6,7,8,9,]
        #       -and the average of [intensity red green blue depth] = [10,11,12,13,14]
        #
        # aux_packet2 = additional auxiliary values from an image processing component)
        detection_count, aux_packet, aux_packet2 = self.sim.handleVisionSensor(self.handle_sensor)

        # Reads the state of a vision sensor. This function doesn't perform detection, it merely reads the result from a previous call to sim.handleVisionSensor
        # read_result = detection = bool (0 or 1), or -1 if the handle isn't initialized
        read_result, aux_packet, aux_packet2= self.sim.handleVisionSensor(self.handle_sensor)
        if read_result == 1:
            print("nothing has been detected")
        elif read_result == -1:
            print("sensor initialization error")
        else:
            return aux_packet