import numpy as np


class FlyController:
    def __init__(self, sim, drones_list):
        self.sim = sim
        self.drones_list = drones_list
        self.n_drones = len(drones_list)

        # define the three basic matrices as zero matrices
        self.matrix_delta = np.zeros((self.n_drones, self.n_drones))
        self.matrix_adj = np.zeros((self.n_drones, self.n_drones))
        self.matrix_laplacian = np.zeros((self.n_drones, self.n_drones))

        # define the min-max distance in meters between the drones
        self.dist_max = 3
        self.dist_min = 0.5

        # define a matrix to store the inter-drone distances of our formation, by default all dist < dmax
        # --> we get a full connected graph
        self.matrix_interdrones_distance = np.full((self.n_drones, self.n_drones), (self.dist_max - 1))

    def update_matrices(self):

        # reset the three basic matrices at each iteration
        self.matrix_delta = np.zeros((self.n_drones, self.n_drones))
        self.matrix_adj = np.zeros((self.n_drones, self.n_drones))
        self.matrix_laplacian = np.zeros((self.n_drones, self.n_drones))

        ### this part is usefull only if we decide that our drones have limited communication range ###
        # define the adjacency matrix as matrix where the i-j th element is = 1 if i and j elements are neighbours
        for i in range(self.n_drones):
            for j in range(self.n_drones):
                # checks if the i-th drone is sufficiently near to the j drones
                if self.matrix_interdrones_distance[i][j] <= self.dist_max:
                    self.matrix_adj[i][j] = 1

        # define delta as matrix where the diagonal elements are = to the degree of node i
        # degree = number of connections of the node
        for i in range(self.n_drones):
            for j in range(self.n_drones):
                self.matrix_delta[i][i] += self.matrix_adj[i][j]

        # calculate the laplacian matrix using the formula L = delta -adj
        self.matrix_laplacian = np.subtract(self.matrix_delta, self.matrix_adj)

        return self.matrix_laplacian, self.matrix_delta

    def compute_drone_actual_config_matrix(self):
        ### we need to decide if we want to pick data from a central server or talking directly with other obj !!!!!!!!!!!!
        ### if we decide for the distributed approach, we must implement something to use the delta matrix to understand which are the near drones at which each drone can talk

        # we want the matrix in this form
        # mat = [
        #     [x1, y1, z1, q11, q12, q13, q14]
        #     [x2, y2, z2, q12, q23, q23, q24]
        #     [x3, y3, z3, q31, q32, q33, q34]
        #     ]

        # reset matrix at each iteration
        matrix_drone_config = []

        # iteratively ask each drone its actual config and save it as a matrix where each line contains a drone config
        for i in range(self.n_drones):
            pos, orientation = self.drones_list[i].get_drone_config_info()
            matrix_drone_config.append(pos + orientation)

        return matrix_drone_config

    def consensus_protocol(self, delta_t, var_to_sync: list):
        # il consensus MUST concern itself only around the calculus of the input var's changing ratio

        # var_to_sync MUST be a matrix [n_drones][7]
        z = var_to_sync

        # compute matrices for this time step
        lap, delta = self.update_matrices()

        # define z_t as my next step configuration
        z_dot = []

        # computing z_t as: z_t[i] = z[i] - delta_t * lap * z[i]
        z_dot = np.dot(lap, z)

        # round the values of z_t to their third decimal value, to prevent the over-usage of the consensus
        # and the consequential divergence due to approximation errors
        z_dot = np.round(z_dot, 3)

        return z_dot

    def rendezvous_protocol(self, delta_t, target_config: list[7]):
        # compute actual drones position as numpy array
        matrix_drone_config = np.array(self.compute_drone_actual_config_matrix())

        # print("drones config: ", np.round(matrix_drone_config,3))

        # compute the error between each drone config and the target config
        matrix_error = np.subtract(matrix_drone_config, target_config)

        # print("error: ", np.round(matrix_error,3))

        # computing the new target configs using the consensus protocol to compute the error rate
        new_drone_targets_config = np.round(
            np.subtract(matrix_drone_config, np.dot(delta_t, self.consensus_protocol(delta_t, matrix_error))), 3)

        # converting the numpy array into a normal one
        print("new config: ", new_drone_targets_config)
        return new_drone_targets_config.tolist()
