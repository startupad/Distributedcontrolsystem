import numpy as np
import math


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

        # define a matrix to store the inter-drone distances of our formation, by default all dist < dmax
        # --> we get a full connected graph
        self.matrix_interdrones_distance = np.zeros((self.n_drones, self.n_drones))

        # define a matrix where to store the actual drone positions info
        self.matrix_drone_config = np.zeros((self.n_drones, self.n_drones))

    def update_matrices(self, type_of_algorthm):

        # reset the three basic matrices at each iteration
        self.matrix_delta = np.zeros((self.n_drones, self.n_drones))
        self.matrix_adj = np.zeros((self.n_drones, self.n_drones))
        self.matrix_laplacian = np.zeros((self.n_drones, self.n_drones))

        ### this part is usefull only if we decide that our drones have limited communication range ###

        for i in range(self.n_drones):
            for j in range(self.n_drones):
                # fill only the diagonal elements
                if i != j:
                    # checks if the i-th drone is sufficiently near to the j drones
                    if self.matrix_interdrones_distance[i][j] <= self.dist_max:
                        norm = np.zeros((3, 3))
                        self.matrix_drone_config = np.array(self.compute_drone_actual_config_matrix())
                        # check what type of protocol we are using and prepare the respective adj matrix
                        if type_of_algorthm == 'c':
                            # for the consensus, we want a matrix where the i-j th element is = 1 if i and j elements are neighbours
                            self.matrix_adj[i, j] = 1
                        elif type_of_algorthm == 'r':
                            pass
                        elif type_of_algorthm == 'f':
                            # for the formation, we want a matrix where each element integrate the Connectivity algorithm
                            norm[i, j] = np.linalg.norm(
                                np.subtract(self.matrix_drone_config[i], self.matrix_drone_config[j]))
                            self.matrix_adj[i, j] = (1 - self.matrix_interdrones_distance[i, j] / norm[i, j]) / pow(
                                (self.dist_max - norm[i, j]), 3)

        # define delta as matrix where the diagonal elements are = to the degree of node i
        # degree = number of connections of the node = sum of the elements in each row of the adjacency matrix
        for i in range(self.n_drones):
            for j in range(self.n_drones):
                self.matrix_delta[i][i] += self.matrix_adj[i][j]

        # calculate the laplacian matrix using the formula L = delta -adj
        self.matrix_laplacian = np.subtract(self.matrix_delta, self.matrix_adj)

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

        # converting to np array
        return np.array(matrix_drone_config)

    def consensus_protocol(self, var_to_sync):
        # il consensus MUST concern itself only around the calculus of the input var's changing ratio

        # var_to_sync MUST be a matrix [n_drones][7]
        z = var_to_sync

        # compute matrices for this time step
        self.update_matrices('c')

        # define z_dot as my next step configuration
        z_dot = np.dot(self.matrix_laplacian, z)

        # round the values of z_t to their third decimal value, to prevent the over-usage of the consensus
        # and the consequential divergence due to approximation errors
        z_dot = np.round(z_dot, 3)

        return z_dot

    def rendezvous_protocol(self, delta_t, target_config: list[7]):
        # compute actual drones position as numpy array
        matrix_drone_config = np.array(self.compute_drone_actual_config_matrix())

        self.update_matrices('r')

        # computing the new target exploiting the consensus protocol theory to compute the variation rate

        rate = np.dot(delta_t, self.consensus_protocol(matrix_drone_config))

        new_drone_targets_config = np.subtract(matrix_drone_config, rate)

        # rounding the results to avoid approximation errors
        new_drone_targets_config = np.round(new_drone_targets_config, 3)

        return new_drone_targets_config

    def formation_control(self, delta_t, interdrones_distances):

        self.matrix_interdrones_distance = interdrones_distances
        self.update_matrices('f')
        self.matrix_drone_config = self.compute_drone_actual_config_matrix()

        # print("dist: ", self.matrix_interdrones_distance)
        print("drones: ", np.round(self.matrix_drone_config, 3))
        # print("lap: ", self.matrix_laplacian)
        # print("t: ", delta_t)

        # computing the new correction rate exploiting the consensus protocol theory
        rate = np.dot(delta_t, np.dot(self.matrix_laplacian, self.matrix_drone_config))
        print("rate = ", np.round(rate, 3))

        new_drone_targets_config = np.subtract(self.matrix_drone_config, rate)

        # converting the numpy array into a normal one and cut the result to the third decimal unit
        new_drone_targets_config = np.round(new_drone_targets_config, 3).tolist()

        # print("new config: ", new_drone_targets_config)
        return new_drone_targets_config
