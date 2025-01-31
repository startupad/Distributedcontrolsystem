import numpy as np
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class FlyController:
    def __init__(self, sim, drones_list):
        self.matrix_norm = None
        self.matrix_interdrones_distance = None
        self.matrix_laplacian = None
        self.matrix_adj = None
        self.matrix_delta = None
        self.matrix_drone_config = None
        self.sim = sim
        self.drones_list = drones_list
        self.n_drones = len(drones_list)
        self.dist_max = 17

        self.initialize_matrices()

    def initialize_matrices(self):
        """Initialize matrices used in the controller."""
        self.matrix_delta = np.zeros((self.n_drones, self.n_drones))
        self.matrix_adj = np.zeros((self.n_drones, self.n_drones))
        self.matrix_laplacian = np.zeros((self.n_drones, self.n_drones))
        self.matrix_interdrones_distance = np.zeros((self.n_drones, self.n_drones))
        self.matrix_norm = np.zeros((self.n_drones, self.n_drones))
        self.matrix_drone_config = np.zeros((self.n_drones, self.n_drones))

    def update_matrices(self, type_of_algorithm):
        """Update the matrices based on the type of algorithm."""
        self.reset_matrices()
        self.compute_adjacency_matrix(type_of_algorithm)
        self.compute_delta_matrix()
        self.compute_laplacian_matrix()

    def reset_matrices(self):
        """Reset the matrices to zero."""
        self.matrix_delta.fill(0)
        self.matrix_adj.fill(0)
        self.matrix_laplacian.fill(0)

    def compute_adjacency_matrix(self, type_of_algorithm):
        """Compute the adjacency matrix based on the algorithm type."""
        self.matrix_drone_config = self.compute_drone_actual_config_matrix()
        for i in range(self.n_drones):
            for j in range(self.n_drones):
                if i != j and self.matrix_interdrones_distance[i][j] <= self.dist_max:
                    if type_of_algorithm == 'c':
                        self.matrix_adj[i, j] = 1
                    elif type_of_algorithm == 'f':
                        self.matrix_norm[i, j] = np.linalg.norm(
                            self.matrix_drone_config[i] - self.matrix_drone_config[j])
                        self.matrix_adj[i, j] = (self.matrix_norm[i, j] - self.matrix_interdrones_distance[i, j]) / \
                                                self.matrix_norm[i, j]

    def compute_delta_matrix(self):
        """Compute the delta matrix."""
        for i in range(self.n_drones):
            self.matrix_delta[i, i] = np.sum(self.matrix_adj[i])

    def compute_laplacian_matrix(self):
        """Compute the Laplacian matrix."""
        self.matrix_laplacian = self.matrix_delta - self.matrix_adj

        # scale the laplacian matriz to have its eigenvalues between 2 desired values
        # convergence speed is directly proportional to the min eigenvalue value
        min_eig = -0.1 * 10 ** (-5)
        max_eig = -min_eig
        matrix_lap_scaled = (self.matrix_laplacian - ((max_eig - min_eig) / 2 * np.identity(3))) / (
                    (max_eig - min_eig) / 2)
        self.matrix_laplacian = matrix_lap_scaled

    def compute_drone_actual_config_matrix(self):
        """Compute the actual configuration matrix of the drones."""
        matrix_drone_config = []
        for i in range(self.n_drones):
            pos, orientation = self.drones_list[i].get_drone_config_info()
            # Ensure pos and orientation are lists of the same length
            if len(pos) == 3 and len(orientation) == 4:
                matrix_drone_config.append(pos + orientation)
            else:
                logging.error(f"Drone {i} returned invalid config: pos={pos}, orientation={orientation}")
        return np.array(matrix_drone_config)

    def consensus_protocol(self, var_to_sync):
        """Perform the consensus protocol."""
        self.update_matrices('c')
        z_dot = np.dot(self.matrix_laplacian, var_to_sync)
        return np.round(z_dot, 3)

    def rendezvous_protocol(self, delta_t):
        """Perform the rendezvous protocol."""
        matrix_drone_config = self.compute_drone_actual_config_matrix()
        self.update_matrices('r')
        rate = np.dot(delta_t, self.consensus_protocol(matrix_drone_config))
        new_drone_targets_config = np.subtract(matrix_drone_config, rate)
        return np.round(new_drone_targets_config, 3)

    def formation_control(self, delta_t, interdrones_distances, tolerance):
        """Perform formation control."""
        self.matrix_drone_config = self.compute_drone_actual_config_matrix()

        """ formation control limited to the (X,Y) plane """
        # extract the height of flight from the drone leader
        z_drone_leader = self.matrix_drone_config[0, 2]
        # Slicing the matrix, selecting all rows and all columns starting from the third column (excluding X and Y)
        extra = self.matrix_drone_config[:, 2:]
        
        extra[:, 0] = z_drone_leader
        # print('colonna Z',extra[:, 0])
        # Slicing the drone matrix at the first 2 columns such that the formation can work only on the plane X,Y
        self.matrix_drone_config = self.matrix_drone_config[:, :2]

        self.matrix_interdrones_distance = interdrones_distances
        self.update_matrices('f')

        diff = np.linalg.norm(self.matrix_interdrones_distance - self.matrix_norm)
        if diff < tolerance:
            # logging.info("Convergence has been already reached")
            # logging.info(f"Actual inter-drones distances = norm matrix: {self.matrix_norm}")

            # recomposing the drone matrix
            self.matrix_drone_config = np.concatenate((self.matrix_drone_config, extra), axis=1)

            return np.round(self.matrix_drone_config, 5).tolist()
        else:
            # logging.info(f"Lap: \n {self.matrix_laplacian}")
            # logging.info(f"Norm matrix: \n {self.matrix_norm}")
            rate = np.dot(delta_t, np.dot(self.matrix_laplacian, self.matrix_drone_config))
            new_drone_targets_config = np.subtract(self.matrix_drone_config, rate)

            # recomposing the drone matrix with the value computed by the formation algorithm
            new_drone_targets_config = np.concatenate((new_drone_targets_config, extra), axis=1)

            return np.round(new_drone_targets_config, 5).tolist()

    def get_drones_positions(self):
        """Get the positions of all drones."""
        positions = []
        for i in range(self.n_drones):
            pos, orientation = self.drones_list[i].get_drone_config_info()
            if len(pos) == 3:
                positions.append(pos)
            else:
                logging.error(f"Drone {i} returned invalid position: {pos}")
        return np.array(positions)
