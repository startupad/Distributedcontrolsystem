import json

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import numpy as np
import logging

import math
import matplotlib.pyplot as plt

from CoppeliaSim_project.config import TOLERANCE
from CoppeliaSim_project.main import load_processed_matrix, processed_matrix_path, find_value_coordinates, \
    create_straight_path

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class Tractor:
    def __init__(self, sim):

        self.sim = sim

        #set up Virtual point B simulation param
        self.b_distance =0.2
        self.point_b_handle = 0

        self.tractor_handle = self.load_tractor_model()

        # set up starting interpolation velocity
        self.velocity = 1

        self.starting_config = [0, 0, 0.3, 0.00013, -0.7, 0.00036, 0.7]
        self.sim.setObjectPosition(self.tractor_handle, self.starting_config, self.sim.handle_world)

        # set up path parameters
        self.path=[]
        self.config_to_reach = []
        self.posAlongPath = 0

        self.previousSimulationTime = 0


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

                #creating the point b
                self.point_b_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_spheroid, [0.1, 0.1, 0.1],0)
                self.sim.setObjectParent(self.point_b_handle, self.tractor_handle, False)
                self.sim.setObjectPosition(self.point_b_handle, [0,0,-self.b_distance], self.sim.handle_parent)
            return self.tractor_handle



    # Definiamo una funzione per calcolare una curva di Bezier quadratica
    # dati i punti di controllo po, p1, p2 e e un parametro t che varia da 0 a 1.
    def bezier_quadratic(self,t, p0, p1, p2):
        return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2

    def calculate_new_path(self, input_path):
        print("input path: ", input_path)
        # Convertiamo la lista in un array NumPy per facilitare le operazioni
        mypath = np.array(input_path)
        print("mypath: ", mypath)
        # Estraiamo le prime due colonne (i primi due valori di ogni sottoarray)
        primi_due_valori = mypath[:, :2]

        # Estraiamo i valori di x e y dalla matrice
        x = mypath[:, 0]
        y = mypath[:, 1]

        # Numero di punti di interpolazione (ad esempio, 10)
        n = 10
        interpolated_values =[]

        # Iteriamo sulle coppie consecutive di punti
        for i in range(len(x) - 1):
            # Definisci il punto di controllo (puoi modificarlo per ottenere diverse curve)
            p_control = (x[i] + x[i + 1]) / 2, (y[i] + y[i + 1]) / 2

            # Creiamo un array con i valori di t per l'interpolazione
            t = np.linspace(0, 1, n + 1)

            # Calcoliamo i punti sulla curva di Bezier
            x_intermedi = self.bezier_quadratic(t, x[i], p_control[0], x[i + 1])
            y_intermedi = self.bezier_quadratic(t, y[i], p_control[1], y[i + 1])
            interpolated_values.append(np.column_stack((x_intermedi, y_intermedi)))
        self.path= np.vstack(interpolated_values)

        #set up the first point to reach for our tractor
        self.config_to_reach = self.path[0]
        self.posAlongPath = 0

        # print(all_interpolated_values)
        # Estraiamo i valori x e y dai dati interpolati
        x_interpolated = self.path[:, 0]
        y_interpolated = self.path[:, 1]

        # Creiamo il grafico
        #plt.plot(x_interpolated, y_interpolated, '-o', label='Dati interpolati')

        # Aggiungiamo una legenda e dei titoli
        #plt.xlabel('X')
        #plt.ylabel('Y')
        #plt.title('Grafico dei dati interpolati')
        #plt.legend()

        plot_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'interpolated_path.png')
        plt.savefig(plot_file)
        print(f"Plot saved to {plot_file}")

        # Mostriamo il grafico
        #plt.show()


    def next_animation_step(self, t_step):
        # """Perform the next animation step."""

        #calculating point b_pos
        actual_pos = self.sim.getObjectPosition(self.point_b_handle, self.sim.handle_world)

        # Calcolo dell'angolo utilizzando il prodotto scalare
        cos_theta = np.dot(actual_pos[0:2], self.config_to_reach) / (np.linalg.norm(actual_pos[0:2]) * np.linalg.norm(self.config_to_reach))
        theta = np.arccos(cos_theta)

        # proportional control coefficient
        k = 0.3
        # virtual inputs
        vdx = k*(self.config_to_reach[0] - actual_pos[0])
        vdy = k*(self.config_to_reach[1] - actual_pos[1])

        linear_vel = vdx * math.cos(theta) + vdy * math.sin(theta)
        angular_vel = (vdy * math.cos(theta) - vdx * math.sin(theta)) / self.b_distance

        #calcolo delle velocità angolari da imporre alle singole ruote
        wheel_radius = 0.086 #[m]
        distance_between_wheels = 0.152 #[m]
        Wr = (linear_vel + (distance_between_wheels / 2) * angular_vel) / wheel_radius
        Wl = (linear_vel - (distance_between_wheels / 2) * angular_vel) / wheel_radius

        #applaying the calculated speed to the wheels
        leftJointHandle = self.sim.getObject(":/leftJoint_")
        rightJointHandle = self.sim.getObject(":/rightJoint_")
        self.sim.setJointTargetVelocity(leftJointHandle,Wl)
        self.sim.setJointTargetVelocity(rightJointHandle,Wr)


    def has_reached_target(self):

        """Check if the tractor has reached its target position."""
        # calculating point b_pos
        actual_pos = self.sim.getObjectPosition(self.point_b_handle, self.sim.handle_world)

        target_pos = self.config_to_reach

        # Check if the drone is close enough to the target position
        distance = np.linalg.norm(np.array(actual_pos[0:2]) - target_pos)
        #print("distance: ", distance)
        #print("tolerance: ", TOLERANCE)
        if distance < TOLERANCE:
            self.posAlongPath +=1
            self.config_to_reach = self.path[self.posAlongPath]
            if self.posAlongPath < len(self.path):
                return False
            else:
                return True
        else:
            return False

def run_tractor_simulation(path_to_follow):
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)

    sim.startSimulation()

    # create the tractor obj
    my_tract = Tractor(sim)

    prev_time = 0


    my_tract.calculate_new_path(path_to_follow)

    while my_tract.has_reached_target() == False:
        t_step = (sim.getSimulationTime() - prev_time)
        prev_time = t_step
        my_tract.next_animation_step(t_step)
        sim.step()

    sim.stopSimulation()

def save_path_to_file(path, file_path):
    """Salva il path su un file JSON."""
    with open(file_path, 'w') as f:
        json.dump(path, f)
    print(f"Path salvato su {file_path}")


def main_tractors():
    print("Simulazione trattori avviata.")
    matrix = load_processed_matrix(processed_matrix_path)
    coordinates = find_value_coordinates(matrix, 3)
    path = create_straight_path(coordinates)

    # Salva il path su un file
    path_file = os.path.join(os.path.dirname(__file__), 'path.json')
    save_path_to_file(path, path_file)
    #pippo = [[2,3],[5,4]]
    run_tractor_simulation(path_file)

if __name__ == "__main__":
    main_tractors()