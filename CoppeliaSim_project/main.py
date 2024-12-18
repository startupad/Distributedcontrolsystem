import logging
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from tessellation import apply_tessellation
from drone import Drone
from terrain import Terrain
from fly_controller import FlyController
from config import TOLERANCE, GRID_SIZE, N_DRONES

import sys
import os

# Aggiungi il percorso della cartella 'web-app' a sys.path
web_app_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web-app/'))
sys.path.append(web_app_path)

# Ora puoi importare 'app' dalla cartella 'web-app'
from api import save_matrix_processed, set_simulation_end, set_coordinates, get_priority_matrix

# Percorso del file processed_matrices.json
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # La cartella 'web-app'
FILE_PATH_PROCESSED = os.path.join(BASE_DIR, '..', 'data', 'processed_matrices.json')
FILE_PATH_PROCESSED = os.path.abspath(FILE_PATH_PROCESSED)  # Assicurati che il percorso sia assoluto

FILE_PATH = '../data/matrices.json'

grid = [[0 for _ in range(6)] for _ in range(6)]  # Creazione della griglia 6x6

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


def initialize_simulation():
    """Initialize the simulation client and start the simulation."""
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()
    logging.info("Simulation started")
    return sim


def create_s_path(centers, width):
    """Create an S-shaped path from the grid centers."""
    s_path = []
    for i in range(0, len(centers), width):
        row = centers[i:i + width]
        if (i // width) % 2 == 1:
            row.reverse()
        s_path.extend(row)
    return s_path



def initialize_drones(sim, n_drones):
    """Initialize drones with their starting configurations."""
    drones = []
    initial_config = [
        [1.0, 1.0, 0.487],
        [0.25, 0.25, 0.487],
        [1.75, 0.25, 0.487]
    ]

    for i in range(n_drones):
        drone = Drone(sim, drone_id=str(i + 1), starting_config=initial_config[i])
        drones.append(drone)
    return drones


def run_simulation(sim, s_path, drones, fc):
    prev_time = 0
    global grid  # Use the global grid variable
    grid_size = 6  # grid size (6x6)
    index = 0  # Index for iterating over s_path

    # Simulation loop
    for center in s_path:
        if index >= grid_size * grid_size:  # If the index exceeds the grid size
            print("Warning: s_path contains more centers than can be accommodated in the grid.")
            break  # Stop if there are too many centers for the grid

        # Calculate the average sensor value for the 3 drones
        total_sensor_value = 0
        # for i in range(3):  # For each drone
        #     sensor_value = drones[i].read_sensor()  # Read the sensor value
        #     print(f"Drone {i + 1} sensor value: {sensor_value}")
        #     total_sensor_value += sensor_value
        # average_sensor_value = total_sensor_value / 3
        sensor_value = drones[0].read_sensor()  # Leggi il valore
        print(f"Drone 1 sensor value: {sensor_value}")

        # Arrotonda il valore a 1, 2 o 3
        if sensor_value < 1.5:
            rounded_value = 1
        elif sensor_value < 2.5:
            rounded_value = 2
        else:
            rounded_value = 3

        # Calcola le coordinate nella griglia
        row = index // grid_size  # Riga
        col = index % grid_size  # Colonna

        # Inserisci il valore arrotondato nella griglia
        grid[row][col] = rounded_value

        index += 1
        # Set up formation control parameters
        desired_dist_matrix = np.array([[0, 0.5, 0.5], [0.5, 0, 1], [0.5, 1, 0]])

        # Set a new target for the drone leader
        drones[0].calculate_new_path(center)
        
        drone_reached = False
        while not drone_reached:
            drone_reached = True

            # standard coppelia sim frame steps are 50ms long, but we need it to be at most 1.5ms
            step = (sim.getSimulationTime() - prev_time) / 34
            #logging.info(f"step size:  \n {step}")

            if step > 0.0015:
                step = 0.0015
            prev_time = sim.getSimulationTime()

            # update animation for the master
            drones[0].next_animation_step(step)

            # I need to run the formation control multiple times to guarantee the convergence at the desired distances.
            # So, I need to divide each animation step in subintervals where I launch the fc to adjust slaves positions
            # theoretical best sub_divider = 50 (by optimization test)
            sub_divider = 10
            for p in range(sub_divider):
                # Compute formation control
                sub_step = step / sub_divider * p
                out = fc.formation_control(sub_step, desired_dist_matrix, TOLERANCE)

                # set new target for the slave drones
                drones[1].calculate_new_path(out[1])
                drones[2].calculate_new_path(out[2])

                # update animation frame for slave robot
                drones[1].next_animation_step(sub_step)
                drones[2].next_animation_step(sub_step)
                            
                target_coordinate_1 = [float(coord) for coord in center[:3]]
                target_coordinate_2 = [float(coord) for coord in out[1][:3]]
                target_coordinate_3 = [float(coord) for coord in out[2][:3]]

                # Check if the values of target_coordinate_2 and target_coordinate_3 are within ±1 of target_coordinate_1
                if all(abs(t2 - t1) <= 0.5 for t1, t2 in zip(target_coordinate_1, target_coordinate_2)) and \
                all(abs(t3 - t1) <= 0.5 for t1, t3 in zip(target_coordinate_1, target_coordinate_3)):
                    # If the condition is met, use the given target coordinates
                    set_coordinates(target_coordinate_1, target_coordinate_2, target_coordinate_3)
                else:
                    # If not, adjust target_coordinate_2 and target_coordinate_3 to target_coordinate_1 ± 1
                    adjusted_target_coordinate_2 = [t1 - 0.5 for t1 in target_coordinate_1]
                    adjusted_target_coordinate_3 = [t1 + 0.5 for t1 in target_coordinate_1]
                    
                    # Call set_coordinates with adjusted coordinates
                    set_coordinates(target_coordinate_1, adjusted_target_coordinate_2, adjusted_target_coordinate_3)
            if not drones[0].has_reached_target():
                drone_reached = False
            sim.step()
        

    for i in range(grid_size):
        if i % 2 == 1:  # Se la riga è dispari
            grid[i].reverse()
    # Save the processed matrix
    print("Griglia finale:")
    for row in grid:
        print(row)
    save_matrix_processed(FILE_PATH_PROCESSED, grid)
    set_simulation_end(True)


def main():
    try:
        sim = initialize_simulation()

        priority_matrix = get_priority_matrix(FILE_PATH) 

        terrain = Terrain(sim)
        tessellation_regular, tessellation_voronoi = apply_tessellation(terrain, priority_matrix)

        # Decide which type of tassellation to use
        tessellation = tessellation_regular

        width = terrain.get_dimensions()[0]
        s_path = create_s_path(tessellation.centers, width)

        drones = initialize_drones(sim, N_DRONES)
        fc = FlyController(sim, drones)

        sim.step()

        # Run the simulation
        run_simulation(sim, s_path, drones, fc)

        sim.stopSimulation()
        
    except Exception as e:
        logging.error(f"An error occurred: {e}")


# Ora la griglia globale è accessibile anche all'esterno della funzione:
def print_grid():
    global grid
    return grid


if __name__ == "__main__":
    main()
