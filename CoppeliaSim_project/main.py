import logging
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
FILE_PATH_PROCESSED = 'data/processed_matrices.json'
FILE_PATH = 'data/matrices.json'

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
    for i in range(n_drones):
        initial_config = [i + 1, 0, 0.5, 0, 0, 0, 1]
        drone = Drone(sim, drone_id=str(i + 1), starting_config=initial_config)
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
        step = (sim.getSimulationTime() - prev_time) / 7
        prev_time = sim.getSimulationTime()
        desired_dist_matrix = np.array([[0, 1.1, 1.1], [0.75, 0, 1.1], [0.75, 1, 0]])
        tolerance = 0.25

        # Compute formation control
        out = fc.formation_control(step, desired_dist_matrix, tolerance)

        # Setup drone parameters and decide the slaves' speed
        for i in range(len(drones)):
            drones[i].velocity = 2
            drones[i].velocity = drones[i].velocity * (1 + pow(pow(fc.matrix_norm[0, i].tolist(), 2), 0.5))
            # print(f"drone {i} speed = ", drones[i].velocity)

        # Set new targets for the slave drones
        drones[0].calculate_new_path(center)
        drones[1].calculate_new_path(out[1])
        drones[2].calculate_new_path(out[2])

        target_coordinate_1 = [float(coord) for coord in center[:3]]
        target_coordinate_2 = [float(coord) for coord in out[1][:3]]
        target_coordinate_3 = [float(coord) for coord in out[2][:3]]
        set_coordinates(target_coordinate_1, target_coordinate_2, target_coordinate_3)

        # Ensure all drones reach their targets
        all_drones_reached = False
        while not all_drones_reached:
            all_drones_reached = True
            for drone in drones:
                drone.next_animation_step()
                if not drone.has_reached_target():
                    all_drones_reached = False
            sim.step()
    # Crea il percorso a S invertendo le righe dispari
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
    """Main function to run the simulation."""
    try:
        sim = initialize_simulation()

        priority_matrix = get_priority_matrix(FILE_PATH)  # matrice priorità assegnata dalla web-app

        terrain = Terrain(sim)
        tessellation_regular, tessellation_voronoi = apply_tessellation(terrain)

        # Variabile per decidere il tipo di tassellazione
        tessellation = tessellation_regular

        print(tessellation.centers)

        width = terrain.get_dimensions()[0]
        s_path = create_s_path(tessellation.centers, width)

        print(s_path)

        drones = initialize_drones(sim, N_DRONES)
        fc = FlyController(sim, drones)

        sim.step()  # Perform the first simulation step

        # Run the simulation with plotting
        run_simulation(sim, s_path, drones, fc)

        sim.stopSimulation()
        logging.info("Simulation ended")

    except Exception as e:
        logging.error(f"An error occurred: {e}")


# Ora la griglia globale è accessibile anche all'esterno della funzione:
def print_grid():
    global grid
    return grid


if __name__ == "__main__":
    main()
