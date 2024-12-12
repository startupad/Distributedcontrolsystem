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
web_app_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../web-app'))
sys.path.append(web_app_path)

# Ora puoi importare 'app' dalla cartella 'web-app'
from app import save_matrix_processed
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


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
        initial_config = [i + 1, 2, 0.5, 0, 0, 0, 1]
        drone = Drone(sim, drone_id=str(i + 1), starting_config=initial_config)
        drones.append(drone)
    return drones

def run_simulation(sim, s_path, drones, fc):
    prev_time = 0
    # Simulation loop
    for center in s_path:
        # Set the new target for the drone leader

        # Set up formation control parameters
        # il fattore di divisione deve essere lo stesso di drone.animationstep()
        step = (sim.getSimulationTime() - prev_time) / 7
        prev_time = sim.getSimulationTime()
        desired_dist_matrix = np.array([[0, 1.1, 1.1], [0.75, 0, 1.1], [0.75, 1, 0]])
        tolerance = 0.25

        # Compute formation control
        out = fc.formation_control(step, desired_dist_matrix, tolerance)

        # Setup drones parameters
        # using a proporitonal control to decide the slaves speed
        for i in range(len(drones)):
            drones[i].velocity = 2
            drones[i].velocity = drones[i].velocity * (1 + pow(pow(fc.matrix_norm[0, i].tolist(), 2), 0.5))
            print(f"drone {i} speed = ", drones[i].velocity)

        # set new target for the slave drones
        drones[0].calculate_new_path(center)
        drones[1].calculate_new_path(out[1])
        drones[2].calculate_new_path(out[2])
        
        all_drones_reached = False
        while not all_drones_reached:
            all_drones_reached = True
            for drone in drones:
                drone.next_animation_step()
                if not drone.has_reached_target():
                    all_drones_reached = False
            sim.step()
    
    print('GRIGLIA FINALE',print_grid())
    save_matrix_processed(grid)


def main():
    """Main function to run the simulation."""
    try:
        sim = initialize_simulation()

        terrain = Terrain(sim)
        tessellation = apply_tessellation(terrain)

        width = terrain.get_dimensions()[0]
        s_path = create_s_path(tessellation.centers, width)

        drones = initialize_drones(sim, N_DRONES)
        fc = FlyController(sim, drones)

        sim.step()  # Perform the first simulation step

        run_simulation(sim, s_path, drones, fc)

        sim.stopSimulation()
        logging.info("Simulation ended")

    except Exception as e:
        logging.error(f"An error occurred: {e}")

grid = [[0 for _ in range(6)] for _ in range(6)]  # Creazione della griglia 6x6

def define_grid(s_path, drones):
    global grid  # Usa la variabile globale grid
    grid_size = 6  # dimensione della griglia (6x6)
    index = 0  # Indice per iterare su s_path
    
    for center in s_path:  # Per ogni centro in s_path
        if index >= grid_size * grid_size:  # Se l'indice supera il numero massimo di celle nella griglia
            print("Warning: s_path contains more centers than can be accommodated in the grid.")
            break  # Ferma il ciclo se ci sono troppi centri per la griglia

        # Calcolare il valore medio per i 3 droni
        total_sensor_value = 0
        for i in range(3):  # Per ogni drone
            sensor_value = drones[i].read_sensor()  # Leggi il valore del sensore
            print(f'SENSORE DRONE {drones[i]}: {sensor_value}')
            total_sensor_value += sensor_value
        average_sensor_value = total_sensor_value / 3
        
        # Arrotondare il valore medio a 1, 2, o 3
        if average_sensor_value < 1.5:
            rounded_value = 1
        elif average_sensor_value < 2.5:
            rounded_value = 2
        else:
            rounded_value = 3

        # Calcolare le coordinate per il 'center'
        row = index // grid_size  # Indice della riga (divisione intera)
        col = index % grid_size   # Indice della colonna (modulo 6)

        # Inserire il valore arrotondato nella griglia
        grid[row][col] = rounded_value

        # Se siamo nella seconda e successiva riga, invertire l'ordine di riempimento
        if row % 2 == 1:
            grid[row].reverse()

        index += 1

    return grid  # Restituisce la griglia popolata

# Ora la griglia globale è accessibile anche all'esterno della funzione:
def print_grid():
    global grid
    return grid

if __name__ == "__main__":
    main()
