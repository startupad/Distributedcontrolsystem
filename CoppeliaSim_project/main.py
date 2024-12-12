import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from tessellation import apply_tessellation
from drone import Drone
from terrain import Terrain
from fly_controller import FlyController


def create_s_path(centers, width):
    s_path = []
    for i in range(0, len(centers), width):
        row = centers[i:i + width]
        if (i // width) % 2 == 1:
            row.reverse()
        s_path.extend(row)
    return s_path


def main():
    # Initialize the client for communication with CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()


    # Variable to remember the previous time
    t_previous = 0

    # Create the terrain and apply tessellation
    terrain = Terrain(sim)
    tessellation = apply_tessellation(terrain)

    # Create the S-shaped path
    width = terrain.get_dimensions()[0]
    s_path = create_s_path(tessellation.centers, width)

    # Initial configuration and dynamic creation of multiple drones
    n_drones = get_n_drones()
    drones = []

    # Define initial configurations for each drone
    for i in range(n_drones):
        initial_config = [i + 1, 2, 0.5, 0, 0, 0, 1]
        drone = Drone(sim, id=str(i + 1), starting_config=initial_config)
        drones.append(drone)


    # Create the flight controller
    fc = FlyController(sim, drones)

    # Perform the first simulation step to avoid errors in calculations involving delta t
    sim.step()
    previousSimulationTime = 0
    

    for center in s_path:  # Per ogni centro in s_path
        # Calcolare il valore medio per i 3 droni
        grid = define_grid(s_path, drones)
        print(s_path.__len__())
                
        # Set the new target for each drone
        drones[0].calculate_new_path(center)

        # applying formation control
        step = (sim.getSimulationTime() - previousSimulationTime) / 10
        previousSimulationTime = sim.getSimulationTime()

        desired_dist_matrix = np.array([[0, 1.1, 0.5], [0.5, 0, 1.1], [0.5, 1.1, 0]])
        tolerance = 0.1

        out = fc.formation_control(step, desired_dist_matrix, tolerance)

        drones[1].calculate_new_path(out[1])
        drones[2].calculate_new_path(out[2])

        # Wait until all drones reach their target
        all_drones_reached = False
        while not all_drones_reached:
            all_drones_reached = True
            for drone in drones:
                drone.next_animation_step()
                if not drone.has_reached_target():
                    all_drones_reached = False

            # Perform a simulation step
            sim.step()
    
    print('GRIGLIA FINALE',print_grid())

    # Stop the simulation
    sim.stopSimulation()
    print("Simulation ended")


# Function to get the number of drones from the interface
def get_n_drones():
    return 3

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
    print("Griglia finale:")
    for row in grid:
        print(row)

if __name__ == "__main__":
    main()
