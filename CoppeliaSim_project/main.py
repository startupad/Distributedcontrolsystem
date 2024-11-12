from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from drone import Drone
from terrain import Terrain

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    # Creazione del terreno
    terrain = Terrain(sim)

    # Configurazione iniziale e creazione di più droni in modo dinamico
    drones = []
    n_drones = 3

    for i in range(n_drones):
        initial_config = [i+1, 0, 0.5*(1+i), 0, 0, 0, 1]
        drone = Drone(sim, id=str(i+1), starting_config=initial_config)
        drones.append(drone)

    # Definizione delle configurazioni target per ciascun drone in modo dinamico
    target_configs = []
    for i in range(n_drones):
        target_configs = [i+1, 0, 1, 0, 0, 0, 1]
        drones[i].calculate_new_path(target_configs)

    # Ciclo di simulazione
    for i in range(1000):
        for drone in drones:
            drone.next_animation_step()
        terrain.update_terrain_colors()
        for drone in drones:
            print(f'Drone {drone.id} position: {drone.get_position()}')
        sim.step()

    sim.stopSimulation()
    print("Simulation ended")

if __name__ == "__main__":
    main()
