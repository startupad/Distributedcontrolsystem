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

    # Configurazione iniziale e creazione di più droni
    drones = []
    initial_configs = [
        [0, 0, 0.5, 0, 0, 0, 1],
        [2, 0, 0.5, 0, 0, 0, 1],
        [0, 2, 0.5, 0, 0, 0, 1]
    ]

    for i, config in enumerate(initial_configs):
        drone = Drone(sim, id=str(i+1), starting_config=config)
        drones.append(drone)

    # Definizione delle configurazioni target per ciascun drone
    target_configs = [
        [2, 2, 0.5, 0, 0, 0, 1],
        [4, 4, 0.5, 0, 0, 0, 1],
        [6, 6, 0.5, 0, 0, 0, 1]
    ]

    for drone, target_config in zip(drones, target_configs):
        drone.calculate_new_path(target_config)

    # Ciclo di simulazione
    for i in range(150):
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
