import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from dask.array import apply_gufunc

from CoppeliaSim_project.tessellation import apply_tessellation
from drone import Drone
from terrain import Terrain
from fly_controller import FlyController


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()

    # define variable to remember previous time
    t_previous = 0

    # Creazione del terreno
    terrain = Terrain(sim)
    apply_tessellation(terrain)

    # Configurazione iniziale e creazione di più droni in modo dinamico
    n_drones = get_n_drones()
    drones = []

    # Definizione delle configurazioni target per ciascun drone in modo dinamico
    for i in range(n_drones):
        initial_config = [i + 1, 2, 0.5, 0, 0, 0, 1]
        drone = Drone(sim, id=str(i + 1), starting_config=initial_config)
        drones.append(drone)

    # build the fly controller
    fc = FlyController(sim, drones)
    target_configs = [3, 2, 1, 0, 0, 0, 1]

    # to avoid error during calculations of protocols involving delta t, we need to start with first t value != 0
    # then, I simply need to take the first simulation step before the simulation cycle
    sim.step()

    # Ciclo di simulazione
    for i in range(1000):
        # compute actual time
        t = sim.getSimulationTime()

        dist = np.array([[0, 1, 1], [1, 0, 1], [1, 1, 0]])
        # The frame of CoppeliaSim is refreshed every 50ms --> each step is 0.05s long that is to much, we need to reduce it
        step = (t - t_previous) / 10
        tolerance = 0.01
        a = fc.formation_control(step, dist, tolerance)

        t_previous = t

        for k in range(len(drones)):
            # print("a_k: ",a[k])
            drones[k].calculate_new_path(a[k])

        for drone in drones:
            drone.next_animation_step()
            # print(f'Drone {drone.id} position: {drone.get_position()}')
            # print("drone", drone.id, "read color: ", drone.read_sensor())
        sim.step()

    sim.stopSimulation()
    print("Simulation ended")


# definition of the function that retrieves the number of drones from the interface
def get_n_drones():
    return 3


if __name__ == "__main__":
    main()
