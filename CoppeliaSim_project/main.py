import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from CoppeliaSim_project.tessellation import apply_tessellation
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
    prev_time = 0

    # Simulation loop
    for center in s_path:
        # Set the new target for the drone leader
        drones[0].calculate_new_path(center)

        # Set up formation control parameters
        step = (sim.getSimulationTime() - prev_time) / 10
        prev_time = sim.getSimulationTime()
        desired_dist_matrix = np.array([[0, 1.1, 1.1], [0.75, 0, 1.1], [0.75, 1, 0]])
        tolerance = 0.1

        # Compute formation control
        out = fc.formation_control(step, desired_dist_matrix, tolerance)

        # Setup drones parameters
        # using a proporitonal control to decide the slaves speed
        # for i in range(len(drones)):
        #     drones[i].velocity = 1
        #     drones[i].velocity = drones[i].velocity * (1 + pow(pow( fc.matrix_norm[0, i].tolist(), 2), 0.5))
        #     print(f"drone {i} speed = ", drones[i].velocity)

        # set new target for the slave drones
        drones[1].calculate_new_path(out[1])
        drones[2].calculate_new_path(out[2])

        # Move all drones reach their target
        all_drones_reached = False
        while not all_drones_reached:
            all_drones_reached = True
            for drone in drones:
                drone.next_animation_step()
                if not drone.has_reached_target():
                    all_drones_reached = False

            # Perform a simulation step
            sim.step()

    # Stop the simulation
    sim.stopSimulation()
    print("Simulation ended")


# Function to get the number of drones from the interface
def get_n_drones():
    return 3


if __name__ == "__main__":
    main()
