import logging
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from CoppeliaSim_project.tessellation import apply_tessellation
from drone import Drone
from terrain import Terrain
from fly_controller import FlyController
from config import TOLERANCE, GRID_SIZE, N_DRONES

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
    """Run the simulation loop."""
    for center in s_path:
        drones[0].calculate_new_path(center)

        previousSimulationTime = 0
        step = (sim.getSimulationTime() - previousSimulationTime) / 10
        previousSimulationTime = sim.getSimulationTime()

        desired_dist_matrix = np.array([[0, 0.5, 1], [1, 0, 0.5], [1, 0.5, 0]])
        out = fc.formation_control(step, desired_dist_matrix, TOLERANCE)

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


if __name__ == "__main__":
    main()
