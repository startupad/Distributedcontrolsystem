import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class Tessellation:
    def __init__(self, terrain, grid_size):
        """Initialize the tessellation with terrain and grid size."""
        self.terrain = terrain
        self.grid_size = grid_size
        self.squares = []
        self.centers = []

    def apply_grid(self):
        """Apply a grid tessellation to the terrain."""
        # Get the terrain dimensions
        width, height = self.terrain.get_dimensions()
        x_coords = np.arange(0, width, self.grid_size)
        y_coords = np.arange(0, height, self.grid_size)

        # Create grid squares
        for x in x_coords:
            for y in y_coords:
                square = [(x, y), (x + self.grid_size, y), (x + self.grid_size, y + self.grid_size),
                          (x, y + self.grid_size)]
                self.squares.append(square)

    def plot_grid(self):
        """Plot the grid tessellation."""
        fig, ax = plt.subplots()
        for square in self.squares:
            rect = patches.Polygon(square, closed=True, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

        # Plot the center points
        self.get_grid_centers()
        for center in self.centers:
            ax.plot(center[0], center[1], 'bo')  # 'bo' means blue color, circle marker

        plt.xlim(0, self.terrain.get_dimensions()[0])
        plt.ylim(0, self.terrain.get_dimensions()[1])
        plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()

    def get_grid_squares(self):
        """Return the grid squares."""
        return self.squares

    def get_grid_centers(self):
        """Calculate and return the center points of the grid squares."""
        for square in self.squares:
            x_center = ((square[0][0] + square[2][0]) / 2)
            y_center = (square[0][1] + square[2][1]) / 2
            self.centers.append([x_center, y_center, 1, 0, 0, 0, 1])
        return self.centers


def apply_tessellation(terrain):
    """Apply tessellation to the given terrain."""
    grid_size = 1  # Define the size of each square
    tessellation = Tessellation(terrain, grid_size)

    # Apply grid tessellation
    tessellation.apply_grid()

    # Plot the grid tessellation
    tessellation.plot_grid()

    # Get and log the grid squares
    squares = tessellation.get_grid_squares()
    logging.info(f"Grid squares: {squares}")

    # Get and log the grid centers
    centers = tessellation.get_grid_centers()
    logging.info(f"Grid centers: {centers}")

    return tessellation
