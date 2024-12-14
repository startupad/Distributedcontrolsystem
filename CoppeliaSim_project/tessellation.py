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
        width, height = self.terrain.get_dimensions()
        x_coords = np.arange(0, width, self.grid_size)
        y_coords = np.arange(0, height, self.grid_size)

        for x in x_coords:
            for y in y_coords:
                square = self.create_square(x, y)
                self.squares.append(square)

    def create_square(self, x, y):
        """Create a square for the grid."""
        return [(x, y), (x + self.grid_size, y), (x + self.grid_size, y + self.grid_size), (x, y + self.grid_size)]

    def plot_grid(self):
        """Plot the grid tessellation."""
        fig, ax = plt.subplots()
        self.plot_squares(ax)
        self.plot_centers(ax)
        self.set_plot_limits(ax)
        plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()

    def plot_squares(self, ax):
        """Plot the grid squares."""
        for square in self.squares:
            rect = patches.Polygon(square, closed=True, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

    def plot_centers(self, ax):
        """Plot the center points of the grid squares."""
        self.get_grid_centers()
        for center in self.centers:
            ax.plot(center[0], center[1], 'bo')

    def set_plot_limits(self, ax):
        """Set the plot limits based on terrain dimensions."""
        width, height = self.terrain.get_dimensions()
        ax.set_xlim(0, width)
        ax.set_ylim(0, height)

    def get_grid_squares(self):
        """Return the grid squares."""
        return self.squares

    def get_grid_centers(self):
        """Calculate and return the center points of the grid squares."""
        self.centers = []  # Clear the list of centers
        for square in self.squares:
            center = self.calculate_center(square)
            self.centers.append(center)

        print(len(self.centers))
        return self.centers

    def calculate_center(self, square):
        """Calculate the center point of a square."""
        x_center = (square[0][0] + square[2][0]) / 2
        y_center = (square[0][1] + square[2][1]) / 2
        return [x_center, y_center, 1, 0, 0, 0, 1]


def apply_tessellation(terrain):
    """Apply tessellation to the given terrain."""
    grid_size = 1
    tessellation = Tessellation(terrain, grid_size)
    tessellation.apply_grid()
    tessellation.plot_grid()
    squares = tessellation.get_grid_squares()
    logging.info(f"Grid squares: {squares}")
    centers = tessellation.get_grid_centers()
    logging.info(f"Grid centers: {centers}")
    return tessellation
