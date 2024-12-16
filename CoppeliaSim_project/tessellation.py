import numpy as np
import matplotlib.pyplot as plt
import logging

from matplotlib import patches
from shapely.geometry import Polygon
from scipy.spatial import Voronoi

np.random.seed(42)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class Tessellation:
    terrain = None
    clipped_regions = None
    squares = None

    def __init__(self, terrain, n_points=40, grid_size=1):
        """Initialize the tessellation with terrain."""
        self.terrain = terrain
        self.width, self.height = terrain.get_dimensions()
        self.points = np.random.rand(n_points, 2) * [self.width, self.height]  # Genera punti casuali
        self.squares = []
        self.grid_size = grid_size

    def generate_voronoi(self):
        """Generate Voronoi tessellation."""
        vor = Voronoi(self.points)
        return vor

    def clip_voronoi(self, vor):
        """Ritaglia le regioni di Voronoi all'interno dei limiti del terreno."""
        (xmin, ymin, xmax, ymax) = (0, 0, self.width, self.height)
        bbox = Polygon([(xmin, ymin), (xmin, ymax), (xmax, ymax), (xmax, ymin)])  # Limiti del terreno
        regions = []
        for region_index in vor.regions:
            if not region_index or -1 in region_index:  # Salta regioni infinite o vuote
                continue
            polygon = Polygon([vor.vertices[i] for i in region_index if i >= 0])
            clipped = polygon.intersection(bbox)  # Ritaglia il poligono
            if not clipped.is_empty:
                regions.append(clipped)
        return regions

    def get_region_centers(self):
        """Calcola i centri (centroidi) delle regioni di Voronoi."""
        self.centers = []  # Inizializza la lista dei centri
        for region in self.clipped_regions:
            centroid = region.centroid.coords[0]  # Usa il centroid di Shapely
            formatted_centroid = [centroid[0], centroid[1], 1, 0, 0, 0, 1]  # Formatta il centroide
            self.centers.append(formatted_centroid)
        return self.centers

    def plot_voronoi(self):
        """Plot the Voronoi tessellation."""
        fig, ax = plt.subplots()
        # Disegna i limiti del terreno
        ax.plot([0, 0, self.width, self.width, 0],
                [0, self.height, self.height, 0, 0], 'k-', lw=2)

        # Disegna le regioni di Voronoi ritagliate
        for region in self.clipped_regions:
            x, y = region.exterior.xy
            ax.fill(x, y, alpha=0.4, edgecolor='black', linewidth=0.8)

        # Disegna i punti originali
        ax.plot(self.points[:, 0], self.points[:, 1], 'ro', markersize=5)

        # Disegna i centroidi delle regioni
        centers = self.get_region_centers()
        for center in centers:
            ax.plot(center[0], center[1], 'bo')  # Centroidi in blu

        # Configurazioni finali
        plt.title("Tassellazione di Voronoi ritagliata sul terreno")
        plt.xlim(0, self.width)
        plt.ylim(0, self.height)
        plt.gca().set_aspect('equal')
        plt.savefig("voronoi_tessellation.png")

    def apply_grid(self):
        """Apply a grid tessellation to the terrain."""
        width, height = self.terrain.get_dimensions()
        x_coords = np.arange(0, width, self.grid_size)
        y_coords = np.arange(0, height, self.grid_size)

        for x in x_coords:
            for y in y_coords:
                square = self.create_square(x, y)
                self.squares.append(square)

    def plot_grid(self):
        """Plot the grid tessellation."""
        fig, ax = plt.subplots()
        self.plot_squares(ax)
        self.plot_centers(ax)
        self.set_plot_limits(ax)
        plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()

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

    def plot_squares(self, ax):
        """Plot the grid squares."""
        for square in self.squares:
            rect = patches.Polygon(square, closed=True, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

    def calculate_center(self, square):
        """Calculate the center point of a square."""
        x_center = (square[0][0] + square[2][0]) / 2
        y_center = (square[0][1] + square[2][1]) / 2
        return [x_center, y_center, 1, 0, 0, 0, 1]

    def create_square(self, x, y):
        """Create a square for the grid."""
        return [(x, y), (x + self.grid_size, y), (x + self.grid_size, y + self.grid_size), (x, y + self.grid_size)]


def apply_tessellation(terrain):
    """Apply tessellation to the given terrain."""

    # Voronoi
    tessellation_voronoi = Tessellation(terrain, n_points=40, grid_size=1)
    vor = tessellation_voronoi.generate_voronoi()
    tessellation_voronoi.clipped_regions = tessellation_voronoi.clip_voronoi(vor)
    tessellation_voronoi.plot_voronoi()

    # regular
    grid_size = 1
    tessellation_regular = Tessellation(terrain, grid_size=grid_size)
    tessellation_regular.apply_grid()
    tessellation_regular.plot_grid()
    squares = tessellation_regular.get_grid_squares()
    logging.info(f"Grid squares: {squares}")
    centers = tessellation_regular.get_grid_centers()
    logging.info(f"Grid centers: {centers}")
    return tessellation_regular, tessellation_voronoi
