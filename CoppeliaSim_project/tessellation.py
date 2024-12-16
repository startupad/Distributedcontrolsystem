import numpy as np
import matplotlib.pyplot as plt
import logging
from shapely.geometry import Polygon
from scipy.spatial import Voronoi

np.random.seed(42)


class Tessellation:
    terrain = None
    clipped_regions = None

    def __init__(self, terrain, n_points=40):
        """Initialize the tessellation with terrain."""
        self.terrain = terrain
        self.width, self.height = terrain.get_dimensions()
        self.points = np.random.rand(n_points, 2) * [self.width, self.height]  # Genera punti casuali

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


def apply_tessellation(terrain):
    """Apply tessellation to the given terrain."""
    tessellation = Tessellation(terrain)
    vor = tessellation.generate_voronoi()
    tessellation.clipped_regions = tessellation.clip_voronoi(vor)
    tessellation.plot_voronoi()
    return tessellation
