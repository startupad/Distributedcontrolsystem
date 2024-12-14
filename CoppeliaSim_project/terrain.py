import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from math import exp
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend


class Terrain:
    def __init__(self, sim):
        self.sim = sim
        self.texture_file_name = "texture.png"
        self.last_color_change_time = 0
        self.colors = ["#2e7411", "#a7ee89", "#c1951c"]  # verde scuro, verde chiaro, marroncino
        self.width = 6
        self.length = 6
        self.height = 0.1
        self.terrain_handle = 0
        self.texture_id = 0
        self.init_terrain(xlim=(-0.1, 1.1), ylim=(-0.1, 1.1), resolution=100)

    def get_dimensions(self):
        return self.width, self.length

    def init_terrain(self, xlim: tuple, ylim: tuple, resolution):
        """
                Plots a 2D Gaussian Mixture Model (GMM) distribution over a specified range.

                Parameters:
                - mean: The mean of the Gaussian distribution (x, y).
                - sigma: The standard deviation of the Gaussian distribution.
                - xlim: Tuple specifying the x-axis limits (xmin, xmax).
                - ylim: Tuple specifying the y-axis limits (ymin, ymax).
                - resolution: The number of points in each axis direction for the grid (higher means smoother).
                """
        # Create a grid of points
        x = np.linspace(xlim[0], xlim[1], resolution)
        y = np.linspace(ylim[0], ylim[1], resolution)
        X, Y = np.meshgrid(x, y)

        # Define the components for a good Gaussian Mixture Model (GMM)
        # means = [(0.1, 0.1), (0.5, 0.5), (1.1, 0.5)]  # Three Gaussian means
        sigmas = [0.3, 0.3, 0.4]  # Standard deviation = variance for each Gaussian component
        # weights = [0.25, 0.35, 0.40]  # Weights for each component, they sum to 1

        # Generate 3 random Gaussian means as 2D tuples (x, y)
        means = [(np.random.uniform(xlim[0], xlim[1]), np.random.uniform(ylim[0], ylim[1])) for _ in range(3)]

        # Generate random weights and normalize them to sum to 1
        weights = np.random.rand(3)
        weights /= weights.sum()  # Normalize so that they sum to 1

        # Calculate Gaussian Mixture Model (GMM) values over the grid
        Z = np.array([self.gauss_pdf_mixture(x, y, means, sigmas, weights) for x, y in zip(np.ravel(X), np.ravel(Y))])
        Z = Z.reshape(X.shape)

        # Create a custom colormap
        cmap = LinearSegmentedColormap.from_list("field_colors", self.colors, N=256)

        # setup fig dimensions --> use the /2.54 to convert from inches to cm
        fig = plt.figure(figsize=(100 / 2.54, 100 / 2.54))

        # Plot the Gaussian Mixture Model distribution as a heatmap
        plt.contourf(X, Y, Z, levels=3, cmap=cmap)

        # remove axis from the generated map
        plt.axis('off')
        # save the generated figure as png file
        plt.savefig(self.texture_file_name, format='png', bbox_inches='tight', pad_inches=0)

        # Show the plot
        # plt.show()

        # Create a primitive texture shape (small plane)
        # Ensure absolute path
        path = os.path.abspath("texture.png")
        print("Absolute path to texture:", path)

        shape, self.texture_id, res = self.sim.createTexture(path, 2, [1, 1], [2, 2], [0, 0, 0], 128, None)

        # Create terrain  = primitive plane
        self.terrain_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_plane, [self.width, self.length, 1],
                                                            0)

        # moving the terrain in the correct position
        self.sim.setObjectPosition(self.terrain_handle, [self.width / 2, self.length / 2, 0], self.sim.handle_world)

        # Cover the terrain with the texture
        self.sim.setShapeTexture(self.terrain_handle, self.texture_id, self.sim.texturemap_plane, 2,
                                 [self.width, self.length], None, None)

        # moving the texture away from the scene
        self.sim.setObjectPosition(shape, [50, 50, 50], self.sim.handle_world)

    def gauss_pdf_mixture(self, x, y, means, sigmas, weights):
        """
        Calculate the value of a Gaussian Mixture Model (GMM) at point (x, y).

        Parameters:
        - means: List of tuples, where each tuple is a mean (xt, yt) for a Gaussian component.
        - sigmas: List of standard deviations for each Gaussian component.
        - weights: List of weights for each Gaussian component (must sum to 1).
        """
        result = 0
        for mean, sigma, weight in zip(means, sigmas, weights):
            xt, yt = mean
            # Gaussian 2D formula
            temp = ((x - xt) ** 2 + (y - yt) ** 2) / (2 * sigma ** 2)
            gaussian_val = exp(-temp) / (2 * np.pi * sigma ** 2)
            result += weight * gaussian_val
        return result
