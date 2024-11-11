import random

class Terrain:
    def __init__(self, sim):
        self.sim = sim
        self.terrain_list = []
        self.last_color_change_time = 0
        self.colors = [
            [0, 128, 0],    # Verde
            [200, 0, 0],    # Rosso
            [255, 255, 0],  # Giallo
            [100, 100, 0]   # Marrone
        ]
        self.init_terrain()

    def init_terrain(self):
        width, length, height = 1, 1, 1
        for y in range(6):
            for x in range(6):
                terrain = self.sim.createPrimitiveShape(self.sim.primitiveshape_plane, [width, length, height], 0)
                self.terrain_list.append(terrain)
                self.sim.setObjectPosition(terrain, [x, y, 0], self.sim.handle_world)
                selected_color = random.choice(self.colors)
                self.sim.setObjectColor(terrain, 0, self.sim.colorcomponent_ambient_diffuse, selected_color)

    def update_terrain_colors(self):
        current_time = self.sim.getSimulationTime()
        if current_time - self.last_color_change_time >= 10:
            for terrain in self.terrain_list:
                selected_color = random.choice(self.colors)
                self.sim.setObjectColor(terrain, 0, self.sim.colorcomponent_ambient_diffuse, selected_color)
            self.last_color_change_time = current_time
