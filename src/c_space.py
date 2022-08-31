import copy

import numpy as np
from shapely.geometry import Polygon


class CSpace:
    def __init__(self, n, workspace):
        self.n = n
        self.minGridX = 0.0
        self.maxGridX = 2 * np.pi
        self.minGridY = 0
        self.maxGridY = 2 * np.pi
        (self.polygon_grid_cells, (self.x_mesh, self.y_mesh), (self.x_coords, self.y_coords),
         self.empty_distance_cells, (self.x_grid_size, self.y_grid_size)) = self.discretize()
        self.workspace = workspace
        self.c_space_grid = None

    def discretize(self):

        x_coords = np.linspace(self.minGridX, self.maxGridX, self.n + 1)
        y_coords = np.linspace(self.minGridY, self.maxGridY, self.n + 1)

        x_grid_size = abs(x_coords[1] - x_coords[0])
        y_grid_size = abs(y_coords[1] - y_coords[0])

        x_mesh, y_mesh = np.meshgrid(x_coords, y_coords)
        empty_distance_cells = np.zeros((self.n, self.n))

        polygon_grid_cells = {}
        for row in range(self.n):
            for col in range(self.n):
                cell = [(x_mesh[row][col], y_mesh[row][col]),
                        (x_mesh[row][col + 1], y_mesh[row][col + 1]),
                        (x_mesh[row + 1][col + 1], y_mesh[row + 1][col + 1]),
                        (x_mesh[row + 1][col], y_mesh[row + 1][col])]

                cell_polygon = Polygon(cell)
                polygon_grid_cells[(row, col)] = copy.deepcopy(cell_polygon)

        return (polygon_grid_cells, (x_mesh, y_mesh), (x_coords, y_coords),
                empty_distance_cells, (x_grid_size, y_grid_size))

    # returns the centroid of a given cell in the c space grid, i.e the q1, q2 configuration
    def state_from_c_space_grid(self, row, col):
        # xMesh, yMesh = self.numericGridCells

        cell = [(self.x_mesh[row][col], self.y_mesh[row][col]),
                (self.x_mesh[row][col + 1], self.y_mesh[row][col + 1]),
                (self.x_mesh[row + 1][col + 1], self.y_mesh[row + 1][col + 1]),
                (self.x_mesh[row + 1][col], self.y_mesh[row + 1][col])]

        tl = cell[0]  # top left
        br = cell[3]  # bottom right

        x_tl, y_tl = tl
        x_br, y_br = br

        x_centroid = (x_tl + x_br) / 2.0
        y_centroid = (y_tl + y_br) / 2.0
        centroid = [x_centroid, y_centroid]

        # give it in the same format as other states
        return np.array(centroid, dtype='float64')

    def build_c_space(self):
        self.c_space_grid = np.zeros((self.n + 1, self.n + 1), dtype='int32')

        for i in range(self.n):
            for j in range(self.n):
                q = self.state_from_c_space_grid(i, j)
                self.workspace.robot.update_state(q, 0, 0)
                self.workspace.build_link_polygons()

                if self.workspace.check_collision():
                    self.c_space_grid[i][j] = 1
                    self.c_space_grid[i + 1][j] = 1
                    self.c_space_grid[i][j + 1] = 1
                    self.c_space_grid[i + 1][j + 1] = 1

        return self.c_space_grid
