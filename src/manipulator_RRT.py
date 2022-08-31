import copy
import math

import numpy as np
import pygame

from src.RRT import RRTMap, RRTGraph
from src.node import Node


class RRTSimulation:
    def __init__(self, c_space, start, goal):
        self.map_window_name = None
        self.map_window_scale = 200
        self.map_dim = None
        self.map_width, self.map_height = None, None
        self.map = None
        self.xc, self.yc = None, None
        self.start = None
        self.goal = None

        # Initial conditions for robot simulation
        self.t = 0.0  # time
        self.c_space = c_space
        dq = np.array([0., 0.])  # initial joint velocity
        tau = np.array([0., 0.])  # initial joint torque

        q_goal = self.c_space.workspace.robot.IK(goal)  # final joint position
        q_start = self.c_space.workspace.robot.IK(start)  # start joint position

        self.c_space.workspace.robot.update_state(q_goal, dq, tau)  # update initial state
        self.c_space.workspace.build_link_polygons()
        try:
            if self.c_space.workspace.check_collision():
                q_goal[0] += math.pi / 2
                q_goal[1] *= -1
                self.c_space.workspace.robot.update_state(q_goal, dq, tau)
                self.c_space.workspace.build_link_polygons()
                if self.c_space.workspace.check_collision():
                    raise ValueError
        except ValueError:
            raise ValueError('Goal position is in collision.')

        self.c_space.workspace.robot.update_state(q_start, dq, tau)  # update initial state
        self.c_space.workspace.build_link_polygons()
        try:
            if self.c_space.workspace.check_collision():
                q_goal[0] += math.pi / 2
                q_goal[1] *= -1
                self.c_space.workspace.robot.update_state(q_start, dq, tau)
                self.c_space.workspace.build_link_polygons()
                if self.c_space.workspace.check_collision():
                    raise ValueError
        except ValueError:
            raise ValueError('Start position is in collision.')

        # Initial conditions for C-Space simulation
        self.start_row, self.start_col = int(q_start[0] / (2 * np.pi) * self.c_space.n), int(
            q_start[1] / (2 * np.pi) * self.c_space.n)
        self.start_node = None

        self.goal_row, self.goal_col = int(q_goal[0] / (2 * np.pi) * self.c_space.n), int(
            q_goal[1] / (2 * np.pi) * self.c_space.n)
        self.goal_node = None
        self.path = None

        q_start_approx = self.c_space.state_from_c_space_grid(self.start_col, self.start_row)
        q_goal_approx = self.c_space.state_from_c_space_grid(self.goal_col, self.goal_row)

        self.c_space.workspace.robot.update_state(q_start_approx, dq, tau)
        self.start = self.c_space.workspace.robot.FK()

        self.c_space.workspace.robot.update_state(q_goal_approx, dq, tau)
        self.goal = self.c_space.workspace.robot.FK()

        # Colors

        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        self.grey = (128, 128, 128)

    def plot_workspace(self):
        # This method plots the workspace in a Pygame window
        self.map_window_name = "Robot Simulation"
        pygame.display.set_caption(self.map_window_name)
        self.map_dim = (800, 600)
        self.map_width, self.map_height = self.map_dim
        self.map = pygame.display.set_mode(self.map_dim)
        self.xc, self.yc = self.map.get_rect().center  # self.map center
        # real-time plotting
        self.map.fill(self.white)  # clear self.map

        for i, obstacle in enumerate(self.c_space.workspace.obstacles):
            if i == len(self.c_space.workspace.obstacles) - 1:
                break
            else:
                vertices = []
                for vertex in obstacle:
                    x, y = copy.deepcopy(vertex)
                    x, y = self.translate(x, y)
                    vertices.append([x, y])
                pygame.draw.polygon(self.map, self.green, vertices, 0)

        x_start, y_start = self.translate(*self.start)
        x_goal, y_goal = self.translate(*self.goal)

        pygame.draw.circle(self.map, self.red, (x_goal, y_goal), 30, 0)
        pygame.draw.circle(self.map, self.blue, (x_start, y_start), 10, 1)

        self.plot_robot()

        pygame.display.update()

    def plot_robot(self):
        # This method plots the robot in a Pygame window

        # get joint coordinates
        (x0, y0), (x1, y1), (x2, y2) = self.c_space.workspace.robot.get_joint_coordinates()

        # real-time plotting
        x0, y0 = self.translate(x0, y0)
        x1, y1 = self.translate(x1, y1)
        x2, y2 = self.translate(x2, y2)

        l1, l2 = self.c_space.workspace.robot.get_geometry()

        pygame.draw.lines(self.map, self.blue, False, [(x0, y0), (x1, y1), (x2, y2)], 7)  # draw links
        pygame.draw.circle(self.map, self.grey, (x0, y0), 7)  # draw shoulder / base
        pygame.draw.circle(self.map, self.grey, (x1, y1), 7)  # draw elbow
        pygame.draw.circle(self.map, self.red, (x2, y2), 7)  # draw hand / endpoint

        base_upper_left_x, base_upper_left_y = self.translate(-l1 / 2, 0)
        base_width = self.map_window_scale * l1
        base_height = self.map_window_scale * self.c_space.workspace.base_y

        pygame.draw.rect(self.map, self.grey, [base_upper_left_x, base_upper_left_y, base_width, base_height])

    def translate(self, x, y):
        # translate coordinates in th#e local coordinate frame to coordinates that can be used to plot in the pygame map
        return self.map_window_scale * (x + self.c_space.workspace.base_x), - self.map_window_scale * (
                y + self.c_space.workspace.base_y) + self.map_height

    def inv_translate(self, x, y):
        new_x = (x / self.map_window_scale) - self.c_space.workspace.base_x
        new_y = ((y - self.map_height) / - self.map_window_scale) - self.c_space.workspace.base_y
        return new_x, new_y

    def algorithm(self, draw, grid):
        # TO DO FOR PEPIJN
        # raise NotImplementedError
        return False

    def make_grid(self):
        grid = []
        gap = self.map_width // self.c_space.n  # integer division; width of each square
        for i in range(self.c_space.n + 1):
            grid.append([])
            for j in range(self.c_space.n + 1):
                node = Node(i, j, gap, self.c_space.n)
                if self.c_space.c_space_grid[i][j] == 1:
                    node.make_barrier()
                grid[i].append(node)
        return grid

    def draw(self, grid):
        obs = []
        self.map.fill(self.white)
        for row in grid:  # TODO this is not row but column!
            row_index = 0
            for node in row:
                row_index += 1
                rect = node.draw(self.map)
                if node.is_barrier() and row_index <= 52:
                    obs.append(rect)
        return obs

    def run_c_space(self):
        self.map_window_name = "Robot Simulation"
        pygame.display.set_caption(self.map_window_name)
        self.map_dim = (800, 800)
        self.map_width, self.map_height = self.map_dim
        self.map = pygame.display.set_mode(self.map_dim)
        self.xc, self.yc = self.map.get_rect().center  # self.map center
        self.map.fill(self.white)

        gap = self.map_width // self.c_space.n
        c0 = self.start_col + 1
        r0 = self.start_row + 1
        c1 = self.goal_col + 1.5
        r1 = self.goal_row + 1.5

        start = (gap * c0, gap * r0)
        goal = (gap * c1, gap * r1)

        world_map = RRTMap(start, goal, self.map_dim, 1, 0)
        graph = RRTGraph(start, goal, self.map_dim, 1, 0)

        grid = self.make_grid()
        obs = self.draw(grid)
        graph.obs = obs
        world_map.obs = obs
        iteration = 0

        # draw start and goal
        pygame.draw.circle(self.map, (255, 0, 0), start, 10, 1)
        pygame.draw.circle(self.map, (0, 0, 255), goal, 10, 1)

        while not graph.path_to_goal():
            if iteration % 10 == 0:
                x, y, parent = graph.bias(self.goal)
            else:
                x, y, parent = graph.expand()

            pygame.draw.circle(self.map, (100, 100, 100), (x[-1], y[-1]), world_map.node_rad, 0)
            pygame.draw.line(self.map, (0, 0, 255), (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]),
                             world_map.edge_th)

            if iteration % 5 == 0:
                pygame.display.update()
            iteration += 1

            if iteration > 1000:
                raise Exception('No path found')

        c_space_path = graph.get_path_coords()
        for node in c_space_path:
            pygame.draw.circle(self.map, (255, 0, 0), node, world_map.node_rad, 0)

        self.path = []
        for node in c_space_path:
            col = node[0] / gap
            row = node[1] / gap
            self.path.append((row, col))

        pygame.display.update()
        pygame.event.clear()

    def run_robot(self):
        self.map_window_name = "Robot Simulation"
        pygame.display.set_caption(self.map_window_name)
        self.map_dim = (800, 600)
        self.map_width, self.map_height = self.map_dim
        self.map = pygame.display.set_mode(self.map_dim)
        self.xc, self.yc = self.map.get_rect().center  # self.map center
        self.map.fill(self.white)

        if self.path:
            print(f'Reached goal in {len(self.path)} steps.')
            for node in reversed(self.path):
                row, col = node
                q0, q1 = self.c_space.state_from_c_space_grid(int(col), int(row))

                self.c_space.workspace.robot.update_state([q0, q1], 0, 0)
                pygame.time.delay(200)
                self.plot_workspace()

        pygame.quit()
