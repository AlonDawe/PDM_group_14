import copy
import math
from queue import PriorityQueue

import numpy as np
import pygame

from src.node import Node


class AStarSimulation:
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
        # translate coordinates in the local coordinate frame to coordinates that can be used to plot in the pygame map
        return self.map_window_scale * (x + self.c_space.workspace.base_x), - self.map_window_scale * (
                y + self.c_space.workspace.base_y) + self.map_height

    def run_robot(self):
        self.map_window_name = "Robot Simulation"
        pygame.display.set_caption(self.map_window_name)
        self.map_dim = (800, 600)
        self.map_width, self.map_height = self.map_dim
        self.map = pygame.display.set_mode(self.map_dim)
        self.xc, self.yc = self.map.get_rect().center  # self.map center
        self.map.fill(self.white)

        run = True

        if self.path:
            print(f'Reached goal in {len(self.path)} steps.')
            for node in self.path:
                row, col = node
                q0, q1 = self.c_space.state_from_c_space_grid(col, row)
                self.c_space.workspace.robot.update_state([q0, q1], 0, 0)
                pygame.time.delay(100)
                self.plot_workspace()

        pygame.quit()

    #### C-SPACE SIMULATION #####

    @staticmethod
    def heuristic(type, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        if type == 'manhattan':
            return abs(x2 - x1) + abs(y2 - y1)  # manhattan distance - i.e L distance
        if type == 'euclidean':
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # euclidean distance

    def reconstruct_path(self, grid, came_from, current):
        # traversing from end node to start_node node
        path = []
        col, row = current.get_pos()
        path.insert(0, (row, col))
        while current in came_from:
            col, row = current.get_pos()
            path.insert(0, (row, col))
            current = came_from[current]
            current.make_path()
            self.draw(grid)
        col, row = current.get_pos()
        path.insert(0, (row, col))
        self.path = path

    def algorithm(self, draw, grid, heuristic_type):  # draw is a function
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, self.start_node))  # fscore, count, start_node
        came_from = {}
        g_score = {node: float("inf") for row in grid for node in row}
        g_score[self.start_node] = 0

        f_score = {node: float("inf") for row in grid for node in row}
        f_score[self.start_node] = self.heuristic(heuristic_type, self.start_node.get_pos(), self.goal_node.get_pos())

        open_set_hash = {self.start_node}

        while not open_set.empty():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            current = open_set.get()[2]  # fscore, count, node --> we want the node
            # popping lowest value node in the priority queue
            open_set_hash.remove(current)

            if current == self.goal_node:
                self.reconstruct_path(grid, came_from, self.goal_node)
                self.goal_node.make_end()
                return True

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1  # assume all edges are of wieght 1

                if temp_g_score < g_score[neighbor]:
                    # if there's a better way to get to this neighbor, we update the path
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + self.heuristic(heuristic_type, neighbor.get_pos(),
                                                                      self.goal_node.get_pos())
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        neighbor.make_open()  # out start_node node has been explore so we move onto neighbors
            draw()

            if current != self.start_node:
                current.make_closed()

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

    def draw_grid(self):
        # this draws the grid lines
        gap = self.map_width // self.c_space.n
        for i in range(self.c_space.n + 1):
            # drawing horizontal lines ; multiply col index by the gap
            pygame.draw.line(self.map, self.grey, (0, i * gap), (self.map_width, i * gap))
            for j in range(self.c_space.n + 1):
                # drawing vertical lines ; multiply row index by the gap
                pygame.draw.line(self.map, self.grey, (j * gap, 0), (j * gap, self.map_width))

    def draw(self, grid):
        self.map.fill(self.white)
        for row in grid:
            for node in row:
                node.draw(self.map)
        self.draw_grid()
        pygame.display.update()

    def run_c_space(self, heuristic='manhattan'):
        self.map_window_name = "C-Space Simulation"
        pygame.display.set_caption(self.map_window_name)
        self.map_dim = (800, 800)
        self.map_width, self.map_height = self.map_dim
        self.map = pygame.display.set_mode(self.map_dim)
        self.xc, self.yc = self.map.get_rect().center  # self.map center
        self.map.fill(self.white)

        # determines the checks that were doing; collision checks and setting obstacles
        grid = self.make_grid()

        run = True

        while run:
            self.draw(grid)
            for event in pygame.event.get():
                # loop through all the events that have happened and check them
                if event.type == pygame.QUIT:
                    run = False

                start_node = grid[self.start_col][self.start_row]
                goal_node = grid[self.goal_col][self.goal_row]

                if not self.start_node and start_node != self.goal_node:
                    self.start_node = start_node
                    self.start_node.make_start()

                elif not self.goal_node and goal_node != self.start_node:
                    self.goal_node = goal_node
                    self.goal_node.make_end()

                if self.start_node and self.goal_node:
                    # update neighbors of the spot class
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    if self.algorithm(lambda: self.draw(grid), grid, heuristic):
                        run = False
                        break
                    else:
                        print('No path from start_node to goal_node was found.')
                        break

        pygame.quit()
