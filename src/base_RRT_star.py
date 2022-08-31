"""
The following reference was used and the code was adapted to this project.
Additional functions and especially the RRT* algorithm (expand_RRT_star)
were developed by the project team. Changes and improvements were also done.

author = {Algobotics},
title = {Python RRT Path Planning},
year = {2021},
note = {Last accessed 10 January 2022},
url1 = {https://www.youtube.com/playlist?list=PL9RPomGb9IpRlfQEGkWnTt8jIauPovpOH}
url2 = {https://www.youtube.com/watch?v=ZekupxukiOM&list=PL9RPomGb9IpTOGS6xjuIb8WdwmmDQOt6L&ab_channel=Algobotics}
"""

import math
import random

import pygame


class RRTMap:
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.map_h = self.map_dimensions[0]
        self.map_w = self.map_dimensions[1]

        # window settings

        self.map_window_name = "RRT* Path Planning"
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.map_w, self.map_h))
        self.map.fill((255, 255, 255))  # Fill map with white background
        self.node_rad = 2
        self.node_thickness = 0
        self.edge_thickness = 1

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # colours TODO: extract to seperate color class or something
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    # draw_map: This function initializes the pygame map by placing the start_node, goal_node and obstacles
    # in their desired locations
    def draw_map(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start, self.node_rad + 5, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.node_rad + 20, 1)
        self.draw_obs(obstacles)

    # draw_path: This function draws the optimal path from start_node to goal_node node on the pygame map
    def draw_path(self, path):
        prev_node = None
        for node in path:
            if node == path[0]:
                prev_node = node
            pygame.draw.circle(self.map, self.red, node, self.node_rad + 2, 0)
            if node != path[0]:
                pygame.draw.line(self.map, self.red, node, prev_node, 3)
                prev_node = node

    # draw_obs: This function draws the obstacles on the pygame map
    def draw_obs(self, obstacles):
        obstaclesList = obstacles.copy()

        while len(obstaclesList) > 0:
            obstacle = obstaclesList.pop(0)

            pygame.draw.rect(self.map, self.green, obstacle)


class RRTGraph:
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_h, self.map_w = map_dimensions
        self.x = []
        self.y = []
        self.parent = []

        # Initialise the path tree

        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # Obstacles

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # Path

        self.goal_state = None
        self.path = []

    def make_random_rect(self):
        upper_corner_x = int(random.uniform(0, self.map_w - self.obs_dim))  # Top left corner
        upper_corner_y = int(random.uniform(0, self.map_h - self.obs_dim))

        return upper_corner_x, upper_corner_y

    def make_obs(self):
        obs = []

        for i in range(0, self.obs_num):
            rectangle = None
            start_goal_col = True

            while start_goal_col:
                upper = self.make_random_rect()
                rectangle = pygame.Rect(upper[0], upper[1], self.obs_dim, self.obs_dim)

                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    start_goal_col = True
                else:
                    start_goal_col = False

            obs.append(rectangle)
        self.obstacles = obs.copy()

        return obs

    def make_greenhouse(self, robot_w, robot_h):
        obs = []

        for h in range(90, self.map_h - 25, 150):
            corner = [0, h]
            rectangle = pygame.Rect(corner[0], corner[1], self.obs_dim * 8, self.obs_dim)
            obs.append(rectangle)

        for w in range(90, self.map_w - 25, 150):
            corner = [(self.map_w - self.obs_dim * 8), w]
            rectangle = pygame.Rect(corner[0], corner[1], self.obs_dim * 8, self.obs_dim)
            obs.append(rectangle)

        # inflate obstacles since robot is modeled as a single point
        inf_obs = []
        for ob in obs:
            inf_obs.append(ob.inflate(robot_w * 2, robot_h * 2))

        self.obstacles = obs.copy()

        return obs, inf_obs

    def make_simple_greenhouse(self, robot_w, robot_h):
        obs = []

        h = 300
        corner = [0, h]
        rectangle = pygame.Rect(corner[0], corner[1], self.obs_dim * 13, self.obs_dim)
        obs.append(rectangle)

        w = 700
        corner = [(self.map_w - self.obs_dim * 13), w]
        rectangle = pygame.Rect(corner[0], corner[1], self.obs_dim * 13, self.obs_dim)
        obs.append(rectangle)

        # inflate obstacles since robot is modeled as a single point
        inf_obs = []
        for ob in obs:
            inf_obs.append(ob.inflate(robot_w * 2, robot_h * 2))

        return obs, inf_obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)
        # self.parent[child] = parent

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    # TODO: merge this static method with dist function from the robot base
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        dist = math.sqrt(px + py)
        return dist

    def sample_envir(self):
        x = int(random.uniform(0, self.map_w))
        y = int(random.uniform(0, self.map_h))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def nearest_nodes(self, n):
        nnear = []
        radius_thresh = 70

        for i in range(0, n):
            if self.distance(i, n) < radius_thresh:
                nnear.append(i)

        if len(nnear) == 0:
            nnear.append(self.nearest(n))
        return nnear

    def is_free(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rectangle = obs.pop(0)
            if rectangle.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def cross_obstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rect = obs.pop(0)

            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)

                if rect.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        if self.cross_obstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=40):
        d = self.distance(nnear, nrand)
        if d > dmax:
            # u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)), int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)

            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goal_state = nrand
                self.goal_flag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            newpos = self.parent[self.goal_state]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goal_flag

    def dist_to_start(self, near_node, n):
        dist = self.distance(near_node, n)
        start_reached = False

        next_child = near_node
        while not start_reached:
            dist = dist + self.distance(self.parent[next_child], next_child)
            next_child = self.parent[next_child]
            if next_child == 0:
                start_reached = True
        return dist

    def get_path_coords(self):
        path_coords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            path_coords.append((x, y))
        return path_coords

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand_RRT(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.is_free():
            nnearest = self.nearest(n)
            self.step(nnearest, n)
            self.connect(nnearest, n)
        return self.x, self.y, self.parent

    def expand_RRT_star(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.is_free():

            # Part 1 of RRT* connect to the node that creates the shortest path to the start_node node

            nearest_nodes = self.nearest_nodes(n)
            best_node = nearest_nodes[0]
            min_dist = self.dist_to_start(nearest_nodes[0], n)

            for near_node in nearest_nodes:
                dist = self.dist_to_start(near_node, n)
                if dist < min_dist:
                    min_dist = dist
                    best_node = near_node

            self.step(best_node, n)
            self.connect(best_node, n)

            # Part 2 is to use the new node as a parent to all the nearest nodes. If this creates a shorter path to
            # goal_node, then update the paths.

            if n < self.number_of_nodes():
                for near_node in nearest_nodes:
                    if near_node != best_node:

                        dist_new = self.dist_to_start(n, near_node)
                        dist_old = self.dist_to_start(self.parent[near_node], near_node)

                        if dist_new < dist_old:
                            (x1, y1) = (self.x[n], self.y[n])
                            (x2, y2) = (self.x[near_node], self.y[near_node])

                            if not self.cross_obstacle(x1, x2, y1, y2):
                                self.parent[near_node] = n

        return self.x, self.y, self.parent

    def cost(self):
        pass
