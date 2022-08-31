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
    def __init__(self, start, goal, map_dim, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.map_w, self.map_h = map_dim

        # window settings
        self.window_name = "RRT path planning"
        pygame.display.set_caption(self.window_name)
        self.map = pygame.display.set_mode(map_dim)
        self.map.fill((255, 255, 255))
        self.node_rad = 2
        self.node_th = 0
        self.edge_th = 1

        self.obs = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

    def draw_map(self, obs):
        pygame.draw.circle(self.map, (0, 255, 0), self.start, self.node_rad + 5, 0)
        pygame.draw.circle(self.map, (255, 0, 0), self.goal, self.node_rad + 20, 1)
        self.draw_obs(obs)

    def draw_path(self, path):
        for node in path:
            pygame.draw.circle(self.map, (255, 0, 0), node, self.node_rad, 0)

    def draw_obs(self, obs):
        for ob in obs:
            pygame.draw.rect(self.map, (100, 100, 100), ob)


class RRTGraph:
    def __init__(self, start, goal, map_dim, obs_dim, obs_num):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_w, self.map_h = map_dim

        # init the tree
        self.x = [x]
        self.y = [y]
        self.parent = [0]

        # obstacles
        self.obs = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # path
        self.goal_state = None
        self.path = []

    def make_random_rect(self):
        upper_x = int(random.uniform(0, self.map_w - self.obs_dim))
        upper_y = int(random.uniform(0, self.map_h - self.obs_dim))

        rect = pygame.Rect((upper_x, upper_y), (self.obs_dim, self.obs_dim))
        return rect

    def make_obs(self):
        obs = []
        for i in range(0, self.obs_num):
            collision = True  # the ob cannot collide with the start or end
            while collision:
                rect = self.make_random_rect()
                collision = rect.collidepoint(self.start) or rect.collidepoint(self.goal)
            obs.append(rect)
        self.obs = obs.copy()
        return obs

    def add_node(self, node_id, x, y):
        self.x.insert(node_id, x)
        self.y.insert(node_id, y)

    def remove_node(self, node_id):
        self.x.pop(node_id)
        self.y.pop(node_id)

    def add_edge(self, parent_id, child_id):
        self.parent.insert(child_id, parent_id)

    def remove_edge(self, node_id):
        self.parent.pop(node_id)

    def number_of_nodes(self):
        return len(self.x)

    def dist(self, n1, n2):
        p0 = (self.x[n1], self.y[n1])
        p1 = (self.x[n2], self.y[n2])
        return math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)

    def sample_env(self):
        # random point between inside map borders
        x = int(random.uniform(0, self.map_w))
        y = int(random.uniform(0, self.map_h))
        return x, y

    def nearest(self, n):
        d_min = self.dist(0, n)
        n_near = 0
        for i in range(0, n):
            if self.dist(i, n) < d_min:
                d_min = self.dist(i, n)
                n_near = i
        return n_near

    def is_free(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        for ob in self.obs:
            if ob.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def cross_obstacle(self, x1, x2, y1, y2):
        for ob in self.obs:
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if ob.collidepoint(x, y):
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

    def step(self, n_near, n_rand, d_max=35):
        d = self.dist(n_near, n_rand)
        if d > d_max:
            (x_near, y_near) = (self.x[n_near], self.y[n_near])
            (x_rand, y_rand) = (self.x[n_rand], self.y[n_rand])
            theta = math.atan2(y_rand - y_near, x_rand - x_near)
            (x, y) = (int(x_near + d_max * math.cos(theta)), int(y_near + d_max * math.sin(theta)))
            self.remove_node(n_rand)
            (x_goal, y_goal) = self.goal
            if abs(x - x_goal) < d_max and abs(y - y_goal) < d_max:
                self.add_node(n_rand, x_goal, y_goal)
                self.goal_state = n_rand
                self.goal_flag = True
            else:
                self.add_node(n_rand, x, y)

    def bias(self, n_goal):
        n = self.number_of_nodes()
        self.add_node(n, n_goal[0], n_goal[1])
        n_near = self.nearest(n)
        self.step(n_near, n)
        self.connect(n_near, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        (x, y) = self.sample_env()
        self.add_node(n, x, y)
        if self.is_free():
            x_near = self.nearest(n)
            self.step(x_near, n)
            self.connect(x_near, n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            new_pos = self.parent[self.goal_state]
            while new_pos != 0:
                self.path.append(new_pos)
                new_pos = self.parent[new_pos]
            self.path.append(0)
        return self.goal_flag

    def get_path_coords(self):
        path_coords = []
        for node_id in self.path:
            (x, y) = (self.x[node_id], self.y[node_id])
            path_coords.append((x, y))
        return path_coords

    def cost(self):
        pass


def main():
    map_dim = (800, 600)
    start = (50, 50)
    goal = (510, 510)
    obs_dim = 30
    obs_num = 60
    iteration = 0

    pygame.init()
    world_map = RRTMap(start, goal, map_dim, obs_dim, obs_num)
    graph = RRTGraph(start, goal, map_dim, obs_dim, obs_num)

    obs = graph.make_obs()
    world_map.draw_map(obs)

    while not graph.path_to_goal():
        if iteration % 10 == 0:
            x, y, parent = graph.bias(goal)
        else:
            x, y, parent = graph.expand()

        pygame.draw.circle(world_map.map, (100, 100, 100), (x[-1], y[-1]), world_map.node_rad, 0)
        pygame.draw.line(world_map.map, (0, 0, 255), (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]),
                         world_map.edge_th)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1

    world_map.draw_path(graph.get_path_coords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    found = False
    while not found:
        try:
            main()
            found = True
        except:
            print("No path found, let's try again")
