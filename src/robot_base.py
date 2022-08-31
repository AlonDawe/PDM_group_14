import math

import pygame


def dist(point1, point2):
    (x1, y1) = point1
    (x2, y2) = point2
    x1 = float(x1)
    x2 = float(x2)
    y1 = float(y1)
    y2 = float(y2)

    px = (x1 - x2) ** 2
    py = (y1 - y2) ** 2
    distance = (px + py) ** 0.5
    return distance


class RobotBase:

    def __init__(self, start, image, width, height):
        self.m_to_p = 3779  # m/s to px/s

        # Base dimensions
        self.width = width
        self.height = height

        # Base Initial Conditions
        self.x, self.y = start
        self.theta = 0
        self.path = []
        self.waypoint = 0
        self.a = 20
        self.u = 30  # pix/s
        self.W = 0  # rad/s
        self.v_left = 0.01 * self.m_to_p  # 0.01 m/s
        self.v_right = 0.01 * self.m_to_p

        self.max_v = 0.02 * self.m_to_p
        self.min_v = -0.02 * self.m_to_p

        self.base = pygame.image.load(image)
        self.rotated_base = self.base
        self.rect = self.rotated_base.get_rect(center=(self.x, self.y))

        # colors
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def draw(self, world_map):
        world_map.blit(self.rotated_base, self.rect)

    def move(self, dt):
        self.x += (self.u * math.cos(self.theta) - self.a * math.sin(self.theta) * self.W) * dt
        self.y += (self.u * math.sin(self.theta) + self.a * math.cos(self.theta) * self.W) * dt
        self.theta += self.W * dt
        self.rotated_base = pygame.transform.rotozoom(self.base, math.degrees(-self.theta), 1)
        self.rect = self.rotated_base.get_rect(center=(self.x, self.y))
        self.follow_path()

    def waypoints2path(self):
        old_path = self.path
        new_path = []
        for i in range(0, len(self.path) - 1):
            if i >= len(self.path):
                break
            x1, y1 = old_path[i]
            x2, y2 = old_path[i + 1]
            for j in range(0, 5):
                u = j / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                new_path.append((x, y))
        self.path = new_path

    def follow_path(self):
        target = self.path[self.waypoint]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y
        self.u = delta_x * math.cos(self.theta) + delta_y * math.sin(self.theta)
        self.W = (-1 / self.a) * math.sin(self.theta) * delta_x + (1 / self.a) * math.cos(self.theta) * delta_y

        if dist((self.x, self.y), self.path[self.waypoint]) <= 35:
            self.waypoint -= 1
        if self.waypoint <= 0:
            self.waypoint = 0

    def base_frame(self, world_map, position, rotation):
        axis_length = 50

        center_x = position[0]
        center_y = position[1]

        x_axis = (center_x + axis_length * math.cos(-rotation), center_y + axis_length * math.sin(-rotation))
        y_axis = (center_x + axis_length * math.cos(-rotation - math.pi / 2),
                  center_y + axis_length * math.sin(-rotation - math.pi / 2))

        pygame.draw.line(world_map, self.red, (center_x, center_y), x_axis, 3)
        pygame.draw.line(world_map, self.green, (center_x, center_y), y_axis, 3)
