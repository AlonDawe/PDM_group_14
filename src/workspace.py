import math

import numpy as np
from shapely.geometry import Polygon


class Workspace:
    def __init__(self, obstacles, robot, base_offset, workspace_dim=(4, 3)):
        self.obstacles = obstacles
        self.polygon_obstacles = []
        self.workspace_width, self.workspace_height = workspace_dim
        self.base_x, self.base_y = self.workspace_width / 2, base_offset
        self.get_polygon_obstacles()
        self.robot = robot

    def get_polygon_obstacles(self):
        for obstacle in self.obstacles:
            for idx, vertex in enumerate(obstacle):
                x, y = vertex
                # must transform from global to manipulator local coordinate system
                x_l, y_l = x - self.base_x, y - self.base_y
                obstacle[idx][0] = x_l
                obstacle[idx][1] = y_l

            polygon_obstacle = Polygon(obstacle)
            self.polygon_obstacles.append(polygon_obstacle)

    def get_transformation_matrices(self):
        (x0, y0), (x1, y1), (_, _) = self.robot.get_joint_coordinates()
        (phi1, phi2), _, _ = self.robot.get_state()  # joint displacement angles

        # rotation angle to use in the transformation matrices
        theta1, theta2 = -math.pi / 2 + phi1, -math.pi / 2 + phi2 + phi1

        # transfer matrix for link one core coordinate system
        # this is the transformation matrix for link 1
        # in this case the origin of the global robot frame
        # and local link frane are the same
        T1 = np.array([math.cos(theta1), -math.sin(theta1), float(x0),
                       math.sin(theta1), math.cos(theta1), float(y0),
                       0, 0, 1]).reshape((3, 3))

        # transfer matrix for link 2
        # so coordinates need to be rotated by (theta1+theta2) + translated
        # by the x coordinate of the first joint
        T2 = np.array([math.cos(theta2), -math.sin(theta2), float(x1),
                       math.sin(theta2), math.cos(theta2), float(y1),
                       0, 0, 1]).reshape((3, 3))

        return T1, T2

    @staticmethod
    def get_edge_vertices_link(link_width, link_length, transformation_matrix):
        l = link_length
        w = link_width
        T = transformation_matrix
        # calculate the vertexes in local link frame
        # origin of this local coordinate frame is at the bottom of the link
        # i.e. at base_coords

        bl_local = np.array([-w / 2, 0, 1]).T
        br_local = np.array([w / 2, 0, 1]).T

        tl_local = np.array([-w / 2, l, 1]).T
        tr_local = np.array([w / 2, l, 1]).T

        # multiply vertices points to get it from local link frame
        # to global robot frame used in the search algorithm and kinematics
        bl = T @ bl_local
        br = T @ br_local
        tl = T @ tl_local
        tr = T @ tr_local

        return [bl[:2], tl[:2], tr[:2], br[:2]]

    def build_link_polygons(self):
        T1, T2 = self.get_transformation_matrices()
        l1, l2 = self.robot.get_geometry()
        link1 = self.get_edge_vertices_link(link_width=0.05, link_length=l1, transformation_matrix=T1)
        link2 = self.get_edge_vertices_link(link_width=0.05, link_length=l2, transformation_matrix=T2)
        self.robot.link_polygons = [Polygon(link1), Polygon(link2)]

    def check_collision(self):
        # returns True if any of the links are in collision with the obstacles
        for obstacle in self.polygon_obstacles:
            for link in self.robot.link_polygons:
                if obstacle.intersects(link):
                    return True
        return False
