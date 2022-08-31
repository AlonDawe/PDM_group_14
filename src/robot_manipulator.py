import math

import numpy as np


class Robot:
    def __init__(self, l, m, g):
        self.l = l  # lists with link lengths
        self.m = m  # link mass
        self.g = g  # acceleration of gravity

        self.q = np.zeros([2])  # joint position
        self.dq = np.zeros([2])  # joint velocity
        self.tau = np.zeros([2])  # joint torque

        self.q_min = [0, 0]
        self.q_max = [np.pi, np.pi]

        self.link_polygons = None

    # forward kinematics

    def FK(self):
        p = np.zeros([2])  # endpoint position

        p[0] = self.l[0] * math.cos(self.q[0]) + \
               self.l[1] * math.cos(self.q[0] + self.q[1])
        p[1] = self.l[0] * math.sin(self.q[0]) + \
               self.l[1] * math.sin(self.q[0] + self.q[1])

        return p

    # Jacobian matrix

    def Jacobian(self):
        J = np.zeros((2, 2))

        J[0][0] = - self.l[0] * \
                  math.sin(self.q[0]) - self.l[1] * math.sin(self.q[0] + self.q[1])
        J[0][1] = - self.l[1] * math.sin(self.q[0] + self.q[1])
        J[1][0] = self.l[0] * \
                  math.cos(self.q[0]) + self.l[1] * math.cos(self.q[0] + self.q[1])
        J[1][1] = self.l[1] * math.cos(self.q[0] + self.q[1])

        return J

    # forward dynamics

    def FD(self):
        ddq = np.zeros([2])  # joint acceleration
        M = np.zeros((2, 2))  # mass matrix
        C = np.zeros([2])  # coriolis and centrifugal force vector
        G = np.zeros([2])  # gravity force vector

        M[0][0] = self.m[0] * self.l[0] ** 2 + self.m[1] * (
                self.l[0] ** 2 + 2 * self.l[0] * self.l[1] * math.cos(self.q[1]) + self.l[1] ** 2)
        M[0][1] = self.m[1] * (self.l[0] * self.l[1] * math.cos(self.q[1]) + self.l[1] ** 2)
        M[1][0] = self.m[1] * (self.l[0] * self.l[1] * math.cos(self.q[1]) + self.l[1] ** 2)
        M[1][1] = self.m[1] * self.l[1] ** 2

        C[0] = - self.m[1] * self.l[0] * self.l[1] * math.sin(self.q[1]) * (
                2 * self.dq[0] * self.dq[1] + self.dq[1] ** 2)
        C[1] = self.m[1] * self.l[0] * self.l[1] * math.sin(self.q[1]) * self.dq[0] ** 2

        G[0] = (self.m[0] + self.m[1]) * self.g * self.l[0] * math.cos(self.q[0]) + self.m[1] * self.g * self.l[
            1] * math.cos(self.q[0] + self.q[1])
        G[1] = self.m[1] * self.g * self.l[1] * math.cos(self.q[0] + self.q[1])

        ddq = np.linalg.inv(M) @ (self.tau - C - G)

        return ddq

    # inverse kinematics

    def IK(self, p):
        q = np.zeros([2])
        q[1] = math.acos((1 / (2 * self.l[0] * self.l[1])) * ((p[0] ** 2 + p[1] ** 2) -
                                                              (self.l[0] ** 2 + self.l[1] ** 2)))
        q[0] = math.atan2(p[1], p[0]) - math.atan2(self.l[1] * math.sin(q[1]),
                                                   self.l[0] + self.l[1] * math.cos(q[1]))
        # q = np.zeros([2])
        # r = np.sqrt(p[0] ** 2 + p[1] ** 2)
        # q[1] = np.pi - \
        #        math.acos((self.l[0] ** 2 + self.l[1] ** 2 - r ** 2) / (2 * self.l[0] * self.l[1]))
        # q[0] = math.atan2(p[1], p[0]) - math.acos((self.l[0] ** 2 - self.l[1] ** 2 + r ** 2) / (2 * self.l[0] * r))
        return q

    def get_state(self):
        return self.q, self.dq, self.tau

    # state change

    def update_state(self, q, dq, tau):
        self.q = q
        self.dq = dq
        self.tau = tau

    def get_geometry(self):
        return self.l[0], self.l[1]

    def get_joint_coordinates(self):
        # update individual link position
        q, dq, tau = self.get_state()
        l1, l2 = self.get_geometry()
        # update individual link position
        x0, y0 = 0, 0
        x1 = l1 * np.cos(q[0])
        y1 = l1 * np.sin(q[0])
        x2 = x1 + l2 * np.cos(q[0] + q[1])
        y2 = y1 + l2 * np.sin(q[0] + q[1])
        return (x0, y0), (x1, y1), (x2, y2)
