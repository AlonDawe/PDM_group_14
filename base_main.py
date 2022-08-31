import math

import pygame

from src.base_RRT_star import RRTGraph
from src.base_RRT_star import RRTMap
from src.robot_base import RobotBase


def main(base_algorithm, base_config):
    def plan():
        if base_algorithm == 'RRT*':
            X, Y, Parent = graph.expand_RRT_star()
        elif base_algorithm == 'RRT':
            X, Y, Parent = graph.expand_RRT()
        for i in range(len(X)):
            pygame.draw.circle(pygame_map.map, pygame_map.grey, (X[i], Y[i]), pygame_map.node_rad, 0)
            pygame.draw.line(pygame_map.map, pygame_map.blue, (X[i], Y[i]), (X[Parent[i]], Y[Parent[i]]),
                             pygame_map.edge_thickness)

    pygame.init()

    # Init map
    dimensions = (1000, 1000)
    start = (50, 50)
    goal = (900, 975)
    obs_dim = 50
    obs_num = 50
    pygame_map = RRTMap(start, goal, dimensions, obs_dim, obs_num)

    # Init base
    image = "src/base1.png"
    width = 0.01 * 3000
    height = 0.01 * 3000
    base = RobotBase(start, image, width, height)
    base.base_frame(pygame_map.map, (base.x, base.y), base.theta)
    graph = RRTGraph(start, goal, dimensions, obs_dim, obs_num)

    # Init obstacles
    if base_config == 'simple':
        obstacles, inf_obs = graph.make_simple_greenhouse(base.width, base.height)
    elif base_config == 'complex':
        obstacles, inf_obs = graph.make_greenhouse(base.width, base.height)
    graph.obstacles = inf_obs
    pygame_map.draw_map(obstacles)

    # Simulation loop
    running = True
    prev_time = pygame.time.get_ticks()
    comp_time_prev = pygame.time.get_ticks()
    exec_time_prev = pygame.time.get_ticks()
    timed = False
    while running:
        dt = (pygame.time.get_ticks() - prev_time) / 1000
        prev_time = pygame.time.get_ticks()

        # Loop over events to check for exit button
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pygame_map.map.fill((255, 255, 255))
        base.draw(pygame_map.map)
        pygame_map.draw_map(obstacles)

        if not graph.path_to_goal():
            plan()
        else:
            if timed == False:
                current_time = pygame.time.get_ticks()
                comp_time = current_time - comp_time_prev
                print('Computation time of the base simulation is', comp_time / 1000, 's.')
                exec_time_prev = pygame.time.get_ticks()
                timed = True
            # A path has been found, assign it to the robot
            if not base.path:
                base.path = graph.get_path_coords()
                # Calc path length
                dist = 0.0
                for i in range(1, len(base.path)):
                    x0, y0 = base.path[i - 1]
                    x1, y1 = base.path[1]
                    dist += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                print(f'Reached goal in {0.01 * dist} m.')

                base.waypoints2path()
                base.waypoint = len(base.path) - 1
            base.move(dt)

            pygame_map.draw_path(graph.get_path_coords())
        pygame.display.update()

        if (math.ceil(base.x), math.ceil(base.y)) == goal:
            current_time = pygame.time.get_ticks()
            exec_time = current_time - exec_time_prev
            print('Execution time of the base simulation is', exec_time / 1000, 's.')
            # print('Path Distance: ', graph.dist_to_start(graph.number_of_nodes(), graph.parent[-1]))
            running = False


if __name__ == '__main__':
    main(None)
