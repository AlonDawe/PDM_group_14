import time

import yaml

from src.c_space import CSpace
from src.manipulator_A_star import AStarSimulation
from src.manipulator_RRT import RRTSimulation
from src.robot_manipulator import Robot
from src.workspace import Workspace


def config(yaml_file):  # as file_name.yaml
    obstacles = []
    with open(yaml_file, 'r') as f:
        try:
            parsed = yaml.load(f, Loader=yaml.Loader)
            start = parsed["start_coordinate"][0]
            goal = parsed["goal_coordinate"][0]
            number_of_obs = parsed["number_of_obstacles"][0]
            for i in range(number_of_obs):
                obstacles.append(parsed["obstacles"][i])
        except yaml.YAMLError as exc:
            print(exc)
    return start, goal, obstacles, number_of_obs


def main(manipulator_config, manipulator_algorithm, heuristic=None):
    # CONFIGURATION
    start, goal, obstacles, number_of_obs = config(manipulator_config)

    # ROBOT PARAMETERS
    l1 = 1  # link 1 length
    l2 = 0.9  # link 2 length
    l = [l1, l2]  # link length
    m = [1.0, 0.9]  # link mass
    g = 9.81  # acceleration of gravity

    # INITIALIZE ROBOT
    robot = Robot(l, m, g)

    # INITIALIZE WORKSPACE
    workspace = Workspace(obstacles, robot, base_offset=0.3, workspace_dim=(4, 3))

    # BUILD WORKSPACE POLYGONS
    workspace.build_link_polygons()

    # INITIALIZE C-SPACE
    c_space = CSpace(n=100, workspace=workspace)

    # BUILD C-SPACE
    c_space.build_c_space()

    # INITIALIZE SIMULATION
    a_star_simulation = AStarSimulation(c_space=c_space, start=start, goal=goal)
    rrt_star_simulation = RRTSimulation(c_space=c_space, start=start, goal=goal)

    # RUN SIMULATION
    if manipulator_algorithm == 'A*' and heuristic:
        t_start = time.time()
        a_star_simulation.run_c_space(heuristic=heuristic)
        t_mid = time.time()
        print(f"Computation time of the of the manipulator simulation is {t_mid - t_start} s.")
        a_star_simulation.run_robot()
        t_end = time.time()
        print(f"Execution time of the manipulator simulation is {t_end - t_mid} s.")
    elif manipulator_algorithm == 'A*' and not heuristic:
        raise AttributeError('Heuristic must be defined for this algorithm.')
    elif manipulator_algorithm == 'RRT' and not heuristic:
        t_start = time.time()
        rrt_star_simulation.run_c_space()
        t_mid = time.time()
        print(f"Computation time of the of the manipulator simulation is {t_mid - t_start} s.")
        rrt_star_simulation.run_robot()
        t_end = time.time()
        print(f"Execution time of the manipulator simulation is {t_end - t_mid} s.")
    elif manipulator_algorithm == 'RRT' and heuristic:
        raise AttributeError('This algorithm does not take a heuristic. Please pass as NoneType.')


if __name__ == '__main__':
    # main(manipulator_config='configs/obstacle_configurations_2.yaml', manipulator_algorithm='A*', heuristic='manhattan')
    main(manipulator_config='configs/obstacle_configurations_2.yaml', manipulator_algorithm='RRT')
