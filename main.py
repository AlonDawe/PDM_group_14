from base_main import main as base_main
from manipulator_main import main as manipulator_main


def main(base_algorithm, base_config, manipulator_algorithm, manipulator_config, heuristic):
    base_main(base_algorithm, base_config)
    manipulator_main(manipulator_config, manipulator_algorithm, heuristic)


# base_algorithm: 'RRT' and 'RRT*'
# base_config: 'simple' and 'complex'
# manipulator_config: 'configs/obstacle_configurations_1.yaml'
#                     'configs/obstacle_configurations_2.yaml'
#                     'configs/obstacle_configurations_3.yaml'
# manipulator_algorithm: 'A*' and 'RRT'
# heuristic: 'manhattan' and 'euclidean'

if __name__ == '__main__':
    main(base_algorithm='RRT*', base_config='complex', manipulator_config='configs/obstacle_configurations_1.yaml',
         manipulator_algorithm='A*', heuristic='manhattan')
    # main(base_algorithm=None, manipulator_config='../configs/obstacle_configurations_1.yaml',
    #          manipulator_algorithm='RRT*', heuristic=None)
