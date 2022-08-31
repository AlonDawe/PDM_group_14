# Planning & Decision Making Project - RO47005

## PDM_group_14:

##### S. Boby: (4645588, sboby)

##### F. Corte Vargas (4674529, fcortevargas)

##### A. Dawe (5603250, ardawe)

##### P. Bogaard (4217489, pbogaard)

## Project Description:

This code evaluates short-listed motion planning algorithms for a mobile robot used in greenhouse applications. The
robot consists of a differential drive base and a 2 DOF manipulator arm. The two systems are decoupled in that when the
robot is moving, the manipulator remains in a fixed static position and vice versa. The short-listed planning algorithms
are tested and evaluated in multiple environments and performance measures including the respective computa- tion time,
execution time and path lengths are compared and explored. The algorithms analysed include RRT and RRT* for the
differential drive base and A* and RRT for the robot manipulator.

## How to get started:

Download all files from the main branch in the repository. Navigate to 'main.py'. Make sure that you have all the
packages imported.

In order to select different algorithms and graph configurations for the base and the manipulator please refer to the
list of options below. These can be used in the 'main.py' file

* **base_algorithm:** <br />
  'RRT' <br />'RRT*'
* **base_config:** <br />
  'simple' <br /> 'complex'
* **manipulator_config:** <br />
  'configs/obstacle_configurations_1.yaml'<br />
  'configs/obstacle_configurations_2.yaml'<br />
  'configs/obstacle_configurations_3.yaml'
* **manipulator_algorithm:** <br />
  'A*' <br /> 'RRT'
* **heuristic:** <br />
  'manhattan' <br /> 'euclidean'

## main.py

```python
def main(base_algorithm,  base_config, manipulator_algorithm, manipulator_config, heuristic):
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
```

## Conclusion:

Enjoy!


