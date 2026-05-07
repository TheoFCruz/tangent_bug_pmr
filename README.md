# PMR TP1

ROS 2 package with path planning and control experiments for the iRobot Create 3 in Gazebo.

## Setup

Source ROS 2 and install package dependencies from the workspace root:

```bash
cd ~/Development/ros2/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --packages-select pmr_tp1 --symlink-install
source install/setup.bash
```

## Tangent Bug

Launch the simulation, RViz, map server, and Tangent Bug node:

```bash
ros2 launch pmr_tp1 tangent_bug.launch.py
```

Send a goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

Optional launch arguments:

```bash
ros2 launch pmr_tp1 tangent_bug.launch.py world:=bug_map.sdf
ros2 launch pmr_tp1 tangent_bug.launch.py map_name:=bug_map.yaml
ros2 launch pmr_tp1 tangent_bug.launch.py rviz_config_path:=/absolute/path/to/config.rviz
```

## Parametric Curve

Launch the simulation, RViz, and parametric curve node:

```bash
ros2 launch pmr_tp1 parametric_curve.launch.py
```

Start or reset the curve:

```bash
ros2 topic pub --once \
  /parametric_curve/start \
  std_msgs/msg/Bool "{data: true}"
```

Stop the robot:

```bash
ros2 topic pub --once \
  /parametric_curve/start \
  std_msgs/msg/Bool "{data: false}"
```

## Path With Potential

Launch the four-robot simulation, RViz, map server, and path-with-potential nodes:

```bash
ros2 launch pmr_tp1 path_with_potential.launch.py
```

Start or reset all robots. The command waits for four subscribers before publishing:

```bash
ros2 topic pub --once \
  -w 4 \
  /path_with_potential/start \
  std_msgs/msg/Bool "{data: true}"
```

Stop all robots:

```bash
ros2 topic pub --once \
  -w 4 \
  /path_with_potential/start \
  std_msgs/msg/Bool "{data: false}"
```

Optional launch arguments:

```bash
ros2 launch pmr_tp1 path_with_potential.launch.py world:=empty.sdf
ros2 launch pmr_tp1 path_with_potential.launch.py map_name:=empty_map.yaml
ros2 launch pmr_tp1 path_with_potential.launch.py rviz_config_path:=/absolute/path/to/config.rviz
```

## Potential Function

Launch the simulation, RViz, map server, and potential function node:

```bash
ros2 launch pmr_tp1 potential_function.launch.py
```

Send a goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

The node combines an attractive velocity toward the goal with a repulsive
velocity from the closest laser obstacle. The attractive term is quadratic near
the goal and conic farther away. The repulsive term is applied only inside the
obstacle influence distance. The final velocity is saturated and converted to
`/cmd_vel` with feedback linearization.

Optional launch arguments:

```bash
ros2 launch pmr_tp1 potential_function.launch.py world:=pot_func_map.sdf
ros2 launch pmr_tp1 potential_function.launch.py map_name:=pot_func_map.yaml
ros2 launch pmr_tp1 potential_function.launch.py rviz_config_path:=/absolute/path/to/config.rviz
```
