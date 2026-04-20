# PMR TP1 - Path Planning Algorithms

This repository contains implementations of path planning algorithms for differential drive robots (specifically the iRobot Create 3) in ROS 2.

## Tangent Bug Algorithm

The Tangent Bug algorithm navigates towards a goal by following the direct path (Motion-to-Goal) until an obstacle is encountered. It then follows the boundary of the obstacle until it can resume its path to the goal.

### Prerequisites

- ROS 2 (Jazzy recommended)
- Gazebo Sim
- Eigen 3

### Running the Algorithm

1. **Build the workspace:**
   ```bash
   colcon build --packages-select pmr_tp1
   source install/setup.bash
   ```

2. **Launch the Tangent Bug simulation:**
   ```bash
   ros2 launch pmr_tp1 tangent_bug.launch.py
   ```
   *Note: This command launches Gazebo, RViz, and the Tangent Bug node. The default world is `test_map.sdf`.*

3. **Send a goal:**
   In a new terminal, publish a point to the `/goal` topic:
   ```bash
   ros2 topic pub /goal geometry_msgs/msg/Point "{x: 5.0, y: 0.0, z: 0.0}" -1
   ```

### Configuration

You can specify a different Gazebo world using the `world` argument:
```bash
ros2 launch pmr_tp1 tangent_bug.launch.py world:=empty.sdf
```
