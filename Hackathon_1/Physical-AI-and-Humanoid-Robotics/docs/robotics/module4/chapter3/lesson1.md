# Lesson 3.1: Navigation Systems with Nav2

## Introduction to Navigation in Robotics

Navigation is a fundamental capability for mobile robots, enabling them to move autonomously from one location to another while avoiding obstacles. Navigation2 (Nav2) is the navigation stack for ROS 2 that provides a complete framework for robot navigation in various environments.

## Key Concepts in Robot Navigation

### Navigation System Components
A typical robot navigation system consists of several key components:

1. **Localization**: Determining the robot's position in the environment
2. **Mapping**: Creating a representation of the environment
3. **Path Planning**: Finding a route from start to goal
4. **Motion Control**: Executing the planned path while avoiding obstacles

### Navigation2 Architecture
Navigation2 provides a robust and flexible architecture for robot navigation:

- **Map Server**: Loads and serves static maps
- **AMCL (Adaptive Monte Carlo Localization)**: Provides pose estimation
- **Global Planner**: Creates a path from start to goal
- **Local Planner**: Creates short-term trajectories to follow the global path
- **Controller**: Sends velocity commands to the robot
- **Recovery Behaviors**: Actions taken when navigation fails

## Setting Up Navigation2

### Installation
Navigation2 can be installed using the ROS 2 package manager:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Basic Launch
Navigation2 provides launch files to start the complete navigation system:

```bash
ros2 launch nav2_bringup navigation_launch.py
```

## Global Path Planning

Global path planning involves creating a path from the robot's current position to a goal position using a known map. The global planner typically uses algorithms like A* or Dijkstra's algorithm to find an optimal path.

```python
# Example of sending a navigation goal
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigationClient:
    def __init__(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigate_to_pose(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = theta

        self.nav_client.send_goal_async(goal)
```

## Local Path Planning and Obstacle Avoidance

Local path planning focuses on navigating the robot in real-time while avoiding obstacles that may not be in the static map. The local planner uses sensor data to create safe trajectories.

Key parameters for local planning include:
- Maximum velocity
- Obstacle inflation radius
- Goal tolerance
- Frequency of path updates

## Navigation Parameters and Tuning

Navigation performance can be tuned through configuration files. Key parameters include:

- `planner_frequency`: How often the global planner updates
- `controller_frequency`: How often the controller updates
- `inflation_radius`: How far to stay from obstacles
- `goal_tolerance`: How close to get to the goal

## Practical Example: Navigation in Simulation

Let's set up a simple navigation scenario in simulation:

1. Launch a robot in Gazebo or Isaac Sim
2. Load a map of the environment
3. Use RViz2 to send navigation goals
4. Observe the robot's path planning and execution

```bash
# Launch simulation environment
ros2 launch nav2_bringup tb3_simulation_launch.py

# In another terminal, send navigation goals via RViz2
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## Navigation Safety and Recovery

Navigation2 includes recovery behaviors to handle situations where the robot gets stuck or cannot progress:

- **Clearing Rotation**: Rotate in place to clear obstacles
- **Back Up**: Move backward to find a clearer path
- **Wait**: Pause briefly before retrying

These behaviors can be configured in the recovery server configuration.

## Summary

Navigation is a critical capability for mobile robots, enabling autonomous movement in complex environments. Navigation2 provides a comprehensive framework for implementing navigation in ROS 2-based robots, with components for path planning, obstacle avoidance, and recovery behaviors.