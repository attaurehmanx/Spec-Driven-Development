---
title: Lesson 1.3 - Environmental Mapping and Scene Understanding
description: Learning environmental mapping and scene understanding in robotics
sidebar_position: 3
---

# Lesson 1.3: Environmental Mapping and Scene Understanding

## Learning Objectives
- Define environmental mapping and scene understanding in robotics
- Explain the role of mapping in robot navigation and perception
- Implement basic mapping techniques using ROS 2 and simulation
- Understand different mapping representations (occupancy grids, point clouds)
- Connect mapping with scene understanding for VLA systems

## Prerequisites
- Completion of Lessons 1.1 and 1.2
- Understanding of ROS 2 navigation concepts (Module 1)
- Basic knowledge of coordinate systems and transformations

## Definitions
- **Environmental Mapping**: The process of creating a representation of the environment for navigation and planning
- **Occupancy Grid**: A 2D grid-based map where each cell represents the probability of being occupied
- **SLAM (Simultaneous Localization and Mapping)**: The process of building a map while simultaneously localizing within it
- **Point Cloud**: A set of data points in space, typically representing the external surface of an object or environment
- **Scene Understanding**: The interpretation of the content and context of a visual scene
- **Semantic Mapping**: Mapping that includes object labels and contextual information
- **Topological Map**: A graph-based representation of the environment with nodes and connections
- **Metric Map**: A geometric representation with precise spatial relationships
- **Costmap**: A 2D grid representing the environment with costs associated with traversing each cell

## Explanations

### Environmental Mapping in Robotics
Environmental mapping is fundamental to robotics, enabling robots to navigate and operate in unknown or changing environments. The map serves as a representation of the world that the robot can use for path planning, obstacle avoidance, and task execution.

In the Vision-Language-Action (VLA) framework, environmental mapping provides the spatial context that connects perception to action. The robot's vision system contributes to mapping by identifying landmarks and obstacles, while the map provides the context for understanding spatial language commands.

### Types of Maps
Robots use different types of maps depending on their needs:

1. **Occupancy Grid Maps**: 2D or 3D grids where each cell contains the probability of being occupied. These are commonly used in ROS 2 navigation stacks.

2. **Point Cloud Maps**: Collections of 3D points that represent surfaces in the environment, often created using LIDAR or stereo vision.

3. **Topological Maps**: Graph-based representations focusing on connectivity between locations rather than geometric accuracy.

4. **Semantic Maps**: Maps that include object labels and contextual information, allowing for higher-level understanding.

### SLAM (Simultaneous Localization and Mapping)
SLAM is a critical capability that allows robots to build maps while simultaneously determining their position within those maps. This is particularly important for robots operating in unknown environments.

SLAM algorithms typically involve:
- Sensor data integration (from cameras, LIDAR, IMU, etc.)
- Feature extraction and matching
- Pose estimation and tracking
- Map building and updating

### Scene Understanding
Scene understanding goes beyond simple mapping to interpret the meaning and context of the environment. This includes:
- Object recognition and classification
- Spatial relationships between objects
- Functional properties of areas
- Potential affordances for action

## Examples

### Example 1: Basic Occupancy Grid Publisher
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Publisher for the map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 1)

        # Timer to publish map periodically
        self.timer = self.create_timer(5.0, self.publish_map)

        self.get_logger().info('Map Publisher Node initialized')

    def create_sample_map(self):
        # Create a sample occupancy grid
        map_msg = OccupancyGrid()

        # Set metadata
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        map_msg.info.resolution = 0.05  # 5cm per cell
        map_msg.info.width = 200  # 200 cells
        map_msg.info.height = 200  # 200 cells

        # Set origin (robot starts at center of map)
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0

        # Create sample data (10x10m map with some obstacles)
        data = np.zeros(200 * 200, dtype=np.int8)

        # Add some obstacles (represented as 100 in occupancy grid)
        # Vertical wall
        for y in range(50, 150):
            data[y * 200 + 100] = 100
            data[y * 200 + 101] = 100

        # Horizontal wall with gap
        for x in range(50, 95):  # Leave gap at x=95-105
            data[75 * 200 + x] = 100
            data[76 * 200 + x] = 100

        map_msg.data = data.tolist()
        return map_msg

    def publish_map(self):
        map_msg = self.create_sample_map()
        self.map_publisher.publish(map_msg)
        self.get_logger().info('Published sample occupancy grid map')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Point Cloud Processing for Mapping
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')

        # Subscriber for point cloud data
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pc_callback,
            10
        )

        # Publisher for processed point cloud
        self.pc_publisher = self.create_publisher(
            PointCloud2,
            '/processed_pointcloud',
            10
        )

        self.get_logger().info('Point Cloud Mapper Node initialized')

    def pc_callback(self, msg):
        # Convert PointCloud2 to list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Filter points (e.g., remove points too far away)
        filtered_points = []
        for point in points:
            x, y, z = point
            # Only keep points within 3 meters
            if np.sqrt(x**2 + y**2 + z**2) < 3.0:
                filtered_points.append([x, y, z])

        # Create new PointCloud2 message with filtered points
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create and publish the filtered point cloud
        filtered_pc = pc2.create_cloud(header, fields, filtered_points)
        self.pc_publisher.publish(filtered_pc)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Scene Understanding with Semantic Information
```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class SceneUnderstandingNode(Node):
    def __init__(self):
        super().__init__('scene_understanding_node')

        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/scene_markers',
            10
        )

        # Timer to publish scene understanding results
        self.timer = self.create_timer(1.0, self.publish_scene)

        self.get_logger().info('Scene Understanding Node initialized')

    def create_scene_markers(self):
        marker_array = MarkerArray()

        # Create marker for detected table
        table_marker = Marker()
        table_marker.header.frame_id = "map"
        table_marker.header.stamp = self.get_clock().now().to_msg()
        table_marker.ns = "furniture"
        table_marker.id = 1
        table_marker.type = Marker.CUBE
        table_marker.action = Marker.ADD

        # Position and size of table
        table_marker.pose.position.x = 2.0
        table_marker.pose.position.y = 1.0
        table_marker.pose.position.z = 0.5
        table_marker.pose.orientation.w = 1.0

        table_marker.scale.x = 1.5  # length
        table_marker.scale.y = 0.8  # width
        table_marker.scale.z = 0.1  # height

        table_marker.color.r = 0.8
        table_marker.color.g = 0.6
        table_marker.color.b = 0.4
        table_marker.color.a = 0.8

        table_marker.text = "Table"

        marker_array.markers.append(table_marker)

        # Create marker for detected cup
        cup_marker = Marker()
        cup_marker.header.frame_id = "map"
        cup_marker.header.stamp = self.get_clock().now().to_msg()
        cup_marker.ns = "objects"
        cup_marker.id = 2
        cup_marker.type = Marker.CYLINDER
        cup_marker.action = Marker.ADD

        # Position and size of cup
        cup_marker.pose.position.x = 2.2
        cup_marker.pose.position.y = 1.1
        cup_marker.pose.position.z = 0.65
        cup_marker.pose.orientation.w = 1.0

        cup_marker.scale.x = 0.08  # diameter
        cup_marker.scale.y = 0.08  # diameter
        cup_marker.scale.z = 0.1   # height

        cup_marker.color.r = 0.0
        cup_marker.color.g = 0.0
        cup_marker.color.b = 1.0
        cup_marker.color.a = 0.9

        cup_marker.text = "Cup"

        marker_array.markers.append(cup_marker)

        return marker_array

    def publish_scene(self):
        scene_markers = self.create_scene_markers()
        self.marker_publisher.publish(scene_markers)

def main(args=None):
    rclpy.init(args=args)
    node = SceneUnderstandingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step-by-Step Exercises

### Exercise 1: Setting up Mapping Infrastructure
1. **Create a new ROS 2 package** for mapping:
   ```bash
   cd ~/your_workspace/src
   ros2 pkg create --build-type ament_python mapping_pkg
   ```

2. **Install navigation dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

3. **Create the mapping node script** in the `mapping_pkg` directory:
   - Create the file `mapping_pkg/map_publisher.py` with the basic occupancy grid publisher from Example 1

4. **Update setup.py** to make the script executable

5. **Build and source your workspace**:
   ```bash
   cd ~/your_workspace
   colcon build --packages-select mapping_pkg
   source install/setup.bash
   ```

6. **Run your mapping node**:
   ```bash
   ros2 run mapping_pkg map_publisher
   ```

7. **Visualize the map** using RViz2:
   ```bash
   rviz2
   ```
   - Add a Map display and set the topic to `/map`

**Verification**: You should see the sample occupancy grid map in RViz2.

### Exercise 2: Point Cloud Processing
1. **Create a new script** called `pointcloud_mapper.py` in your mapping package

2. **Implement the point cloud processing node** from Example 2

3. **Test with simulated point cloud data**:
   - If using Isaac Sim, ensure depth camera is publishing point clouds
   - If using Gazebo, use a depth camera plugin

4. **Visualize the processed point cloud** in RViz2:
   - Add a PointCloud2 display
   - Set the topic to `/processed_pointcloud`

**Verification**: The processed point cloud should show only nearby points after filtering.

### Exercise 3: Scene Understanding Implementation
1. **Create a new script** called `scene_understanding.py` in your mapping package

2. **Implement the scene understanding node** from Example 3

3. **Integrate with object detection** (from Lesson 1.2):
   - Subscribe to object detection results
   - Create markers based on detected objects
   - Add semantic information to your map

4. **Visualize the scene understanding results** in RViz2:
   - Add a MarkerArray display
   - Set the topic to `/scene_markers`

**Verification**: You should see markers representing detected objects in the environment.

### Exercise 4: Mapping with Isaac Sim
1. **Set up Isaac Sim with mapping capabilities**:
   - Create a scene with various objects
   - Add a depth camera to your robot
   - Configure the ROS bridge for depth data

2. **Connect Isaac Sim to your mapping node**:
   - Ensure depth camera data is published to `/camera/depth/points`
   - Run your point cloud processing node

3. **Create a semantic map**:
   - Combine object detection results with spatial mapping
   - Create a map that includes both occupancy information and object labels

4. **Test navigation with your map**:
   - Use Nav2 to plan paths using your generated map
   - Verify that the robot can navigate around obstacles

**Verification**: The robot should successfully navigate using the map generated from Isaac Sim data.

## Quiz

### Question 1
What is the primary purpose of environmental mapping in robotics?

A) To store robot programs
B) To create a representation of the environment for navigation and planning
C) To improve robot aesthetics
D) To increase robot speed

**Correct Answer**: B
**Explanation**: Environmental mapping creates a representation of the environment that robots use for navigation, path planning, obstacle avoidance, and task execution. It's fundamental for autonomous operation.

### Question 2
What does SLAM stand for in robotics?

A) Systematic Localization and Mapping
B) Simultaneous Localization and Mapping
C) Sensor-based Localization and Mapping
D) Simple Localization and Mapping

**Correct Answer**: B
**Explanation**: SLAM stands for Simultaneous Localization and Mapping, which is the process of building a map while simultaneously determining the robot's position within that map.

### Question 3
In an occupancy grid map, what does a cell value of 100 typically represent?

A) Free space
B) Unknown space
C) Occupied space
D) Robot position

**Correct Answer**: C
**Explanation**: In ROS 2 occupancy grid maps, a cell value of 100 represents occupied space (definitely filled with an obstacle), 0 represents free space, and -1 represents unknown space.

### Question 4
What is a point cloud in the context of mapping?

A) A 2D grid map
B) A set of data points in space representing surfaces
C) A list of robot positions
D) A type of camera sensor

**Correct Answer**: B
**Explanation**: A point cloud is a collection of 3D points that represent the external surface of objects or environments, typically generated by LIDAR or depth cameras.

### Question 5
What is the difference between scene understanding and basic mapping?

A) There is no difference
B) Scene understanding includes object labels and contextual information
C) Scene understanding is only for indoor environments
D) Basic mapping is more accurate than scene understanding

**Correct Answer**: B
**Explanation**: While basic mapping focuses on geometric representation of space, scene understanding goes further to interpret the meaning and context of the environment, including object recognition, spatial relationships, and functional properties.

## Outcomes
- You understand the concepts of environmental mapping and scene understanding
- You can create and publish occupancy grid maps in ROS 2
- You can process point cloud data for mapping purposes
- You can implement scene understanding with semantic information
- You can integrate mapping with Isaac Sim simulation

## Summary
This lesson covered environmental mapping and scene understanding, including different mapping representations, SLAM concepts, and semantic mapping. We implemented basic mapping nodes and connected them to the broader VLA framework.

## Next Steps
- Proceed to Lesson 1.4: Perception Integration with ROS 2
- Practice creating more complex maps with varied environments
- Explore advanced mapping algorithms and techniques
- Consider how mapping connects to navigation and action planning in the VLA framework