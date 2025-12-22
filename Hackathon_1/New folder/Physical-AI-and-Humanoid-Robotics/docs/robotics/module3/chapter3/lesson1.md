---
title: "Lesson 3.1: Nav2 Basics"
description: "Introduction to Navigation 2 system and core navigation concepts"
tags: [navigation, Nav2, path-planning, robotics, Isaac ROS]
learning_objectives:
  - "Students will understand what Nav2 is and its role in robotics"
  - "Students will identify the main components of the Nav2 system"
  - "Students will recognize how Nav2 integrates with perception and mapping"
prerequisites:
  - "Module 1: Basic robotics concepts"
  - "Module 2: Simulation fundamentals"
  - "Chapter 1: AI Perception Fundamentals"
  - "Chapter 2: Mapping and Localization"
  - "Chapter 3 Introduction"
validation_status: draft
---

# Lesson 3.1: Nav2 Basics

## Introduction

Welcome to the exciting world of robot navigation! In this lesson, we'll explore Navigation 2 (Nav2), the ROS 2 navigation stack that enables robots to move autonomously through environments. After learning about perception and mapping in previous chapters, we now turn to the critical capability of navigation - how robots use their understanding of the environment to move safely to desired destinations.

Nav2 is the successor to the ROS 1 navigation stack and provides a comprehensive framework for robot navigation. It builds on the mapping and localization capabilities we've learned about, using maps to plan safe paths and executing those paths while avoiding obstacles. In this lesson, we'll explore the fundamental concepts of Nav2 and how it enables autonomous robot movement.

## Definitions

- **Nav2 (Navigation 2)**: The ROS 2 navigation stack that provides tools for autonomous robot navigation
- **Navigation**: The process of planning and executing a path from a starting point to a goal location
- **Path Planning**: The process of finding a safe and efficient route through an environment
- **Path Execution**: The process of following a planned path while handling real-world conditions
- **Navigation Stack**: A collection of software components that work together to provide navigation capabilities
- **Behavior Tree**: A hierarchical structure used in robotics to organize navigation tasks and decision-making processes
- **Recovery Behavior**: A strategy used by navigation systems to handle failures or difficult situations
- **Robot Coordinate Frame**: A reference system that defines the robot's position and orientation in space

## Core Concepts

### What is Nav2?

Nav2 (Navigation 2) is the official navigation stack for ROS 2, designed to provide robots with the ability to autonomously navigate through environments. It's a complete framework that includes:

- **Global Path Planning**: Finding optimal routes across known maps
- **Local Path Planning**: Adjusting paths in real-time to avoid obstacles
- **Path Execution**: Following paths while controlling robot motion
- **Recovery Behaviors**: Handling navigation failures gracefully

### Nav2 Architecture Components

**Global Planner**: Creates a path from the robot's current location to the goal based on a map of the environment. It plans the overall route considering known obstacles and map features.

**Local Planner**: Executes the path while avoiding obstacles in real-time. It makes moment-to-moment decisions about robot movement based on immediate sensor data.

**Controller**: Translates path plans into actual robot commands, managing the low-level motion control.

**Costmap System**: Maintains a representation of the environment with different areas assigned costs based on obstacles, with higher costs indicating areas to avoid.

**Lifecycle Manager**: Coordinates the state of navigation components, ensuring they're properly initialized and managed.

### Integration with Previous Concepts

Nav2 builds directly on the perception and mapping systems we've learned about:

- **Perception**: Provides real-time obstacle detection for local planning
- **SLAM/Mapping**: Supplies the global map for path planning
- **Localization**: Provides the robot's current position within the map

## Examples

### Example 1: Warehouse Robot Navigation
A warehouse robot uses Nav2 to navigate between storage locations. It receives a goal (e.g., "go to shelf A4"), the global planner creates a path across the warehouse map, and the local planner adjusts the path in real-time to avoid dynamic obstacles like workers and other robots.

### Example 2: Service Robot in Hospital
A service robot in a hospital uses Nav2 to deliver supplies between departments. It uses its map of the hospital to plan routes while the local planner handles dynamic obstacles like people walking in hallways.

### Example 3: Isaac ROS and Nav2 Integration
Isaac ROS components can integrate with Nav2 to provide enhanced perception capabilities. For example, Isaac ROS perception modules can provide more sophisticated obstacle detection to Nav2's costmap system.

## Diagrams

Using the System Architecture Template for Nav2:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Nav2 System                                   │
├─────────────────┬───────────────────────┬───────────────────────────────┤
│   Global        │      Local            │      Recovery                 │
│  Planning       │     Planning          │     Behaviors                 │
│  ┌───────────┐  │  ┌─────────────────┐  │  ┌─────────────────────────┐  │
│  │Global      │  │  │Local Planner    │  │  │Spin, Back Up, Wait    │  │
│  │Planner     │  │  │• Local Planner  │  │  │• Handle failures      │  │
│  │• Path Plan │  │  │• Trajectory Gen │  │  │• Recovery strategies  │  │
│  │• Map-based │  │  │• Obstacle avoid │  │  │• Safety behaviors     │  │
│  └───────────┘  │  └─────────────────┘  │  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

### Nav2 Component Flow
```
Goal Command → Global Planner → Local Planner → Controller → Robot Motion
     ↓              ↓               ↓            ↓            ↓
Receive Goal   Plan Path      Adjust Path   Send Cmds   Move Robot
Goal: x,y,θ   Global Path    Local Path    Velocities   Physical
              (Waypoints)    (Trajectory)  (Linear/Ang)  Movement
```

## Exercises

### Exercise 1: Nav2 Concept Understanding
1. Draw a simple map of your home or school
2. Mark a starting point and a destination
3. Sketch a potential path between them
4. Identify obstacles that would need to be avoided
5. Explain how this process relates to Nav2's path planning

### Exercise 2: Navigation Component Analysis
1. Research the different components of Nav2
2. Identify the purpose of each component
3. Explain how they work together to enable navigation
4. Consider what would happen if one component failed

### Exercise 3: Real-World Navigation Applications
1. Research a real-world application of Nav2 or similar navigation systems
2. Identify the environment and challenges
3. Explain how Nav2 addresses these challenges
4. Consider the safety and reliability requirements

### Exercise 4: Basic Nav2 Setup in Isaac Sim (Simulation)
In this exercise, you'll explore the basic setup of Nav2 within Isaac Sim. This is a conceptual exercise to familiarize yourself with the components without requiring actual hardware.

**Part A: Understanding Nav2 Configuration**
1. Nav2 requires configuration files that define parameters for each component (global planner, local planner, costmaps, etc.)
2. These configuration files typically include settings like robot footprint, inflation radius, and planner parameters
3. In Isaac Sim, these configurations would be integrated with the simulation environment

**Part B: Simulated Navigation Workflow**
1. Launch Isaac Sim and load a simple environment with obstacles
2. Set up a map using SLAM techniques from previous chapters
3. Define a goal location in the environment
4. Observe how Nav2 processes the goal and plans a path
5. Watch how the local planner adjusts the path when encountering simulated obstacles

**Part C: Component Interaction Analysis**
1. Notice how the global planner creates a path based on the static map
2. Observe how the local planner modifies the path based on real-time sensor data
3. Identify how the costmap system influences both planners
4. Understand how the controller translates path commands to actual robot motion

**Expected Outcome**: By completing this exercise, you'll understand how Nav2 components work together in simulation and be prepared to work with actual navigation systems in future lessons.

## Quiz Questions

1. **What is Nav2?**
   - A) A mapping system for robots
   - B) The ROS 2 navigation stack that provides tools for autonomous robot navigation
   - C) A perception system for robots
   - D) A robot simulation platform
   - **Answer: B** - Nav2 is the ROS 2 navigation stack that provides tools for autonomous robot navigation.

2. **What are the main components of the Nav2 system?**
   - A) Camera, LIDAR, and IMU
   - B) Global planner, local planner, controller, and costmap system
   - C) Wheels, motors, and sensors
   - D) Computer, battery, and chassis
   - **Answer: B** - The main components are global planner, local planner, controller, and costmap system.

3. **What is the role of the global planner in Nav2?**
   - A) To avoid obstacles in real-time
   - B) To create a path from the robot's current location to the goal based on a map
   - C) To control the robot's motors
   - D) To detect obstacles using sensors
   - **Answer: B** - The global planner creates a path from the robot's current location to the goal based on a map.

4. **What is a behavior tree in the context of navigation?**
   - A) A tree in the environment that the robot must avoid
   - B) A hierarchical structure used to organize navigation tasks and decision-making processes
   - C) A type of sensor
   - D) A mapping algorithm
   - **Answer: B** - A behavior tree is a hierarchical structure used to organize navigation tasks and decision-making processes.

5. **How does Nav2 integrate with the perception systems we learned about?**
   - A) It doesn't integrate with perception systems
   - B) It uses perception data for real-time obstacle detection in local planning
   - C) It only uses pre-built maps
   - D) It replaces perception systems
   - **Answer: B** - Nav2 uses perception data for real-time obstacle detection in local planning.

6. **What is the primary function of the local planner in Nav2?**
   - A) To create long-term paths across maps
   - B) To execute paths while avoiding obstacles in real-time
   - C) To maintain the robot's map
   - D) To control the robot's sensors
   - **Answer: B** - The local planner executes paths while avoiding obstacles in real-time based on immediate sensor data.

7. **What are recovery behaviors in Nav2?**
   - A) Strategies to recover from navigation failures or difficult situations
   - B) Methods to recover lost robot localization
   - C) Techniques to recover from sensor failures
   - D) Ways to recover robot battery power
   - **Answer: A** - Recovery behaviors are strategies used by navigation systems to handle failures or difficult situations.

8. **What is the purpose of costmaps in Nav2?**
   - A) To store robot localization data
   - B) To maintain a representation of the environment with different areas assigned costs based on obstacles
   - C) To store sensor data
   - D) To record robot movement history
   - **Answer: B** - Costmaps maintain a representation of the environment with different areas assigned costs based on obstacles, with higher costs indicating areas to avoid.

9. **Which component in Nav2 translates path plans into actual robot commands?**
   - A) Global planner
   - B) Local planner
   - C) Controller
   - D) Costmap system
   - **Answer: C** - The controller translates path plans into actual robot commands, managing the low-level motion control.

10. **How does Nav2 differ from the ROS 1 navigation stack?**
    - A) Nav2 is specifically designed for ROS 2 and provides an updated navigation framework
    - B) Nav2 only works with Isaac Sim
    - C) Nav2 doesn't use maps
    - D) Nav2 is only for humanoid robots
    - **Answer: A** - Nav2 is the successor to the ROS 1 navigation stack and is specifically designed for ROS 2, providing an updated and comprehensive navigation framework.

## Summary

In this lesson, we introduced Nav2 - the Navigation 2 system that enables autonomous robot navigation. We learned about its main components and how it builds on the perception and mapping capabilities from previous chapters. Nav2 represents the culmination of perception, mapping, and navigation, allowing robots to move safely and efficiently through environments. Understanding Nav2 is crucial for building robots that can perform autonomous navigation tasks.

## Additional Resources

- Nav2 Documentation and Tutorials
- ROS 2 Navigation Concepts
- Navigation in Robotics: A Survey
- Chapter 2, Lesson 2.1: Intro to SLAM (for mapping background)