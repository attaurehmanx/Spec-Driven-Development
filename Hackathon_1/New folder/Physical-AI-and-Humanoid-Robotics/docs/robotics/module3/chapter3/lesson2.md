---
title: "Lesson 3.2: Costmaps and Planning"
description: "Understanding costmaps and path planning in Nav2 navigation system"
tags: [navigation, Nav2, costmaps, path-planning, robotics, Isaac ROS]
learning_objectives:
  - "Students will understand what costmaps are and how they represent obstacles"
  - "Students will identify the different layers of costmaps and their purposes"
  - "Students will explain how path planners use costmaps to find safe routes"
  - "Students will recognize the difference between global and local path planning"
prerequisites:
  - "Module 1: Basic robotics concepts"
  - "Module 2: Simulation fundamentals"
  - "Chapter 1: AI Perception Fundamentals"
  - "Chapter 2: Mapping and Localization"
  - "Chapter 3 Introduction"
  - "Lesson 3.1: Nav2 Basics"
validation_status: draft
---

# Lesson 3.2: Costmaps and Planning

## Introduction

In the previous lesson, we learned about the Nav2 system and its main components. Now, let's dive deeper into one of the most important concepts in navigation: costmaps and path planning. Costmaps are essential for safe navigation - they help robots understand which areas of their environment are safe to travel through and which areas should be avoided. Understanding costmaps is crucial for creating robots that can navigate complex environments safely and efficiently.

## Definitions

- **Costmap**: A representation of the environment where different areas are assigned costs based on obstacles, with higher costs indicating areas to avoid
- **Global Costmap**: A costmap that covers a large area based on static map information, used by the global planner
- **Local Costmap**: A costmap that covers a smaller area around the robot, updated with real-time sensor data for local planning
- **Path Planning**: The process of finding a safe and efficient route through an environment using cost information
- **Global Path Planning**: Planning a route across a known map using static obstacles and global costmap
- **Local Path Planning**: Adjusting the path in real-time using local costmap and dynamic obstacle information
- **Inflation**: The process of expanding obstacles in a costmap to create safety margins around them
- **Obstacle Layer**: A layer in costmaps that represents static or dynamic obstacles
- **Map Layer**: A layer in costmaps that represents static map information
- **Voxel Layer**: A 3D layer in costmaps that represents obstacles in three dimensions

## Core Concepts

### What are Costmaps?

Costmaps are like a robot's "safety map" of its environment. Instead of just showing where obstacles are, costmaps assign different "costs" to different areas. Think of it like a heat map where:
- Green areas (low cost) are safe and easy to travel through
- Yellow areas (medium cost) are somewhat difficult or risky
- Red areas (high cost) are dangerous or should be avoided entirely

This allows the robot to make smart decisions about which path to take. Instead of just avoiding obstacles, it can choose the safest and most efficient route based on these cost values.

### Costmap Layers

Costmaps in Nav2 are built from multiple layers, each contributing different information:

**Map Layer**: This layer comes from the static map created during mapping and localization. It shows permanent features like walls, furniture, and other static obstacles that don't change.

**Obstacle Layer**: This layer uses real-time sensor data (from cameras, LIDAR, etc.) to detect dynamic obstacles like people, other robots, or moving objects that weren't in the original map.

**Inflation Layer**: This layer creates safety margins around all obstacles by "inflating" them. This ensures the robot doesn't get too close to obstacles, providing a buffer zone for safety.

**Voxel Layer**: For more complex environments, this 3D layer helps the robot understand obstacles in three dimensions, which is especially important for humanoid robots that need to navigate with height considerations.

### Global vs. Local Path Planning

Nav2 uses two types of path planning that work together:

**Global Path Planning**:
- Uses the global costmap (based on static map)
- Plans the overall route from start to goal
- Finds the most efficient path across the entire known environment
- Updates less frequently as it's computationally intensive

**Local Path Planning**:
- Uses the local costmap (with real-time sensor data)
- Makes moment-to-moment decisions about robot movement
- Adjusts the path to avoid dynamic obstacles
- Updates frequently (typically 10-20 times per second)

## Examples

### Example 1: Warehouse Navigation with Costmaps
In a warehouse environment, the global costmap shows permanent obstacles like shelves and walls. The local costmap updates in real-time to show workers moving around. The robot's path planner uses both to navigate efficiently while avoiding collisions.

### Example 2: Hospital Corridor Navigation
In a hospital, the global costmap includes the layout of rooms and walls. The local costmap detects people walking in corridors, medical equipment being moved, or doors opening and closing. The robot adjusts its path accordingly while respecting the overall building layout.

### Example 3: Home Environment Navigation
A home robot has a global costmap of the house layout. The local costmap detects pets, children, or furniture being moved. The costmap system helps the robot navigate safely while accomplishing its tasks.

## Diagrams

### Costmap Layer Architecture
```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Costmap Layers                                   │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐     │
│  │   Map Layer     │    │  Obstacle       │    │  Inflation      │     │
│  │ [Static map     │    │  Layer         │    │  Layer          │     │
│  │  information]   │    │ [Dynamic        │    │ [Safety         │     │
│  │                 │    │  obstacles]     │    │  margins]       │     │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘     │
│         │                       │                       │               │
│         ▼                       ▼                       ▼               │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Combined Costmap                           │   │
│  │  ┌─────────────────────────────────────────────────────────┐  │   │
│  │  │ • Static obstacles (walls, furniture)                 │  │   │
│  │  │ • Dynamic obstacles (people, moving objects)          │  │   │
│  │  │ • Inflated safety margins around all obstacles        │  │   │
│  │  │ • Navigation costs for path planning                  │  │   │
│  │  └─────────────────────────────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Path Planning Process
```
Global Planner (Long-term)        Local Planner (Short-term)
┌─────────────────────────┐      ┌─────────────────────────┐
│ • Uses global costmap   │      │ • Uses local costmap    │
│ • Plans overall route   │      │ • Adjusts path in real- │
│ • Considers static      │─────▶│   time based on        │
│   obstacles             │      │   dynamic obstacles    │
│ • Finds optimal path    │      │ • Makes moment-to-      │
│   across map           │      │   moment decisions     │
└─────────────────────────┘      └─────────────────────────┘
         │                                │
         ▼                                ▼
    Global Path                    Local Path Adjustments
    (Waypoints)                    (Trajectory Execution)
```

## Exercises

### Exercise 1: Costmap Visualization
1. Imagine a simple room with walls, a table in the center, and a chair near the door
2. Draw a grid representing this room (like graph paper)
3. Mark the static obstacles (walls, table) with one color
4. Mark the inflated safety margins around obstacles with another color
5. Mark a dynamic obstacle (a person walking) with a third color
6. Explain how these different layers would combine in a costmap

### Exercise 2: Path Planning Comparison
1. Draw a simple maze-like environment with start and goal points
2. Draw a global path that avoids static obstacles only
3. Now add a dynamic obstacle in the path
4. Draw how the local planner would adjust the path in real-time
5. Explain why both global and local planning are necessary

### Exercise 3: Costmap Parameter Analysis
1. Research how inflation radius affects robot navigation
2. Explain what would happen if the inflation radius is too small
3. Explain what would happen if the inflation radius is too large
4. Consider the trade-offs between safety and efficiency

### Exercise 4: Isaac Sim Costmap Simulation (Conceptual)
In this conceptual exercise, you'll explore how costmaps work in Isaac Sim:

**Part A: Static Map Integration**
1. In Isaac Sim, load a pre-built environment map
2. Observe how static obstacles are represented in the global costmap
3. Note the resolution and update rate of the static map layer

**Part B: Dynamic Obstacle Detection**
1. Add moving objects to the simulation (people, other robots)
2. Observe how these dynamic obstacles appear in the local costmap
3. Watch how the obstacle layer updates in real-time

**Part C: Path Planning Interaction**
1. Set a navigation goal in the environment
2. Watch how the global planner creates an initial path
3. Introduce a dynamic obstacle in the path
4. Observe how the local planner adjusts the robot's trajectory

**Expected Outcome**: By completing this exercise, you'll understand how costmaps enable safe navigation by representing environmental information in a way that path planners can use effectively.

## Quiz Questions

1. **What is a costmap in Nav2?**
   - A) A map showing only static obstacles
   - B) A representation of the environment where different areas are assigned costs based on obstacles
   - C) A map showing only dynamic obstacles
   - D) A map showing robot localization
   - **Answer: B** - A costmap is a representation of the environment where different areas are assigned costs based on obstacles, with higher costs indicating areas to avoid.

2. **What is the difference between global and local costmaps?**
   - A) Global costmaps are bigger than local costmaps
   - B) Global costmaps use static map information while local costmaps use real-time sensor data
   - C) Global costmaps show obstacles while local costmaps show the robot
   - D) There is no difference between them
   - **Answer: B** - Global costmaps use static map information for long-term planning while local costmaps use real-time sensor data for short-term adjustments.

3. **What is the purpose of the inflation layer in costmaps?**
   - A) To make obstacles larger for safety margins
   - B) To add visual effects to the map
   - C) To increase computation speed
   - D) To store localization data
   - **Answer: A** - The inflation layer creates safety margins around obstacles by expanding them, ensuring the robot doesn't get too close to obstacles.

4. **What does the global planner use for path planning?**
   - A) Only local costmap
   - B) Only sensor data
   - C) Global costmap based on static map information
   - D) Robot odometry only
   - **Answer: C** - The global planner uses the global costmap based on static map information to plan the overall route.

5. **What does the local planner do?**
   - A) Plans the overall route across the map
   - B) Makes moment-to-moment decisions about robot movement based on local costmap
   - C) Creates the static map
   - D) Localizes the robot only
   - **Answer: B** - The local planner makes moment-to-moment decisions about robot movement based on local costmap and real-time sensor data.

6. **Which layer in costmaps represents static map information?**
   - A) Obstacle layer
   - B) Inflation layer
   - C) Map layer
   - D) Voxel layer
   - **Answer: C** - The map layer represents static map information like walls, furniture, and other permanent obstacles.

7. **What is the purpose of local path planning?**
   - A) To plan the overall route across the environment
   - B) To make moment-to-moment decisions to avoid dynamic obstacles
   - C) To create the initial map
   - D) To control the robot's sensors
   - **Answer: B** - Local path planning makes moment-to-moment decisions to avoid dynamic obstacles using real-time sensor data.

8. **How often does local path planning typically update compared to global path planning?**
   - A) Local planning updates less frequently
   - B) Both update at the same rate
   - C) Local planning updates more frequently (10-20 times per second)
   - D) Global planning doesn't update after initial path
   - **Answer: C** - Local path planning updates more frequently (typically 10-20 times per second) while global planning updates less frequently.

9. **What is the obstacle layer in costmaps used for?**
   - A) Storing robot localization data
   - B) Representing dynamic obstacles using real-time sensor data
   - C) Storing map information permanently
   - D) Controlling robot motors
   - **Answer: B** - The obstacle layer represents dynamic obstacles using real-time sensor data from cameras, LIDAR, etc.

10. **Why are both global and local path planning necessary?**
    - A) For redundancy only
    - B) Global planning handles overall route while local planning handles real-time adjustments
    - C) One is for simulation and one is for real robots
    - D) Global planning is for indoor and local is for outdoor
    - **Answer: B** - Global planning handles the overall route across known static obstacles while local planning handles real-time adjustments for dynamic obstacles.

## Summary

In this lesson, we explored costmaps and path planning in Nav2 - essential concepts for safe robot navigation. We learned how costmaps represent environmental information with different costs assigned to different areas, helping robots make intelligent navigation decisions. We also examined the difference between global path planning (for overall routes) and local path planning (for real-time adjustments), and how both are necessary for effective navigation. Understanding costmaps is crucial for developing robots that can navigate complex environments safely and efficiently.

## Additional Resources

- Nav2 Costmap Configuration Guide
- Path Planning Algorithms in Robotics
- ROS 2 Navigation Tutorials
- Chapter 3, Lesson 3.1: Nav2 Basics (for component overview)
- Chapter 2, Lesson 2.3: Localization Methods (for map integration)