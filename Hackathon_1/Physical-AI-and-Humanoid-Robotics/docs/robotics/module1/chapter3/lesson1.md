# Lesson 1: URDF Basics

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, and other properties. This lesson covers the fundamental concepts and structure of URDF files.

## What is URDF?

URDF stands for Unified Robot Description Format and is:
- An XML-based format for describing robot models in ROS
- Used to define the physical and visual properties of robots
- Essential for robot simulation, visualization, and kinematics
- A standard format supported by ROS tools like RViz and Gazebo

## URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="fixed">
    <parent link="base_link"/>
    <child link="child_link"/>
  </joint>

  <link name="child_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links

Links represent rigid bodies in the robot:
- Each link has a unique name
- Links contain visual, collision, and inertial properties
- Links can have multiple visual and collision elements

### Visual Properties
Define how the link appears in visualization tools:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties
Define the collision geometry for physics simulation:

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties
Define the physical properties for dynamics simulation:

```xml
<link name="link_name">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

## Joint Types

Joints connect links and define their relative motion:

### Fixed Joint
No relative motion between parent and child:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

### Revolute Joint
Rotational motion around an axis:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Continuous Joint
Continuous rotation around an axis (like a wheel):

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Prismatic Joint
Linear motion along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="10" velocity="1"/>
</joint>
```

## Geometry Types

URDF supports several geometric shapes:

### Box
```xml
<geometry>
  <box size="0.1 0.2 0.3"/>  <!-- width, depth, height -->
</geometry>
```

### Cylinder
```xml
<geometry>
  <cylinder radius="0.1" length="0.2"/>
</geometry>
```

### Sphere
```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```

### Mesh
```xml
<geometry>
  <mesh filename="package://my_robot/meshes/link.stl" scale="1 1 1"/>
</geometry>
```

## Origin and Transformation

The `origin` element specifies position and orientation:

```xml
<origin xyz="1.0 0.0 0.0" rpy="0.0 0.0 1.57"/>
```
- `xyz`: Position (x, y, z) in meters
- `rpy`: Orientation (roll, pitch, yaw) in radians

## Materials

Materials define the visual appearance:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>
```

## Best Practices

### 1. Use Meaningful Names
Use descriptive names for links and joints:

```xml
<link name="wheel_front_left"/>
<joint name="hip_pitch_joint"/>
```

### 2. Organize Complex Models
For complex robots, consider using xacro (XML macros) to make URDF files more manageable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- Use xacro properties and macros -->
</robot>
```

### 3. Proper Inertial Values
For accurate simulation, provide realistic inertial properties:

```xml
<inertial>
  <mass value="0.5"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

## Summary

URDF is the standard format for describing robot models in ROS. Understanding links, joints, and their properties is essential for creating robot models for simulation and visualization. In the next lesson, we'll explore how to define links and joints in more detail.

## Exercises

1. Create a simple URDF file with a base link and one child link connected by a fixed joint.
2. Add visual properties to your links using different geometric shapes.
3. Explain the difference between visual and collision properties in URDF.