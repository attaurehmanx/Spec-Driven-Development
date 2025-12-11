# Lesson 1: Gazebo Installation and Basic Physics

## Introduction to Gazebo Physics Simulation

Gazebo is a powerful 3D simulation environment that enables accurate and efficient testing of robotics algorithms, design of robots, and training of AI systems. In the context of digital twins, Gazebo provides realistic physics simulation that mirrors real-world behavior of humanoid robots.

## Installing Gazebo Garden

Gazebo Garden is the latest stable version of Gazebo that integrates well with ROS 2 Humble. To install Gazebo Garden on Ubuntu 22.04:

```bash
# Add the OSRF APT repository
sudo apt update && sudo apt install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update

# Install Gazebo Garden
sudo apt install gazebo
```

## Understanding Physics Engines

Gazebo supports multiple physics engines that determine how objects interact with each other:

1. **ODE (Open Dynamics Engine)**: The default physics engine, good for general-purpose simulation
2. **Bullet**: Provides better stability for complex collision scenarios
3. **DART**: Advanced physics engine with support for complex articulated bodies

## Basic Physics Concepts in Gazebo

### Gravity
Gravity is a fundamental force in physics simulation. By default, Gazebo simulates Earth's gravity (-9.8 m/s² in the Z direction):

```xml
<!-- Setting gravity in a world file -->
<world name="basic_world">
  <gravity>0 0 -9.8</gravity>
  <!-- Other world elements -->
</world>
```

### Collision Detection
Collision detection determines when two objects make contact. Gazebo uses geometric approximations to efficiently calculate collisions:

- **Collision geometry**: Used for collision detection calculations
- **Visual geometry**: Used for rendering (can be different from collision geometry)

### Mass and Inertia
For realistic physics simulation, each object must have properly defined mass and inertia properties:

- **Mass**: The amount of matter in an object
- **Inertia**: Resistance to rotational motion (defined as a 3x3 matrix)

## Creating Your First Physics Simulation

Let's create a simple world with a ground plane and a box:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="basic_physics_world">
    <!-- Define gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Create a simple box model -->
    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.083</iyy>
            <iyz>0.0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Running Your First Simulation

1. Save the above XML as `basic_physics_world.sdf`
2. Launch Gazebo with your world file:
```bash
gz sim basic_physics_world.sdf
```

## Physics Parameters and Tuning

Gazebo allows you to tune various physics parameters for different simulation scenarios:

- **Real-time factor**: Controls how fast the simulation runs compared to real-time
- **Max step size**: Time step for physics calculations (smaller = more accurate but slower)
- **Solver iterations**: Number of iterations for constraint solving (more = more stable but slower)

## Best Practices for Physics Simulation

1. **Start Simple**: Begin with basic shapes and simple physics before adding complexity
2. **Match Real-World Properties**: Use realistic mass, friction, and inertia values
3. **Tune Parameters**: Adjust physics parameters based on your simulation requirements
4. **Validate Results**: Compare simulation results with expected real-world behavior

## Summary

Gazebo provides a robust physics simulation environment essential for creating accurate digital twins of humanoid robots. Understanding basic physics concepts and how to configure them is crucial for realistic simulation.

In the next lesson, we'll explore creating more complex physics environments with different terrains and obstacles.

## Exercises

1. Create a simulation with two boxes of different masses and observe their falling behavior
2. Modify the gravity value in the world file and describe how it affects the simulation
3. Research and explain the difference between collision and visual geometry in Gazebo