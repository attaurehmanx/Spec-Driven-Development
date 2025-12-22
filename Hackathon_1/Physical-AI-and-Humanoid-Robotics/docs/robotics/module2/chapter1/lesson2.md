# Lesson 2: Creating Physics Environments

## Introduction to Physics Environments

A physics environment in Gazebo defines the world where your robot operates. This includes terrain, obstacles, lighting conditions, and physical properties like gravity. Creating realistic environments is crucial for effective digital twin applications.

## World File Structure

Gazebo uses SDF (Simulation Description Format) to define world files. The basic structure includes:

- **Gravity settings**: Defines the gravitational force in the world
- **Models**: Physical objects in the environment (robots, obstacles, etc.)
- **Lights**: Lighting sources for visualization
- **Physics engine**: Configuration of the physics simulation parameters

## Creating Different Terrain Types

### Flat Ground Environment
For basic testing, a flat ground environment is often sufficient:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="flat_ground">
    <gravity>0 0 -9.8</gravity>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Complex Terrain with Elevation
For more realistic environments, you can create terrains with varying elevations:

```xml
<model name="elevated_platform">
  <pose>5 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>4 4 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>4 4 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

### Obstacle Courses
Creating obstacle courses helps test robot navigation and path planning:

```xml
<!-- Series of obstacles -->
<model name="obstacle_1">
  <pose>2 1 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.5 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>

<model name="obstacle_2">
  <pose>3 -1 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.0</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.0</length>
        </cylinder>
      </geometry>
    </visual>
  </link>
</model>
```

## Physics Engine Configuration

### ODE Physics Parameters
```xml
<physics name="ode_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Environmental Effects

### Wind Simulation
Gazebo can simulate wind effects on objects:

```xml
<world name="windy_environment">
  <!-- Wind parameters -->
  <wind>
    <linear_velocity>0.5 0 0</linear_velocity>
  </wind>

  <!-- Rest of world definition -->
</world>
```

### Different Gravity Environments
You can simulate different gravitational environments (e.g., Moon, Mars):

```xml
<!-- Moon gravity (approximately 1/6 of Earth) -->
<gravity>0 0 -1.62</gravity>
```

## Using Built-in Models

Gazebo comes with many built-in models that can be easily included:

```xml
<!-- Include a warehouse model -->
<include>
  <uri>model://warehouse</uri>
  <pose>10 0 0 0 0 0</pose>
</include>

<!-- Include a table -->
<include>
  <uri>model://table</uri>
  <pose>0 3 0 0 0 0</pose>
</include>
```

## Advanced Environment Features

### Population Tool
For creating complex environments with many objects, you can use the population feature:

```xml
<state world_name="default">
  <model name="ground_plane">
    <pose>0 0 0 0 -0 0</pose>
    <link name="link">
      <pose>0 0 0 0 -0 0</pose>
      <velocity>0 0 0 0 -0 0</velocity>
      <acceleration>0 0 0 0 -0 0</acceleration>
      <wrench>0 0 0 0 -0 0</wrench>
    </link>
  </model>
</state>
```

## Performance Considerations

1. **Simplify Geometry**: Use simpler collision geometries for objects that don't require high precision
2. **Optimize Physics Step Size**: Balance accuracy and performance by tuning the max step size
3. **Limit Object Count**: Too many objects can slow down simulation significantly
4. **Use Appropriate Solvers**: Choose physics solvers based on your simulation requirements

## Best Practices

1. **Start Simple**: Begin with basic environments and add complexity gradually
2. **Validate Physics**: Ensure your environment behaves as expected physically
3. **Use Templates**: Create templates for common environment types
4. **Document Parameters**: Keep track of physics parameters that work well for your use cases

## Summary

Creating realistic physics environments is essential for effective digital twin applications. By understanding world file structure and physics parameters, you can create environments that accurately represent real-world conditions.

In the next lesson, we'll explore how to configure robot models with proper physics properties for realistic simulation.

## Exercises

1. Create a world file with a sloped terrain and test how a humanoid robot model would behave on it
2. Design an obstacle course with at least 5 different obstacles of various shapes
3. Experiment with different physics engine parameters and document their effects on simulation stability