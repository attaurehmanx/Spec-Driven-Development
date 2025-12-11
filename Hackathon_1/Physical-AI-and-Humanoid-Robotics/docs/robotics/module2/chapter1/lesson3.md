# Lesson 3: Configuring Robot Models with Physics

## Introduction to Robot Physics Configuration

Configuring robot models with proper physics properties is crucial for realistic simulation in digital twin applications. This lesson covers how to define mass, inertia, joints, and other physical properties that determine how a robot behaves in Gazebo.

## Robot Model Structure in SDF/URDF

Robot models in Gazebo are typically defined using either SDF (Simulation Description Format) or URDF (Unified Robot Description Format). Both formats define:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links that allow movement
- **Inertial properties**: Mass, center of mass, and inertia tensors
- **Collision geometry**: Shapes used for collision detection
- **Visual geometry**: Shapes used for rendering

## Defining Links with Physics Properties

### Basic Link Structure
```xml
<link name="link_name">
  <!-- Inertial properties -->
  <inertial>
    <mass>1.0</mass>  <!-- Mass in kg -->
    <inertia>
      <ixx>0.01</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.01</iyy>
      <iyz>0.0</iyz>
      <izz>0.01</izz>
    </inertia>
  </inertial>

  <!-- Collision geometry -->
  <collision name="collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
  </collision>

  <!-- Visual geometry -->
  <visual name="visual">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
  </visual>
</link>
```

### Calculating Inertia Values
For common shapes, you can use these formulas:
- **Box**: `I = 1/12 * m * (h² + d²)` for axis through center, where h and d are dimensions perpendicular to the axis
- **Cylinder**: `I = 1/12 * m * (3r² + h²)` for axis through center, perpendicular to height
- **Sphere**: `I = 2/5 * m * r²` for axis through center

## Joint Configuration for Physics

### Fixed Joint
```xml
<joint name="fixed_joint" type="fixed">
  <parent>parent_link</parent>
  <child>child_link</child>
</joint>
```

### Revolute Joint (Rotational)
```xml
<joint name="revolute_joint" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>
  <axis>
    <xyz>0 0 1</xyz>  <!-- Rotation axis -->
    <limit>
      <lower>-1.57</lower>  <!-- Lower limit in radians -->
      <upper>1.57</upper>   <!-- Upper limit in radians -->
      <effort>100.0</effort>  <!-- Maximum effort -->
      <velocity>1.0</velocity>  <!-- Maximum velocity -->
    </limit>
  </axis>
</joint>
```

### Prismatic Joint (Linear)
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent>parent_link</parent>
  <child>child_link</child>
  <axis>
    <xyz>1 0 0</xyz>  <!-- Linear motion axis -->
    <limit>
      <lower>0.0</lower>
      <upper>0.5</upper>
      <effort>100.0</effort>
      <velocity>1.0</velocity>
    </limit>
  </axis>
</joint>
```

## Creating a Simple Humanoid Robot Model

Here's a basic humanoid robot model with physics properties:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <!-- Torso -->
    <link name="torso">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </visual>
    </link>

    <!-- Head -->
    <link name="head">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="head_collision">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="head_visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>
    </link>

    <!-- Joint connecting torso and head -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10.0</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
      <pose>0 0 0.3 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

## Physics Material Properties

### Friction Parameters
```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- Static friction coefficient -->
        <mu2>1.0</mu2>    <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>  <!-- Primary slip coefficient -->
        <slip2>0.0</slip2>  <!-- Secondary slip coefficient -->
      </ode>
    </friction>
  </surface>
</collision>
```

### Contact Parameters
```xml
<surface>
  <contact>
    <ode>
      <soft_cfm>0.0</soft_cfm>
      <soft_erp>0.2</soft_erp>
      <kp>1000000.0</kp>  <!-- Contact stiffness -->
      <kd>1.0</kd>        <!-- Contact damping -->
      <max_vel>100.0</max_vel>
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
</surface>
```

## Advanced Physics Configuration

### Adding Sensors to Robot Model
```xml
<link name="sensor_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
    </imu>
  </sensor>
</link>
```

### Adding Actuators
For realistic simulation, you might want to add plugin controllers:

```xml
<plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
  <ros>
    <namespace>/simple_humanoid</namespace>
  </ros>
  <update_rate>30</update_rate>
</plugin>
```

## Validation of Physics Configuration

### Checking Mass Properties
1. Ensure total mass is reasonable for the robot size
2. Verify center of mass is physically plausible
3. Check that inertia values are positive and realistic

### Testing Stability
1. Test the model in Gazebo to ensure it doesn't exhibit unstable behavior
2. Verify that joints move as expected
3. Check that collisions behave correctly

## Common Physics Issues and Solutions

### Robot Sinks into Ground
- Increase collision mesh resolution
- Adjust ERP (Error Reduction Parameter) values
- Verify mass and inertia properties

### Unstable Joint Motion
- Reduce joint limits if too permissive
- Add damping to joints
- Check for singularities in kinematic chain

### Excessive Simulation Time
- Simplify collision geometry
- Use fewer joints if possible
- Adjust physics engine parameters

## Best Practices

1. **Start Simple**: Begin with basic shapes and add complexity gradually
2. **Validate Mass**: Ensure mass distribution is realistic
3. **Test Regularly**: Test models frequently during development
4. **Use Real Data**: When possible, use real robot specifications
5. **Document Parameters**: Keep track of physics parameters for future reference

## Summary

Properly configuring robot models with physics properties is essential for realistic simulation. By understanding link definitions, joint configurations, and material properties, you can create robot models that behave realistically in Gazebo.

In the next lesson, we'll practice what we've learned with exercises and a quiz.

## Exercises

1. Create a simple humanoid robot model with at least 5 links and 4 joints
2. Configure realistic mass and inertia properties for each link of your robot
3. Test your robot model in Gazebo and document any physics-related issues you encounter