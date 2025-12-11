# Lesson 2: Links and Joints

## Understanding Links and Joints in URDF

Links and joints form the fundamental building blocks of robot models in URDF. Links represent rigid bodies, while joints define the connections and allowed motions between these bodies. This lesson provides an in-depth look at creating and configuring links and joints.

## Links in Detail

Links are the rigid components of a robot model. Each link can have multiple properties that define its behavior in simulation and visualization.

### Link Properties

A complete link definition includes visual, collision, and inertial properties:

```xml
<link name="example_link">
  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Visual Elements

The visual element defines how a link appears in visualization tools like RViz:

```xml
<visual>
  <!-- Position and orientation relative to link frame -->
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>

  <!-- Geometric shape -->
  <geometry>
    <cylinder radius="0.05" length="0.2"/>
  </geometry>

  <!-- Material properties -->
  <material name="blue_material">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Elements

The collision element defines the collision geometry for physics simulation:

```xml
<collision>
  <!-- Position and orientation relative to link frame -->
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>

  <!-- Geometric shape for collision detection -->
  <geometry>
    <cylinder radius="0.05" length="0.2"/>
  </geometry>
</collision>
```

### Inertial Elements

The inertial element defines the physical properties for dynamics simulation:

```xml
<inertial>
  <!-- Mass in kilograms -->
  <mass value="0.5"/>

  <!-- Origin of the inertial frame -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Inertia matrix (symmetric, only 6 values needed) -->
  <inertia
    ixx="0.001" ixy="0" ixz="0"
    iyy="0.001" iyz="0" izz="0.002"/>
</inertial>
```

## Joints in Detail

Joints define the relationship between links and specify how they can move relative to each other.

### Joint Properties

A complete joint definition includes type, parent, child, and limits:

```xml
<joint name="example_joint" type="revolute">
  <!-- Links this joint connects -->
  <parent link="parent_link"/>
  <child link="child_link"/>

  <!-- Joint axis of rotation/translation -->
  <axis xyz="0 0 1"/>

  <!-- Position and orientation of joint -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>

  <!-- Joint limits -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Joint Types

#### Fixed Joint
No relative motion between parent and child:

```xml
<joint name="fixed_connection" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

#### Revolute Joint
Rotational motion around an axis with limits:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>
```

#### Continuous Joint
Continuous rotation around an axis (like a wheel):

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->
  <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
</joint>
```

#### Prismatic Joint
Linear motion along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider"/>
  <axis xyz="0 0 1"/>  <!-- Move along Z-axis -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

#### Planar Joint
Motion in a plane:

```xml
<joint name="planar_joint" type="planar">
  <parent link="base_link"/>
  <child link="platform"/>
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <limit lower="0" upper="1" effort="100" velocity="1"/>
</joint>
```

#### Floating Joint
6 degrees of freedom (no constraints):

```xml
<joint name="floating_joint" type="floating">
  <parent link="world"/>
  <child link="floating_object"/>
  <origin xyz="0 0 1" rpy="0 0 0"/>
</joint>
```

### Joint Origins and Axes

The origin element in a joint defines the position and orientation of the joint frame:

```xml
<!-- Position of joint relative to parent link frame -->
<origin xyz="0.1 0 0.2" rpy="0 0 1.57"/>

<!-- Axis of motion (for revolute/prismatic joints) -->
<axis xyz="0 0 1"/>  <!-- Z-axis rotation -->
```

## Complex Link and Joint Examples

### Simple Arm with Multiple Joints

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

## Best Practices for Links and Joints

### 1. Proper Frame Definitions
Ensure joint origins and link frames are defined consistently:

```xml
<!-- Good: Clear relationship between parent and child -->
<joint name="joint1" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>  <!-- 10cm along X-axis from parent -->
  <axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->
</joint>
```

### 2. Realistic Inertial Properties
Use proper mass and inertia values for stable simulation:

```xml
<!-- Calculate inertia for common shapes -->
<!-- For a cylinder: Ixx=Iyy=1/12*m*(3*r²+h²), Izz=1/2*m*r² -->
<inertial>
  <mass value="0.5"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
</inertial>
```

### 3. Joint Limits
Set appropriate limits for safe operation:

```xml
<limit lower="-2.0" upper="2.0" effort="100" velocity="2"/>
<!-- effort: maximum torque/force -->
<!-- velocity: maximum angular/linear velocity -->
```

## Summary

Links and joints are the fundamental building blocks of URDF robot models. Understanding how to properly define their properties, relationships, and constraints is crucial for creating accurate robot models that work well in simulation and visualization. In the next lesson, we'll explore how to build a complete humanoid robot model.

## Exercises

1. Create a URDF model with a base link and 3 connected links using different joint types.
2. Add proper visual, collision, and inertial properties to each link.
3. Explain how joint limits affect robot simulation behavior.