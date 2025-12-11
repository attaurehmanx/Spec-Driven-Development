# Lesson 4: Exercises & Quiz - URDF and Humanoid Robot Modeling

## Practical Exercises

### Exercise 1: Simple Robot Arm
Create a URDF model of a simple 3-DOF robot arm with a base, upper arm, and forearm connected by revolute joints.

**Steps:**
1. Create a base link with a cylindrical shape
2. Add an upper arm link connected by a revolute joint (shoulder)
3. Add a forearm link connected by another revolute joint (elbow)
4. Include proper visual, collision, and inertial properties for each link
5. Set appropriate joint limits
6. Visualize the model in RViz

### Exercise 2: Mobile Robot with Sensors
Create a URDF model of a simple wheeled robot with sensors:
- A main body/chassis
- Two wheels (left and right)
- A camera sensor mounted on top

**Steps:**
1. Create a rectangular chassis as the base link
2. Add two cylindrical wheels connected with continuous joints
3. Add a camera sensor as a fixed link attached to the chassis
4. Include appropriate materials and colors
5. Test the model by loading it in RViz

### Exercise 3: Humanoid with Xacro
Convert your simple humanoid model to use Xacro macros to reduce code duplication.

**Steps:**
1. Create macros for arms and legs
2. Use properties to define common values
3. Instantiate left and right limbs using the same macro
4. Verify the resulting URDF is equivalent to the original

## Chapter Quiz

### Question 1
What does URDF stand for?

A) Unified Robot Development Framework
B) Universal Robot Description Format
C) Unified Robot Description Format
D) Universal Robot Development Framework

**Answer:** C

### Question 2
Which element in URDF defines how a link appears in visualization tools?

A) `<collision>`
B) `<visual>`
C) `<appearance>`
D) `<render>`

**Answer:** B

### Question 3
What type of joint allows continuous rotation like a wheel?

A) Fixed
B) Revolute
C) Prismatic
D) Continuous

**Answer:** D

### Question 4
In the inertia matrix, what does `izz` represent?

A) Inertia around the X-axis
B) Inertia around the Y-axis
C) Inertia around the Z-axis
D) Total inertia

**Answer:** C

### Question 5
Which attribute defines the axis of rotation for a revolute joint?

A) `<origin>`
B) `<direction>`
C) `<axis>`
D) `<rotation>`

**Answer:** C

### Question 6
What is the correct way to specify a box geometry in URDF?

A) `<box size="0.1 0.1 0.1">`
B) `<geometry type="box" width="0.1" height="0.1" depth="0.1">`
C) `<box width="0.1" height="0.1" depth="0.1">`
D) `<geometry><box size="0.1 0.1 0.1"/></geometry>`

**Answer:** D

### Question 7
Which joint type allows linear motion along an axis?

A) Fixed
B) Revolute
C) Prismatic
D) Continuous

**Answer:** C

## Hands-on Challenge

Create a complete humanoid robot model with the following requirements:
1. A torso with a head
2. Two arms with shoulder and elbow joints
3. Two legs with hip, knee, and ankle joints
4. Proper inertial properties for stable simulation
5. Use Xacro to reduce code duplication for symmetrical parts (left/right arms and legs)

## Summary

This chapter covered the fundamentals of URDF and humanoid robot modeling:
- Understanding the structure of URDF files
- Creating links with visual, collision, and inertial properties
- Defining joints with appropriate types and limits
- Building complete humanoid models
- Using Xacro to simplify complex models

These skills are essential for creating robot models that can be used in simulation, visualization, and control systems in ROS 2.