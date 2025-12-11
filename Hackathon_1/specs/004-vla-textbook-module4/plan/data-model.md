# Data Model: Vision-Language-Action (VLA) System

**Feature**: Module 4 — Vision-Language-Action (VLA) for Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-10

## Core Entities

### VLACommand
**Description**: Represents a command received from natural language processing
- **command_text**: String - Original natural language command
- **command_type**: Enum (NAVIGATION, MANIPULATION, PERCEPTION, MISC) - Type of command
- **target_object**: String - Object the command refers to (if any)
- **target_location**: String - Location for navigation commands (if any)
- **confidence**: Float (0.0-1.0) - Confidence in command interpretation
- **timestamp**: DateTime - When command was processed

### VisionObservation
**Description**: Represents visual information from the environment
- **objects_detected**: Array of ObjectInfo - List of objects detected
- **environment_map**: OccupancyGrid - Current environment map
- **camera_feed**: Image - Raw camera data
- **depth_data**: PointCloud - 3D depth information
- **timestamp**: DateTime - When observation was made

### ObjectInfo
**Description**: Information about a detected object
- **name**: String - Object name/class
- **confidence**: Float (0.0-1.0) - Detection confidence
- **position**: Vector3 - 3D position relative to robot
- **bounding_box**: BoundingBox2D - 2D bounding box in image
- **properties**: Dictionary - Additional object properties

### BoundingBox2D
**Description**: 2D bounding box representation
- **x_min**: Float - Minimum X coordinate
- **y_min**: Float - Minimum Y coordinate
- **x_max**: Float - Maximum X coordinate
- **y_max**: Float - Maximum Y coordinate

### ActionPlan
**Description**: Sequence of actions to execute a command
- **plan_id**: String - Unique identifier for the plan
- **actions**: Array of ActionStep - Ordered sequence of actions
- **status**: Enum (PENDING, EXECUTING, COMPLETED, FAILED) - Current status
- **created_at**: DateTime - When plan was created
- **estimated_duration**: Float - Estimated time to complete (seconds)

### ActionStep
**Description**: Individual action in a plan
- **action_type**: Enum (NAVIGATE, GRASP, DETECT, SPEAK, WAIT) - Type of action
- **parameters**: Dictionary - Action-specific parameters
- **preconditions**: Array of String - Conditions that must be true
- **effects**: Array of String - Expected outcomes
- **timeout**: Float - Maximum time to complete (seconds)

### RobotState
**Description**: Current state of the robot
- **position**: Pose - Current position and orientation
- **battery_level**: Float (0.0-1.0) - Battery charge level
- **gripper_status**: Enum (OPEN, CLOSED, UNKNOWN) - Gripper state
- **current_task**: String - Currently executing task
- **sensors_active**: Dictionary - Status of various sensors
- **timestamp**: DateTime - When state was updated

## Relationships

### VLACommand → ActionPlan
- One VLACommand generates one ActionPlan
- ActionPlan references the VLACommand that created it

### VisionObservation → ObjectInfo
- One VisionObservation contains multiple ObjectInfo
- ObjectInfo belongs to one VisionObservation

### ActionPlan → ActionStep
- One ActionPlan contains multiple ActionStep
- ActionStep belongs to one ActionPlan

### RobotState → ActionStep
- ActionStep may depend on RobotState conditions
- ActionStep execution updates RobotState

## State Transitions

### ActionPlan States
- PENDING → EXECUTING (when execution starts)
- EXECUTING → COMPLETED (when all steps succeed)
- EXECUTING → FAILED (when a step fails)
- EXECUTING → PENDING (when paused)

### ActionStep States
- Implicit in ActionPlan status, but individual steps can be:
- Not started → In progress → Completed/Failed

## Validation Rules

### VLACommand Validation
- command_text must not be empty
- confidence must be between 0.0 and 1.0
- timestamp must be recent (within 5 minutes)

### VisionObservation Validation
- objects_detected array size must be reasonable (< 100 objects)
- timestamp must be recent (within 1 second)
- camera_feed must be valid image format

### ActionPlan Validation
- actions array must not be empty
- estimated_duration must be positive
- plan_id must be unique

### ActionStep Validation
- action_type must be valid enum value
- parameters must match action_type requirements
- timeout must be positive and reasonable

## ROS 2 Message Mappings

### VisionObservation → sensor_msgs/Image, sensor_msgs/PointCloud2, vision_msgs/Detection2DArray
### VLACommand → std_msgs/String, custom messages
### ActionStep → action_msgs/GoalStatus, custom action definitions
### RobotState → nav_msgs/Odometry, sensor_msgs/BatteryState, custom messages