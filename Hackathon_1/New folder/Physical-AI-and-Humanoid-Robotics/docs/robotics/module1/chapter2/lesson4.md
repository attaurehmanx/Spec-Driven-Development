# Lesson 4: Exercises & Quiz - Python Agents Controlling ROS Systems

## Practical Exercises

### Exercise 1: Temperature Publisher and Subscriber
Create a publisher that publishes temperature readings every 2 seconds and a subscriber that receives and logs these readings.

**Steps:**
1. Create a publisher node that publishes `std_msgs/msg/Float64` messages with temperature values
2. Include a timestamp in the message or as a parameter
3. Create a subscriber node that listens to the temperature topic
4. The subscriber should log the temperature and trigger warnings for extreme values
5. Run both nodes and verify they communicate properly

### Exercise 2: Robot Control Service
Create a service that accepts robot movement commands (linear and angular velocity) and returns whether the command was accepted.

**Steps:**
1. Define a service file (`.srv`) that takes linear and angular velocities and returns success status
2. Implement the service server that validates and processes movement commands
3. Implement the service client that sends various movement commands
4. Test the service functionality with different command values

### Exercise 3: Multi-Node System
Create a system with:
- A sensor node that publishes random sensor readings
- A processing node that subscribes to sensor data, processes it, and publishes processed data
- A display node that subscribes to processed data and prints it to console

## Chapter Quiz

### Question 1
What is the correct way to create a publisher in rclpy?

A) `self.publisher = Publisher(String, 'topic', 10)`
B) `self.publisher = self.create_publisher(String, 'topic', 10)`
C) `self.publisher = rclpy.create_publisher(String, 'topic', 10)`
D) `self.publisher = Node.create_publisher(String, 'topic', 10)`

**Answer:** B

### Question 2
How do you properly initialize ROS 2 in a Python script?

A) `rclpy.start()`
B) `rclpy.init()`
C) `rclpy.begin()`
D) `rclpy.setup()`

**Answer:** B

### Question 3
What happens when you call `rclpy.spin(node)`?

A) The node is destroyed
B) The node runs and processes callbacks until shut down
C) The node publishes a message
D) The node creates a timer

**Answer:** B

### Question 4
In a subscriber callback, what does the parameter represent?

A) The topic name
B) The node instance
C) The received message
D) A timer object

**Answer:** C

### Question 5
How do you wait for a service to be available before making a call?

A) `while not service.available():`
B) `service.wait_for_availability()`
C) `while not client.wait_for_service(timeout_sec=1.0):`
D) `client.wait_for_service()`

**Answer:** C

## Hands-on Challenge

Create a complete ROS 2 system that simulates a simple robot control system:
1. A sensor node that publishes random battery level readings
2. A controller node that subscribes to battery levels and publishes motor commands when battery is low
3. A service server that can be called to get the current robot status
4. A client node that periodically calls the status service and logs the result

## Summary

This chapter covered the practical aspects of using `rclpy` to create Python agents that control ROS systems:
- Creating publishers to send data to topics
- Creating subscribers to receive data from topics
- Implementing service servers for request-response communication
- Creating service clients to make requests

These concepts enable the development of sophisticated ROS 2 systems where Python agents can control and interact with robotic components. The next chapter will explore URDF and humanoid robot modeling.