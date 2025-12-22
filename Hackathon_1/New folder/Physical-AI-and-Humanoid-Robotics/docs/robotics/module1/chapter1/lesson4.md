# Lesson 4: Exercises & Quiz - ROS 2 Communication Basics

## Practical Exercises

### Exercise 1: Publisher-Subscriber Pair
Create a publisher that publishes messages containing the current time every second. Create a subscriber that receives these messages and prints them to the console.

**Steps:**
1. Create a publisher node that publishes `std_msgs/msg/String` messages
2. Include the current timestamp in each message
3. Create a subscriber node that listens to the same topic
4. Run both nodes and verify they communicate properly

### Exercise 2: Service for Simple Calculation
Create a service that takes two numbers as input and returns their product. Implement both the server and client.

**Steps:**
1. Define a service file (`.srv`) that takes two floats and returns their product
2. Implement the service server
3. Implement the service client that calls the service with sample values
4. Test the service functionality

### Exercise 3: Node with Multiple Publishers and Subscribers
Create a single node that:
- Publishes sensor readings
- Subscribes to a command topic
- Prints received commands to the console

## Chapter Quiz

### Question 1
What is the primary difference between topics and services in ROS 2?

A) Topics are for communication between nodes, services are for communication within a node
B) Topics provide asynchronous communication, services provide synchronous communication
C) Topics can only be used by C++ nodes, services can only be used by Python nodes
D) Topics are faster than services

**Answer:** B

### Question 2
Which of the following is true about ROS 2 nodes?

A) A node can only have one publisher
B) A node cannot contain both publishers and subscribers
C) A node is the basic unit of execution in ROS 2
D) Nodes must be written in the same programming language

**Answer:** C

### Question 3
In the publisher-subscriber pattern:

A) Publishers must know which subscribers exist
B) Subscribers must know which publishers exist
C) Publishers and subscribers are decoupled
D) Communication is synchronous

**Answer:** C

### Question 4
What happens when a service client calls a service in ROS 2?

A) The client continues execution immediately
B) The client waits for the server's response
C) The server runs in a separate thread automatically
D) The communication is asynchronous

**Answer:** B

### Question 5
Which Quality of Service (QoS) setting determines whether messages are delivered reliably?

A) History
B) Durability
C) Reliability
D) Deadline

**Answer:** C

## Hands-on Challenge

Create a complete ROS 2 system that includes:
1. A node with a publisher that publishes random temperature readings
2. A node with a subscriber that receives these readings and logs them
3. A service server that can be called to get the last received temperature
4. A service client that calls the service and prints the result

## Summary

This chapter covered the fundamental communication patterns in ROS 2:
- Nodes as the basic execution units
- Topics for asynchronous publish-subscribe communication
- Services for synchronous request-response communication

Understanding these concepts is essential for building more complex ROS 2 systems. The next chapter will explore how to write Python agents that control ROS systems using the `rclpy` library.