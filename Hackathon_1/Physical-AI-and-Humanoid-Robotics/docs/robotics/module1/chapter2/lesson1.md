# Lesson 1: rclpy Basics

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides a Python API for creating ROS 2 nodes, publishers, subscribers, services, and other ROS 2 entities. This lesson covers the fundamental concepts and patterns used in `rclpy`.

## What is rclpy?

`rclpy` is:
- The Python client library for ROS 2
- Part of the ROS 2 client library ecosystem
- Provides Python bindings to the ROS 2 middleware
- The primary way to create ROS 2 nodes in Python

## Basic Node Structure

Every ROS 2 Python node follows a similar structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 communications
    node = MyNode()        # Create node
    rclpy.spin(node)       # Keep node alive
    node.destroy_node()    # Destroy node explicitly
    rclpy.shutdown()       # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()
```

## Essential rclpy Functions

### rclpy.init()
- Initializes the ROS 2 client library
- Must be called before creating any nodes
- Handles command-line arguments

### rclpy.spin()
- Keeps the node running and processing callbacks
- Blocks until the node is shut down
- Processes incoming messages and service requests

### rclpy.shutdown()
- Cleans up ROS 2 resources
- Should be called when the program exits

## Creating a Node Class

The recommended approach is to subclass `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        # Initialize the parent class with a node name
        super().__init__('minimal_node')

        # Log a message to indicate the node has started
        self.get_logger().info('Minimal node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Parameters

`rclpy` provides built-in support for parameters:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('frequency', 1.0)

        # Get parameter values
        my_param = self.get_parameter('my_param').value
        frequency = self.get_parameter('frequency').value

        self.get_logger().info(f'My param: {my_param}, Frequency: {frequency}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Timer Usage

Timers are commonly used to execute code at regular intervals:

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer that calls timer_callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Timer callback: %d' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Best Practices

### Proper Resource Management
Always properly destroy nodes and shut down ROS 2:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Ensure cleanup happens
        node.destroy_node()
        rclpy.shutdown()
```

### Logging
Use the node's built-in logger for messages:

```python
self.get_logger().info('Informational message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().debug('Debug message')
```

## Summary

`rclpy` provides the foundation for creating ROS 2 nodes in Python. Understanding the basic node structure, initialization process, and resource management is crucial for building robust ROS 2 applications. In the next lesson, we'll explore how to use `rclpy` for publishing messages.

## Exercises

1. Create a simple node that prints "Hello from rclpy" when it starts.
2. Create a node that declares a parameter and logs its value.
3. Explain the purpose of `rclpy.init()` and `rclpy.shutdown()`.