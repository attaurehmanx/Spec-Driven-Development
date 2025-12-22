# Lesson 1: ROS 2 Nodes

## Introduction to ROS 2 Nodes

In ROS 2 (Robot Operating System 2), a node is a fundamental component that performs computation. Nodes are the basic unit of execution in ROS 2, and they can be thought of as processes that perform specific tasks within a robotic system.

## What is a Node?

A node in ROS 2 is:
- A process that performs computation
- The basic unit of execution in ROS 2
- A container for publishers, subscribers, services, and other ROS 2 entities
- Identified by a unique name within the ROS 2 graph

## Key Concepts

### Node Creation
Nodes in ROS 2 are typically created using client libraries like `rclpy` (Python) or `rclcpp` (C++). Here's a basic example of creating a node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code goes here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    # Node work happens here

    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle
- Nodes are initialized with `rclpy.init()`
- Each node has a unique name within the ROS 2 graph
- Nodes can contain publishers, subscribers, services, and parameters
- Nodes are destroyed when no longer needed with `destroy_node()`

## ROS 2 Node Characteristics

1. **Communication**: Nodes communicate with each other through topics, services, and actions
2. **Modularity**: Each node performs a specific task, promoting modularity
3. **Distributed**: Nodes can run on the same or different machines
4. **Discovery**: Nodes automatically discover each other on the same ROS 2 domain

## Best Practices

- Use descriptive names for your nodes
- Initialize nodes properly with error handling
- Clean up resources when the node is destroyed
- Use parameters for configurable values

## Summary

ROS 2 nodes form the foundation of the ROS 2 architecture. Understanding nodes is crucial for building any ROS 2-based robotic system. In the next lesson, we'll explore topics and how nodes communicate through them.

## Exercises

1. Create a simple node that prints "Hello from ROS 2 Node" when initialized.
2. Explain the difference between a node in ROS 1 versus ROS 2.