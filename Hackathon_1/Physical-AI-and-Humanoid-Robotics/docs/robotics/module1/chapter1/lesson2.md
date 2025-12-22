# Lesson 2: ROS 2 Topics

## Introduction to ROS 2 Topics

Topics in ROS 2 provide a way for nodes to communicate with each other through a publish-subscribe communication pattern. This is one of the fundamental communication mechanisms in ROS 2.

## What are Topics?

A topic in ROS 2 is:
- A named bus over which nodes exchange messages
- The basis for asynchronous communication between nodes
- Identified by a unique name within the ROS 2 graph
- Unidirectional: publishers send messages, subscribers receive them

## Publisher-Subscriber Pattern

In ROS 2's publish-subscribe model:
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic
- Publishers and subscribers are decoupled - they don't need to know about each other

## Creating a Publisher

Here's how to create a publisher in Python using `rclpy`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber

Here's how to create a subscriber in Python using `rclpy`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topic Communication Characteristics

1. **Asynchronous**: Publishers and subscribers don't need to run simultaneously
2. **Unidirectional**: Data flows from publisher to subscriber
3. **Anonymous**: Publishers and subscribers are unaware of each other
4. **Loosely coupled**: Changes to one node don't affect others (as long as interfaces remain the same)

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service settings to configure how messages are delivered:

- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep last N messages or keep all messages

## Best Practices

- Use descriptive topic names
- Follow naming conventions (e.g., lowercase with underscores)
- Consider QoS settings for your specific use case
- Ensure message types are compatible between publishers and subscribers

## Summary

Topics enable asynchronous communication between ROS 2 nodes using the publish-subscribe pattern. Understanding topics is essential for creating distributed robotic systems. In the next lesson, we'll explore services, which provide synchronous request-response communication.

## Exercises

1. Create a publisher that publishes temperature readings every 2 seconds.
2. Create a subscriber that receives the temperature readings and logs them.
3. Explain when you would use topics versus services for communication.