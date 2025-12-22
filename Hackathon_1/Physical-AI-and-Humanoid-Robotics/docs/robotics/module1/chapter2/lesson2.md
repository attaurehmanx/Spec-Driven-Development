# Lesson 2: Publishing with rclpy

## Introduction to Publishing in rclpy

Publishing in ROS 2 using `rclpy` involves creating publishers that send messages to topics. This lesson covers how to create publishers, define message types, and publish data to the ROS 2 graph.

## Creating a Publisher

To create a publisher in `rclpy`, use the `create_publisher()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Create a publisher
        # Parameters: message_type, topic_name, qos_profile
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)  # Publish the message
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

## Message Types

ROS 2 provides various standard message types in packages like `std_msgs`, `geometry_msgs`, etc. Common message types include:

- `std_msgs.msg.String` - String messages
- `std_msgs.msg.Int32` - 32-bit integer messages
- `std_msgs.msg.Float64` - 64-bit float messages
- `geometry_msgs.msg.Twist` - Velocity commands for robots
- `sensor_msgs.msg.LaserScan` - Laser scan data

## Quality of Service (QoS) Profiles

When creating a publisher, you specify a QoS profile that determines how messages are delivered:

```python
from rclpy.qos import QoSProfile

# Basic QoS profile with history depth
publisher = self.create_publisher(String, 'topic', 10)

# Custom QoS profile
qos_profile = QoSProfile(depth=10)
publisher = self.create_publisher(String, 'topic', qos_profile)

# Reliable delivery QoS
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
```

## Publishing Different Message Types

### Publishing Integer Messages

```python
from std_msgs.msg import Int32

class IntegerPublisher(Node):
    def __init__(self):
        super().__init__('integer_publisher')
        self.publisher_ = self.create_publisher(Int32, 'integer_topic', 10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_integer)
        self.value = 0

    def publish_integer(self):
        msg = Int32()
        msg.data = self.value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing integer: %d' % msg.data)
        self.value += 1
```

### Publishing Custom Messages

If you have custom message types, import them from your package:

```python
from my_robot_msgs.msg import SensorData  # Custom message

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(SensorData, 'sensor_data', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

    def publish_sensor_data(self):
        msg = SensorData()
        msg.temperature = 25.0
        msg.humidity = 60.0
        msg.timestamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
```

## Publisher Best Practices

### 1. Store Publisher References
Keep a reference to your publisher to prevent garbage collection:

```python
def __init__(self):
    super().__init__('publisher_node')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    # Store as instance variable
```

### 2. Use Appropriate QoS Settings
Choose QoS settings based on your application's needs:

```python
# For real-time control
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
real_time_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For logging
logging_qos = QoSProfile(
    depth=1000,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

### 3. Handle Publisher Liveliness
Check if there are subscribers before publishing expensive data:

```python
def timer_callback(self):
    if self.publisher_.get_subscription_count() > 0:
        # Only publish if there are subscribers
        msg = self.generate_expensive_message()
        self.publisher_.publish(msg)
```

## Publishing with Parameters

Use ROS 2 parameters to configure publishing behavior:

```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare parameters
        self.declare_parameter('topic_name', 'default_topic')
        self.declare_parameter('publish_rate', 1.0)

        topic_name = self.get_parameter('topic_name').value
        publish_rate = self.get_parameter('publish_rate').value

        self.publisher_ = self.create_publisher(String, topic_name, 10)

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Parameter-controlled message'
        self.publisher_.publish(msg)
```

## Summary

Publishing with `rclpy` involves creating publishers, defining appropriate message types, and publishing messages to topics. Understanding QoS profiles and best practices for publisher management is essential for creating robust ROS 2 systems. In the next lesson, we'll explore subscribing and services with `rclpy`.

## Exercises

1. Create a publisher that publishes the current system time every second.
2. Create a publisher that uses parameters to control the topic name and publish rate.
3. Explain the difference between RELIABLE and BEST_EFFORT QoS reliability policies.