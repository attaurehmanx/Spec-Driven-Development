# Lesson 3: Subscribing and Services with rclpy

## Introduction to Subscribing in rclpy

Subscribing in ROS 2 using `rclpy` involves creating subscribers that receive messages from topics. This lesson covers how to create subscribers, handle incoming messages, and implement services for request-response communication.

## Creating a Subscriber

To create a subscriber in `rclpy`, use the `create_subscription()` method:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create a subscription
        # Parameters: message_type, topic_name, callback, qos_profile
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        # This callback is called when a message is received
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

## Subscriber Callback Functions

The callback function receives messages of the specified type:

```python
def listener_callback(self, msg):
    # msg is an instance of the message type (e.g., String, Int32, etc.)
    # Process the message data
    self.get_logger().info(f'Received message: {msg.data}')

    # Perform any necessary processing
    self.process_message(msg)
```

## Subscribing to Different Message Types

### Subscribing to Integer Messages

```python
from std_msgs.msg import Int32

class IntegerSubscriber(Node):
    def __init__(self):
        super().__init__('integer_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'integer_topic',
            self.integer_callback,
            10)
        self.subscription

    def integer_callback(self, msg):
        self.get_logger().info(f'Received integer: {msg.data}')
        # Process the integer value
        if msg.data > 100:
            self.get_logger().warn('Integer value is high!')
```

### Subscribing with Custom QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CustomQosSubscriber(Node):
    def __init__(self):
        super().__init__('custom_qos_subscriber')

        # Custom QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            qos_profile)
        self.subscription

    def reliable_callback(self, msg):
        self.get_logger().info(f'Received reliably: {msg.data}')
```

## Creating Service Servers

Services provide synchronous request-response communication:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service server
        # Parameters: service_type, service_name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # Process the request and populate the response
        response.sum = request.a + request.b
        self.get_logger().info(
            'Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Service Clients

Service clients make requests to service servers:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')

        # Create a service client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create a request object
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Make an asynchronous service call
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    # Send a request and get the response
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Client with Callbacks

For non-blocking service calls, use callbacks:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientCallback(Node):
    def __init__(self):
        super().__init__('minimal_client_callback')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_async_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Make an asynchronous service call with a callback
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = MinimalClientCallback()

    # Send request and continue execution
    client.send_async_request(5, 7)

    # Spin to process callbacks
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Best Practices

### 1. Store Subscription References
Keep a reference to your subscription to prevent garbage collection:

```python
def __init__(self):
    super().__init__('subscriber_node')
    self.subscription = self.create_subscription(String, 'topic', self.callback, 10)
    # Store as instance variable to prevent garbage collection
```

### 2. Use Appropriate QoS Settings
Match QoS settings with the publisher:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# If the publisher uses reliable delivery, match it
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)

self.subscription = self.create_subscription(
    String, 'topic', self.callback, qos_profile)
```

### 3. Handle Message Processing Efficiently
Avoid heavy processing in callbacks to prevent blocking:

```python
def listener_callback(self, msg):
    # Store message for later processing instead of heavy processing here
    self.received_messages.append(msg)

    # Or use threading for heavy processing
    # self.process_message_in_thread(msg)
```

## Service Best Practices

### 1. Handle Service Exceptions
Wrap service callbacks in try-catch blocks:

```python
def service_callback(self, request, response):
    try:
        # Process the request
        result = self.calculate_result(request)
        response.result = result
        return response
    except Exception as e:
        self.get_logger().error(f'Service error: {e}')
        # Return appropriate error response
        response.error = str(e)
        return response
```

### 2. Use Appropriate Service Timeouts
Set timeouts when calling services:

```python
def call_with_timeout(self):
    future = self.cli.call_async(self.req)

    # Wait for result with timeout
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    if future.done():
        return future.result()
    else:
        self.get_logger().warn('Service call timed out')
        return None
```

## Summary

Subscribing and services in `rclpy` provide both asynchronous and synchronous communication patterns in ROS 2. Subscribers handle incoming messages through callbacks, while services provide request-response communication. Understanding both patterns is essential for building complete ROS 2 applications. In the next lesson, we'll explore exercises and quizzes to reinforce these concepts.

## Exercises

1. Create a subscriber that listens to a topic and logs the received messages.
2. Implement a service that converts temperatures from Celsius to Fahrenheit.
3. Create a client that calls the temperature conversion service with different values.