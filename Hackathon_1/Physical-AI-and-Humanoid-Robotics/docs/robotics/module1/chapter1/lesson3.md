# Lesson 3: ROS 2 Services

## Introduction to ROS 2 Services

Services in ROS 2 provide a way for nodes to communicate using a request-response pattern. Unlike topics which provide asynchronous communication, services offer synchronous communication where a client sends a request and waits for a response from a server.

## What are Services?

A service in ROS 2 is:
- A named interface for synchronous request-response communication
- Composed of a request message and a response message
- Implemented as a server that processes requests and a client that makes requests
- Identified by a unique name within the ROS 2 graph

## Service Architecture

In ROS 2's request-response model:
- **Service Servers** provide a specific functionality and respond to requests
- **Service Clients** make requests to servers and wait for responses
- Communication is synchronous - the client waits for the response
- Services are typically used for operations that have a clear beginning and end

## Creating a Service Server

First, you need a service definition file (e.g., `AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

Here's how to create a service server in Python using `rclpy`:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Here's how to create a service client in Python using `rclpy`:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Communication Characteristics

1. **Synchronous**: Client waits for response from server
2. **Bidirectional**: Request goes from client to server, response goes from server to client
3. **Tightly coupled**: Client and server must be available for communication
4. **Stateless**: Each request is independent of others

## When to Use Services

Services are appropriate when:
- You need a synchronous request-response pattern
- The operation has a clear beginning and end
- You need to ensure that a request is processed before continuing
- The operation is not time-critical and blocking is acceptable

## Best Practices

- Use services for operations that have a clear request-response pattern
- Handle service call failures gracefully
- Use appropriate service definition files (.srv) for type safety
- Consider timeouts when making service calls

## Summary

Services provide synchronous request-response communication in ROS 2, complementing the asynchronous communication provided by topics. Understanding when to use services versus topics is crucial for effective ROS 2 system design. In the next lesson, we'll explore exercises and quizzes to reinforce these concepts.

## Exercises

1. Create a service that converts temperatures between Celsius and Fahrenheit.
2. Implement both the service server and client for the temperature conversion.
3. Explain the difference between using topics and services for the same functionality.