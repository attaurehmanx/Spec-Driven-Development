#!/usr/bin/env python3
"""
Service Client Example for Python Agents Controlling ROS Systems

This example demonstrates how to create a service client that calls
a service to perform operations in ROS 2 using rclpy.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClientExample(Node):
    def __init__(self):
        super().__init__('service_client_example')

        # Create a client for the 'add_two_ints' service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send a request to the service and return the response."""
        self.request.a = a
        self.request.b = b

        # Make an asynchronous service call
        future = self.client.call_async(self.request)
        return future


def main(args=None):
    rclpy.init(args=args)
    client_example = ServiceClientExample()

    # Example 1: Simple synchronous call
    future = client_example.send_request(3, 4)

    # Wait for the result
    rclpy.spin_until_future_complete(client_example, future)

    try:
        response = future.result()
        client_example.get_logger().info(
            f'Result of {client_example.request.a} + {client_example.request.b} = {response.sum}')
    except Exception as e:
        client_example.get_logger().error(f'Service call failed: {e}')

    # Example 2: Multiple requests
    test_values = [(1, 2), (5, 7), (10, -3), (0, 0)]

    for a, b in test_values:
        future = client_example.send_request(a, b)
        rclpy.spin_until_future_complete(client_example, future)

        try:
            response = future.result()
            client_example.get_logger().info(
                f'{a} + {b} = {response.sum}')
        except Exception as e:
            client_example.get_logger().error(f'Service call failed for {a}, {b}: {e}')

    # Example 3: Asynchronous calls with callbacks
    # Create another client for callback example
    callback_client = ServiceClientExample()

    # Function to create callback for each request
    def create_callback(a, b):
        def callback(future):
            try:
                response = future.result()
                callback_client.get_logger().info(
                    f'Async result: {a} + {b} = {response.sum}')
            except Exception as e:
                callback_client.get_logger().error(f'Async call failed: {e}')
        return callback

    # Make several async requests
    for a, b in [(15, 25), (100, 1), (-5, 8)]:
        future = callback_client.send_request(a, b)
        future.add_done_callback(create_callback(a, b))

    # Spin briefly to process the async callbacks
    from time import time
    start_time = time()
    while time() - start_time < 2.0:  # Wait up to 2 seconds for callbacks
        rclpy.spin_once(callback_client, timeout_sec=0.1)

    # Clean up
    client_example.destroy_node()
    callback_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()