---
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

## Introduction

In Chapter 1, you learned the conceptual foundations of ROS 2: nodes, topics, and services. Now it's time to put that knowledge into practice by writing Python code that creates functional ROS 2 applications.

The `rclpy` library is the official Python client library for ROS 2. It provides all the tools you need to create nodes, publish and subscribe to topics, create services, and interact with the ROS 2 ecosystem from Python (ROS 2 Documentation, 2024).

In this chapter, you'll learn how to:
- Set up a ROS 2 Python workspace
- Create a simple publisher node
- Create a subscriber node that receives messages
- Implement a service server and client
- Follow best practices for ROS 2 Python development

## Prerequisites

Before starting this chapter, ensure you have:

- **ROS 2 Humble or newer** installed on your system
- **Python 3.10 or newer** (included with ROS 2)
- A **text editor or IDE** (VS Code, PyCharm, or even nano/vim)
- Completed Chapter 1 or equivalent understanding of ROS 2 concepts

## ROS 2 Python Workspace Setup

While you can run simple ROS 2 Python scripts directly, it's best practice to organize your code in a properly structured ROS 2 package. However, for learning purposes, we'll start with standalone scripts and then show the package structure.

### Environment Setup

Every time you open a new terminal to work with ROS 2, you need to source the setup file:

```bash
# On Ubuntu/Linux
source /opt/ros/humble/setup.bash

# On Windows (adjust version as needed)
call C:\dev\ros2_humble\local_setup.bat
```

To avoid typing this every time, you can add it to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Verify Installation

Check that ROS 2 is properly installed:

```bash
ros2 --version
python3 --version
```

You should see ROS 2 Humble (or your installed version) and Python 3.10 or newer.

## Your First Node: A Simple Publisher

Let's create a node that publishes messages to a topic. We'll build a "heartbeat" node that sends a message every second.

### Code: Minimal Publisher

Create a file named `simple_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A simple publisher node that sends periodic heartbeat messages.

    This node demonstrates the basics of creating a ROS 2 node in Python
    and publishing messages to a topic.
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('minimal_publisher')

        # Create a publisher on the 'heartbeat' topic
        # Queue size of 10 means we'll buffer up to 10 messages
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)

        # Create a timer that calls our callback function every second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track how many messages we've sent
        self.count = 0

        self.get_logger().info('Minimal Publisher node has started')

    def timer_callback(self):
        """
        This function is called every second by the timer.
        It creates and publishes a message.
        """
        msg = String()
        msg.data = f'Heartbeat {self.count}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log what we published (visible in terminal)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """
    The main function that runs when the script is executed.
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our node
    node = MinimalPublisher()

    # Spin the node (keeps it running and processing callbacks)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully')
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Publisher

Make the script executable and run it:

```bash
chmod +x simple_publisher.py
python3 simple_publisher.py
```

You should see output like:

```
[INFO] [minimal_publisher]: Minimal Publisher node has started
[INFO] [minimal_publisher]: Publishing: "Heartbeat 0"
[INFO] [minimal_publisher]: Publishing: "Heartbeat 1"
[INFO] [minimal_publisher]: Publishing: "Heartbeat 2"
...
```

### Understanding the Code

Let's break down the key components:

1. **Import statements**: We import `rclpy` (the ROS 2 Python library), the `Node` base class, and the `String` message type.

2. **Node class**: Our `MinimalPublisher` class inherits from `Node`. This gives us access to all ROS 2 node functionality.

3. **`__init__` method**:
   - Calls `super().__init__()` with our node name
   - Creates a publisher with `self.create_publisher()`
   - Sets up a timer with `self.create_timer()`

4. **Callback function**: `timer_callback()` is executed every second by the timer. It creates a message, fills in the data, and publishes it.

5. **Main function**:
   - Initializes rclpy
   - Creates our node
   - Calls `rclpy.spin()` to keep the node running
   - Handles cleanup on shutdown

### Inspecting the Running Node

While your publisher is running, open another terminal and try these commands:

```bash
# Source ROS 2 first
source /opt/ros/humble/setup.bash

# List all running nodes
ros2 node list

# See information about our node
ros2 node info /minimal_publisher

# List all active topics
ros2 topic list

# Echo messages being published
ros2 topic echo /heartbeat
```

This demonstrates that your Python code is a full-fledged part of the ROS 2 ecosystem!

## Creating a Subscriber Node

Now let's create a node that subscribes to the `heartbeat` topic and receives the messages our publisher sends.

### Code: Minimal Subscriber

Create a file named `simple_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A simple subscriber node that listens for heartbeat messages.

    This node demonstrates how to subscribe to a topic and process
    incoming messages.
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'heartbeat' topic
        # The message type must match the publisher (String)
        # Queue size of 10
        self.subscription = self.create_subscription(
            String,
            'heartbeat',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        # (Python convention for intentionally unused variables)
        self.subscription

        self.get_logger().info('Minimal Subscriber node has started')

    def listener_callback(self, msg):
        """
        This function is called every time a message arrives on the topic.

        Args:
            msg: The message received (String type in this case)
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running Publisher and Subscriber Together

Now you can see the power of ROS 2's communication system:

1. In terminal 1: Run the publisher
   ```bash
   python3 simple_publisher.py
   ```

2. In terminal 2: Run the subscriber
   ```bash
   python3 simple_subscriber.py
   ```

You should see the subscriber receiving the messages published by the first node:

```
[INFO] [minimal_subscriber]: Minimal Subscriber node has started
[INFO] [minimal_subscriber]: I heard: "Heartbeat 12"
[INFO] [minimal_subscriber]: I heard: "Heartbeat 13"
[INFO] [minimal_subscriber]: I heard: "Heartbeat 14"
...
```

The two nodes are communicating through the ROS 2 middleware, and they don't even know about each other directly!

## Practical Example: A Robot Velocity Commander

Let's build something more practical: a node that publishes velocity commands for a robot. This is the kind of code you'd use to control a mobile robot.

### Code: Velocity Commander

Create `velocity_commander.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VelocityCommander(Node):
    """
    Publishes velocity commands to make a robot move in a circle.

    This demonstrates using standard ROS 2 message types for robot control.
    """

    def __init__(self):
        super().__init__('velocity_commander')

        # Publisher for velocity commands (standard topic name for mobile robots)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish commands at 10 Hz (every 0.1 seconds)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_velocity)

        # Keep track of time for creating smooth motions
        self.time = 0.0

        self.get_logger().info('Velocity Commander started - robot will move in a circle')

    def publish_velocity(self):
        """
        Publishes velocity commands to make the robot drive in a circle.
        """
        msg = Twist()

        # Twist message has two Vector3 fields: linear and angular
        # For a 2D mobile robot, we primarily use:
        # - linear.x: forward/backward speed
        # - angular.z: rotational speed

        # Move forward at 0.5 m/s
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Rotate at 0.5 rad/s (this creates a circular path)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        self.publisher_.publish(msg)

        # Log occasionally (not every message, that would be too much)
        if int(self.time) % 2 == 0 and self.time < int(self.time) + 0.1:
            self.get_logger().info(f'Publishing velocity: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

        self.time += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # When stopping, publish zero velocity to stop the robot
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.get_logger().info('Sent stop command, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Understanding Twist Messages

The `geometry_msgs/Twist` message is the standard way to command velocities for mobile robots. It contains:

```
Vector3 linear   # Linear velocity (m/s)
  float64 x      # Forward/backward
  float64 y      # Left/right (usually 0 for wheeled robots)
  float64 z      # Up/down (usually 0 for ground robots)

Vector3 angular  # Angular velocity (rad/s)
  float64 x      # Roll rate (usually 0)
  float64 y      # Pitch rate (usually 0)
  float64 z      # Yaw rate (rotation in place)
```

For a differential drive robot (like a TurtleBot), you typically only use `linear.x` and `angular.z`.

## Implementing a Service

Services are perfect for operations that need a response. Let's create a simple service that adds two numbers.

### Code: Service Server

Create `add_two_ints_server.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    A service server that adds two integers and returns the result.

    This demonstrates the request-response pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create a service named 'add_two_ints'
        # The service type is AddTwoInts (request: two int64, response: one int64)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints service is ready')

    def add_two_ints_callback(self, request, response):
        """
        This function is called when a client sends a request.

        Args:
            request: Contains two int64 values (a and b)
            response: Object where we set the result

        Returns:
            The response object with the sum
        """
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Received request: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down service server')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Code: Service Client

Create `add_two_ints_client.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class AddTwoIntsClient(Node):
    """
    A service client that calls the add_two_ints service.
    """

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create a client for the 'add_two_ints' service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Connected to add_two_ints service')

    def send_request(self, a, b):
        """
        Send a request to add two numbers.

        Args:
            a: First number
            b: Second number

        Returns:
            The sum from the service response
        """
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        future = self.client.call_async(request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)

    # Get numbers from command line arguments
    if len(sys.argv) != 3:
        print('Usage: python3 add_two_ints_client.py <num1> <num2>')
        return

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Arguments must be integers')
        return

    node = AddTwoIntsClient()
    node.send_request(a, b)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Testing the Service

1. In terminal 1, start the server:
   ```bash
   python3 add_two_ints_server.py
   ```

2. In terminal 2, call the service:
   ```bash
   python3 add_two_ints_client.py 5 7
   ```

You should see:
```
[INFO] [add_two_ints_client]: Connected to add_two_ints service
[INFO] [add_two_ints_client]: Result: 5 + 7 = 12
```

And on the server side:
```
[INFO] [add_two_ints_server]: Received request: 5 + 7 = 12
```

## Best Practices for ROS 2 Python Development

As you develop more complex ROS 2 applications, keep these best practices in mind:

### 1. Use Class-Based Nodes

Always structure your nodes as classes that inherit from `Node`. This provides clean encapsulation and access to all node functionality.

### 2. Handle Shutdown Gracefully

Use try-except blocks to catch `KeyboardInterrupt` and clean up properly:

```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    node.get_logger().info('Shutting down gracefully')
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### 3. Use Logging Instead of Print

Always use the node's logger instead of `print()`:

```python
# Good
self.get_logger().info('Node started')
self.get_logger().warn('Low battery')
self.get_logger().error('Sensor failure')

# Bad
print('Node started')
```

Logging integrates with ROS 2's logging system and can be filtered, redirected, and configured.

### 4. Choose Appropriate Timer Rates

For publishers, choose timer rates appropriate to your data:
- Sensor data: 10-100 Hz (0.01-0.1 second periods)
- Status updates: 1-10 Hz (0.1-1 second periods)
- Heartbeats: 1 Hz (1 second period)

### 5. Specify Queue Sizes Thoughtfully

The queue size parameter determines how many messages are buffered:
- For real-time control: small queues (1-10)
- For data logging: larger queues (100+)
- For occasional updates: 10 is usually fine

### 6. Use Standard Message Types

Whenever possible, use standard message types from packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`. This makes your nodes interoperable with others in the ROS ecosystem.

## Summary

In this chapter, you learned how to:

- Set up a ROS 2 Python development environment
- Create publisher nodes that send messages to topics
- Create subscriber nodes that receive messages
- Implement service servers that respond to requests
- Create service clients that call services
- Use standard message types like `String`, `Twist`, and `AddTwoInts`
- Follow best practices for ROS 2 Python development

You now have practical experience creating the building blocks of ROS 2 applications. These patterns—publishers, subscribers, services—form the foundation of every ROS 2 robot system.

In the next chapter, we'll explore how to model robot structures using URDF, allowing you to describe your robot's physical form and visualize it in simulation tools.

## Further Reading

- rclpy API Documentation: https://docs.ros.org/en/humble/p/rclpy/
- ROS 2 Python Tutorials: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Standard ROS 2 Message Types: https://github.com/ros2/common_interfaces
- Python Best Practices for ROS 2: https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html

---

**Learning Check**: Before moving on, make sure you can:
- ✓ Create a Python node that publishes to a topic
- ✓ Create a Python node that subscribes to a topic
- ✓ Implement a service server and client
- ✓ Use standard message types like Twist
- ✓ Explain the purpose of rclpy.spin()
