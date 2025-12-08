---
sidebar_position: 1
---

# Chapter 1: ROS 2 Basics - Nodes, Topics, and Services

## Introduction

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. Despite its name, ROS 2 is not an operating system in the traditional sense. Instead, it's a middleware—a layer of software that sits between the operating system and your application code, providing tools and libraries for building robot applications (Macenski et al., 2022).

ROS 2 is the successor to ROS 1, redesigned from the ground up to address limitations in real-time systems, security, and multi-robot communication. It has become the de facto standard for robotics research and is increasingly adopted in commercial robotics applications (Maruyama et al., 2016).

## Core Architecture

At its heart, ROS 2 is a distributed communication system. A robot application is typically composed of many small, specialized programs running simultaneously. These programs need to exchange data efficiently and reliably. ROS 2 provides the infrastructure to make this communication seamless.

The fundamental building blocks of any ROS 2 system are:

1. **Nodes** - Independent processes that perform computation
2. **Topics** - Named channels for streaming data
3. **Services** - Request-response communication mechanisms
4. **Messages** - Data structures sent between nodes

Let's explore each of these concepts in detail.

## ROS 2 Nodes

### What is a Node?

A **node** is a single executable process in a ROS 2 system. Think of nodes as individual specialists, each responsible for a specific task. For example, in a mobile robot system, you might have:

- A node that reads data from a laser scanner
- A node that processes camera images
- A node that plans navigation paths
- A node that controls wheel motors

Each node runs independently and can be started, stopped, or restarted without affecting other nodes (assuming proper error handling). This modularity makes ROS 2 systems easier to develop, test, and maintain.

### Why Multiple Nodes?

You might wonder: why not write one large program instead of many small nodes? The multi-node architecture provides several advantages:

1. **Modularity**: Each node has a single, well-defined responsibility
2. **Reusability**: Nodes can be reused across different robot projects
3. **Fault Isolation**: If one node crashes, others can continue running
4. **Parallel Execution**: Nodes can run on different CPU cores or even different computers
5. **Language Independence**: Different nodes can be written in different programming languages

### Node Naming

Every node in a ROS 2 system must have a unique name. Node names use a hierarchical naming scheme with namespaces separated by forward slashes. For example:

- `/camera_node`
- `/robot1/sensor_processor`
- `/navigation/path_planner`

Namespaces help organize nodes in complex systems with many components.

## Topics: Publish-Subscribe Communication

### The Publish-Subscribe Pattern

**Topics** implement a publish-subscribe communication pattern. This is an asynchronous, many-to-many communication model where:

- **Publishers** send messages to a named topic without knowing who (if anyone) is listening
- **Subscribers** receive messages from a named topic without knowing who sent them
- Any number of nodes can publish to the same topic
- Any number of nodes can subscribe to the same topic

This loose coupling between publishers and subscribers makes systems more flexible. You can add new subscribers without modifying publishers, or swap out a publisher without affecting subscribers.

### How Topics Work

When a node wants to send data, it creates a **publisher** on a specific topic. The publisher specifies:

1. The topic name (e.g., `/robot/velocity`)
2. The message type (e.g., `geometry_msgs/Twist`)

When a node wants to receive data, it creates a **subscriber** to a specific topic, specifying the same information.

ROS 2's middleware layer (built on DDS - Data Distribution Service) handles all the networking details. Publishers and subscribers can be on the same computer or on different computers on the same network—the code looks identical.

### Topic Naming Conventions

Topic names follow similar conventions to node names:

- Use lowercase letters and underscores
- Use forward slashes for namespaces
- Make names descriptive but concise

Examples:
- `/camera/image_raw`
- `/robot/odometry`
- `/sensors/lidar/scan`

### When to Use Topics

Topics are ideal for:

- **Streaming sensor data** (camera images, laser scans, IMU readings)
- **Continuous state information** (robot position, battery level)
- **Command streams** (velocity commands to motors)
- **Any data that updates frequently**

The publish-subscribe pattern works well when you don't need confirmation that a message was received, and when data is produced and consumed at regular intervals.

## Services: Request-Response Communication

### The Request-Response Pattern

While topics are great for streaming data, sometimes you need a different communication pattern. **Services** implement request-response communication, similar to function calls in programming or HTTP requests on the web.

A service interaction has three parts:

1. A **client** node sends a **request** message
2. A **service server** node receives the request, performs some computation
3. The server sends a **response** message back to the client

Unlike topics, service communication is:

- **Synchronous**: The client waits for a response (with an optional timeout)
- **One-to-one**: Each request goes to exactly one server
- **Bidirectional**: Information flows both ways

### How Services Work

A node that wants to provide a service creates a **service server**. The server specifies:

1. The service name (e.g., `/add_two_ints`)
2. The service type (e.g., `example_interfaces/AddTwoInts`)

The service type defines both the request message structure and the response message structure.

A node that wants to use a service creates a **service client**. When the client calls the service, execution blocks until the response arrives or a timeout occurs.

### When to Use Services

Services are ideal for:

- **Infrequent operations** (triggering a camera capture, resetting odometry)
- **Configuration queries** (getting parameter values, checking system status)
- **Computations that produce a result** (inverse kinematics, path planning requests)
- **Operations that need confirmation**

Use services when you need to know that an operation completed successfully, or when you need a computed result back from another node.

## Messages: The Data Structures

Both topics and services send **messages**—data structures that can be serialized and transmitted over a network. ROS 2 defines messages using a simple interface description language (IDL).

### Common Message Types

ROS 2 comes with many standard message types organized into packages:

- `std_msgs`: Basic types like `String`, `Int32`, `Float64`, `Bool`
- `geometry_msgs`: Geometric primitives like `Point`, `Pose`, `Twist`, `Transform`
- `sensor_msgs`: Sensor data like `Image`, `LaserScan`, `Imu`, `PointCloud2`
- `nav_msgs`: Navigation data like `Odometry`, `Path`

Using standard messages promotes interoperability—your nodes can work with nodes written by others if you all use the same message types.

### Message Definition Example

Here's what a simple message definition looks like (from `geometry_msgs/Point`):

```
float64 x
float64 y
float64 z
```

And a more complex one (`geometry_msgs/Twist`):

```
Vector3 linear
Vector3 angular
```

Where `Vector3` is itself defined as:

```
float64 x
float64 y
float64 z
```

Messages can be nested, allowing you to build complex data structures from simpler ones.

## The ROS 2 Graph

The collection of all nodes, topics, and services in a running system forms what's called the **computation graph** or simply the **ROS graph**. You can visualize this graph to understand how your system is structured.

### Viewing the Graph

ROS 2 provides command-line tools to inspect the running system:

```bash
# List all running nodes
ros2 node list

# List all active topics
ros2 topic list

# List all available services
ros2 service list

# Show information about a specific topic
ros2 topic info /camera/image_raw

# Echo messages being published on a topic
ros2 topic echo /robot/velocity
```

These tools are invaluable for debugging and understanding system behavior.

## Quality of Service (QoS)

One powerful feature of ROS 2 is its Quality of Service (QoS) settings. QoS policies let you tune the behavior of topics to match your application's needs.

### Key QoS Policies

- **Reliability**:
  - `RELIABLE`: Guarantee message delivery (like TCP)
  - `BEST_EFFORT`: Send messages without guarantees (like UDP, faster)

- **Durability**:
  - `VOLATILE`: New subscribers only get messages published after they subscribe
  - `TRANSIENT_LOCAL`: New subscribers get some number of past messages

- **History**:
  - `KEEP_LAST`: Keep only the last N messages in the queue
  - `KEEP_ALL`: Keep all messages (until memory limits)

Choosing the right QoS profile is important. For example, sensor data like laser scans typically uses `BEST_EFFORT` reliability because if a message is lost, a new one arrives milliseconds later. Configuration data might use `RELIABLE` to ensure nothing is missed.

## Communication Patterns Compared

Let's summarize when to use each communication pattern:

| Feature | Topics | Services |
|---------|--------|----------|
| **Pattern** | Publish-Subscribe | Request-Response |
| **Direction** | Unidirectional | Bidirectional |
| **Timing** | Asynchronous | Synchronous (blocking) |
| **Cardinality** | Many-to-many | One-to-one |
| **Best For** | Streaming data, state updates | Infrequent operations, computations |
| **Example Uses** | Sensor readings, robot pose | Triggering actions, querying status |

## Practical Example: A Simple Robot System

Let's imagine a simple mobile robot with the following components:

```
[Laser Scanner Node] --> publishes to --> /scan topic
                                            |
                                            v
                                   [Obstacle Detector Node]
                                            |
                                            v
                          publishes to --> /obstacles topic
                                            |
                                            v
                                     [Path Planner Node]
                                            |
                                            v
                          publishes to --> /cmd_vel topic
                                            |
                                            v
                                      [Motor Controller Node]
```

Additionally, you might have a service:

```
[Path Planner Node] <-- service call -- [Web Interface Node]
                    "Plan path to (x, y)"
                    --> response -->
                    "Path computed" or "No path found"
```

This architecture demonstrates:
- **Topics** for continuous sensor data and commands
- **Services** for on-demand computation
- **Multiple nodes** each with a specific responsibility
- **Loose coupling** allowing components to be developed and tested independently

## Summary

In this chapter, you learned about the fundamental building blocks of ROS 2:

- **Nodes** are independent processes that perform specific tasks in your robot system
- **Topics** enable publish-subscribe communication for streaming data
- **Services** provide request-response communication for operations that need confirmation
- **Messages** are the data structures passed between nodes
- **Quality of Service** settings allow fine-tuning of communication behavior

These concepts form the foundation for all ROS 2 applications. Understanding how nodes communicate through topics and services is essential for designing effective robot software architectures.

In the next chapter, we'll put this knowledge into practice by writing Python code that creates nodes, publishes to topics, and calls services using the rclpy library.

## Further Reading

- ROS 2 Official Documentation: https://docs.ros.org/en/humble/
- Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, 7(66).
- Maruyama, Y., Kato, S., & Azumi, T. (2016). Exploring the performance of ROS2. *Proceedings of the 13th International Conference on Embedded Software (EMSOFT)*, 1-10.
- Data Distribution Service (DDS) Specification: https://www.omg.org/spec/DDS/

---

**Learning Check**: Before moving on, make sure you can:
- ✓ Explain what a ROS 2 node is and why multiple nodes are used
- ✓ Describe the publish-subscribe pattern used by topics
- ✓ Explain when to use services instead of topics
- ✓ Identify appropriate uses for topics vs. services in a robot system
