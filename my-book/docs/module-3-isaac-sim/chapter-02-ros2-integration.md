---
sidebar_position: 2
---

# Chapter 2: ROS 2 Integration and Robot Control

## Introduction

One of Isaac Sim's most powerful features is its native ROS 2 integration. Unlike other simulators that require third-party bridges, Isaac Sim provides built-in ROS 2 support with minimal latency and full message type compatibility.

In this chapter, you'll learn how to connect Isaac Sim to ROS 2, control robots using familiar ROS patterns, and build complete robot applications.

## ROS 2 Bridge Architecture

Isaac Sim's ROS 2 bridge operates at two levels:

1. **Action Graph**: Visual node-based programming for ROS 2 communication
2. **Python API**: Programmatic ROS 2 publishers and subscribers

Both methods provide real-time, bidirectional communication between Isaac Sim and ROS 2.

## Setting Up ROS 2 in Isaac Sim

### Enable ROS 2 Extension

1. In Isaac Sim: Window > Extensions
2. Search for "ROS2"
3. Enable "omni.isaac.ros2_bridge"
4. Restart Isaac Sim if prompted

### Verify ROS 2 Environment

```bash
# In terminal, ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Check ROS_DOMAIN_ID (must match Isaac Sim)
echo $ROS_DOMAIN_ID

# Set if needed (default is 0)
export ROS_DOMAIN_ID=0
```

### Configure Isaac Sim for ROS 2

```python
# In Isaac Sim Script Editor
import os
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"  # Or fastrtps
```

## Publishing Topics from Isaac Sim

### Using Action Graph

Action Graph is Isaac Sim's visual programming interface:

1. **Open Action Graph Editor**: Window > Visual Scripting > Action Graph

2. **Create Graph**: Click "New Action Graph"

3. **Add Clock Node**: Required for ROS 2 timing
   - Right-click > Add Node > Isaac Sim > ROS2 > ROS2 Context

4. **Publish Robot State**:
   ```
   Add Node > Isaac Sim > ROS2 > ROS2 Publish Joint State
   - Connect "exec" from Context to "exec" input
   - Set "topicName": /joint_states
   - Set "targetPrim": /World/Robot
   ```

5. **Play Simulation**: Press Play button

6. **Verify in ROS 2**:
   ```bash
   ros2 topic list
   # Should see: /joint_states

   ros2 topic echo /joint_states
   ```

### Python API Publishing

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

# Import ROS 2 bridge
from omni.isaac.ros2_bridge import ROS2Bridge

# Create world
world = World()

# Create ROS 2 bridge
bridge = ROS2Bridge()

# Load robot
robot = Robot("/World/Robot")
world.scene.add(robot)

# Create publisher for custom topic
from std_msgs.msg import String
publisher = bridge.create_publisher(String, "/robot/status", 10)

# Simulation loop
world.reset()
for i in range(1000):
    # Publish status
    msg = String()
    msg.data = f"Step {i}: Robot is running"
    publisher.publish(msg)

    # Step simulation
    world.step(render=True)
```

## Subscribing to ROS 2 Topics

### Receiving Velocity Commands

```python
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.cmd_vel = np.zeros(2)  # [linear, angular]

        # Subscribe to velocity commands
        bridge = ROS2Bridge()
        bridge.create_subscription(
            Twist,
            "/cmd_vel",
            self.velocity_callback,
            10
        )

    def velocity_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.angular.z

    def apply_velocity(self):
        # Apply to differential drive robot
        wheel_base = 0.4  # meters
        wheel_radius = 0.1  # meters

        # Convert to wheel velocities
        left_vel = (self.cmd_vel[0] - self.cmd_vel[1] * wheel_base / 2) / wheel_radius
        right_vel = (self.cmd_vel[0] + self.cmd_vel[1] * wheel_base / 2) / wheel_radius

        # Apply to robot
        self.robot.set_joint_velocities([left_vel, right_vel])

# Usage
controller = RobotController(robot)
world.reset()

for _ in range(1000):
    controller.apply_velocity()
    world.step(render=True)
```

### Testing with Teleop

```bash
# In separate terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Use keyboard to control robot in Isaac Sim!
```

## Complete ROS 2 Robot Example

### Carter Robot with ROS 2

```python
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.ros2_bridge import ROS2Bridge
from geometry_msgs.msg import Twist
import numpy as np

class CarterROS2:
    def __init__(self):
        # Create world
        self.world = World()

        # Load Carter robot
        assets_root = get_assets_root_path()
        carter_path = assets_root + "/Isaac/Robots/Carter/carter_v1.usd"

        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Carter",
                name="carter",
                wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
                create_robot=True,
                usd_path=carter_path
            )
        )

        # Create controller
        self.controller = DifferentialController(
            name="differential_controller",
            wheel_radius=0.0325,
            wheel_base=0.413
        )

        # ROS 2 setup
        self.bridge = ROS2Bridge()
        self.cmd_vel = np.zeros(2)

        # Subscribe to velocity commands
        self.bridge.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Publish odometry
        from nav_msgs.msg import Odometry
        self.odom_pub = self.bridge.create_publisher(
            Odometry,
            "/odom",
            10
        )

    def cmd_vel_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.angular.z

    def publish_odometry(self):
        # Get robot state
        position, orientation = self.robot.get_world_pose()
        linear_vel, angular_vel = self.robot.get_linear_velocity(), self.robot.get_angular_velocity()

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.bridge.get_ros_time()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = float(position[0])
        odom_msg.pose.pose.position.y = float(position[1])
        odom_msg.pose.pose.position.z = float(position[2])

        # Orientation
        odom_msg.pose.pose.orientation.x = float(orientation[0])
        odom_msg.pose.pose.orientation.y = float(orientation[1])
        odom_msg.pose.pose.orientation.z = float(orientation[2])
        odom_msg.pose.pose.orientation.w = float(orientation[3])

        # Velocity
        odom_msg.twist.twist.linear.x = float(linear_vel[0])
        odom_msg.twist.twist.angular.z = float(angular_vel[2])

        self.odom_pub.publish(odom_msg)

    def run(self):
        self.world.reset()

        while True:
            # Apply velocity commands
            action = self.controller.forward(self.cmd_vel)
            self.robot.apply_wheel_actions(action)

            # Publish odometry
            self.publish_odometry()

            # Step simulation
            self.world.step(render=True)

# Run
carter_ros = CarterROS2()
carter_ros.run()
```

## Sensor Publishing

### Camera (RGB)

```python
from sensor_msgs.msg import Image
from omni.isaac.sensor import Camera

# Create camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2, 0, 1]),
    frequency=30
)

# Initialize
camera.initialize()

# Get RGB image
rgb_data = camera.get_rgb()

# Publish to ROS 2
def publish_camera():
    rgb = camera.get_rgba()[:, :, :3]  # Remove alpha

    img_msg = Image()
    img_msg.header.stamp = bridge.get_ros_time()
    img_msg.header.frame_id = "camera"
    img_msg.height = rgb.shape[0]
    img_msg.width = rgb.shape[1]
    img_msg.encoding = "rgb8"
    img_msg.step = rgb.shape[1] * 3
    img_msg.data = rgb.tobytes()

    camera_pub.publish(img_msg)
```

### LiDAR

```python
from omni.isaac.sensor import RotatingLidarPhysX
from sensor_msgs.msg import LaserScan

# Create LiDAR
lidar = RotatingLidarPhysX(
    prim_path="/World/Lidar",
    name="lidar",
    rotation_frequency=20,  # Hz
    horizontal_fov=360,
    horizontal_resolution=0.4,
    max_range=100.0
)

lidar.initialize()

# Publish scans
def publish_lidar():
    data = lidar.get_current_frame()
    ranges = data["linear_depth_data"].flatten()

    scan_msg = LaserScan()
    scan_msg.header.stamp = bridge.get_ros_time()
    scan_msg.header.frame_id = "lidar"
    scan_msg.angle_min = -3.14159
    scan_msg.angle_max = 3.14159
    scan_msg.angle_increment = 0.4 * 3.14159 / 180
    scan_msg.time_increment = 0
    scan_msg.scan_time = 1/20.0
    scan_msg.range_min = 0.1
    scan_msg.range_max = 100.0
    scan_msg.ranges = ranges.tolist()

    lidar_pub.publish(scan_msg)
```

## TF (Transform) Publishing

```python
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def publish_transforms():
    # Get robot links
    links = robot.get_links()

    tf_msg = TFMessage()

    for link in links:
        pos, rot = link.get_world_pose()

        transform = TransformStamped()
        transform.header.stamp = bridge.get_ros_time()
        transform.header.frame_id = "world"
        transform.child_frame_id = link.name

        transform.transform.translation.x = float(pos[0])
        transform.transform.translation.y = float(pos[1])
        transform.transform.translation.z = float(pos[2])

        transform.transform.rotation.x = float(rot[0])
        transform.transform.rotation.y = float(rot[1])
        transform.transform.rotation.z = float(rot[2])
        transform.transform.rotation.w = float(rot[3])

        tf_msg.transforms.append(transform)

    tf_pub.publish(tf_msg)

# Create TF publisher
tf_pub = bridge.create_publisher(TFMessage, "/tf", 10)
```

## Running Navigation Stack

With proper ROS 2 integration, you can use Nav2:

```bash
# Terminal 1: Launch Isaac Sim with robot

# Terminal 2: Launch Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

## Best Practices

1. **Match Frame Rates**: Sync ROS 2 publishing with simulation rate
2. **Use Proper Frame IDs**: Consistent naming (base_link, odom, map)
3. **Handle Startup**: Wait for simulation to initialize before publishing
4. **Error Handling**: Check for null data before publishing
5. **Performance**: Don't publish faster than needed (30Hz is often sufficient)

## Summary

In this chapter, you learned:

- Isaac Sim's native ROS 2 integration architecture
- How to publish and subscribe to ROS 2 topics
- Complete robot control with velocity commands
- Publishing sensor data (camera, LiDAR, IMU)
- Transform (TF) publishing
- Integration with ROS 2 navigation stack

With Isaac Sim and ROS 2 working together, you have a powerful platform for developing and testing robot software before hardware deployment.

## Further Reading

- Isaac Sim ROS 2 Documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials.html
- ROS 2 Nav2: https://navigation.ros.org/
- Isaac ROS: https://github.com/NVIDIA-ISAAC-ROS/

---

**Learning Check**:
- ✓ Set up ROS 2 bridge in Isaac Sim
- ✓ Publish and subscribe to topics
- ✓ Control robots with ROS 2 commands
- ✓ Publish sensor data to ROS 2
- ✓ Integrate with ROS 2 navigation
