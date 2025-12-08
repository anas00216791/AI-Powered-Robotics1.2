---
sidebar_position: 1
---

# Chapter 1: Gazebo Physics Simulation

## Introduction

Gazebo is a powerful 3D robotics simulator that integrates seamlessly with ROS 2. It provides accurate physics simulation, sensor modeling, and a plugin architecture that allows you to test robot behaviors in realistic environments before deploying to hardware (Koenig & Howard, 2004).

In this chapter, you'll learn how physics engines work, how to create simulation worlds, and how to configure Gazebo for your robotics projects.

## Understanding Physics Engines

A physics engine simulates the laws of physics in a virtual environment. For robotics, this includes:

### Core Physics Components

1. **Rigid Body Dynamics**: How objects move and rotate under forces
2. **Collision Detection**: Determining when objects touch or overlap
3. **Contact Resolution**: Computing forces when objects collide
4. **Constraints**: Joints, springs, and other connections between bodies
5. **Gravity**: The constant downward force affecting all objects

### Physics Engines in Gazebo

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Fast, stable, good for mobile robots
- **Bullet**: Accurate collision detection, good for manipulation
- **DART**: Excellent for legged locomotion and contact-rich scenarios
- **Simbody**: High-fidelity biomechanical simulation

The default is ODE, which provides a good balance of speed and accuracy for most applications.

## Installing Gazebo with ROS 2

Gazebo comes bundled with most ROS 2 installations. Verify your installation:

```bash
# Check Gazebo version
gazebo --version

# Launch Gazebo
gazebo

# Launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

If Gazebo isn't installed:

```bash
# Ubuntu/Debian
sudo apt install ros-humble-gazebo-ros-pkgs

# This includes Gazebo and ROS 2 integration packages
```

## Creating Your First World

A Gazebo world is defined in an SDF (Simulation Description Format) file. Let's create a simple world.

### Example: Basic World File

Create a file named `simple_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">

    <!-- Physics configuration -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- Gravity (m/s²) -->
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>false</static>

      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>

        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.67</ixx>
            <iyy>1.67</iyy>
            <izz>1.67</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### Launching Your World

```bash
gazebo simple_world.sdf
```

You should see a ground plane with a red box floating above it. The box will fall due to gravity and land on the ground.

## Physics Configuration Parameters

### Time Step Configuration

The physics update rate determines simulation accuracy and speed:

```xml
<physics>
  <!-- Size of each physics step (seconds) -->
  <max_step_size>0.001</max_step_size>

  <!-- Target real-time speed (1.0 = real-time, 0.5 = half speed) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Physics updates per second -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**Rule of thumb**: Smaller step sizes = more accurate but slower simulation.

For mobile robots: `0.001s` (1ms) is usually sufficient
For manipulation: `0.0001s` (0.1ms) may be needed for stability

### Gravity

Change gravity for different scenarios:

```xml
<!-- Earth gravity -->
<gravity>0 0 -9.81</gravity>

<!-- Moon gravity (1/6 of Earth) -->
<gravity>0 0 -1.62</gravity>

<!-- Mars gravity -->
<gravity>0 0 -3.71</gravity>

<!-- Zero gravity (space) -->
<gravity>0 0 0</gravity>
```

### Contact Parameters

Control how objects interact when colliding:

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <!-- Friction coefficients -->
        <mu>0.8</mu>   <!-- Primary direction -->
        <mu2>0.8</mu2> <!-- Secondary direction -->
      </ode>
    </friction>

    <contact>
      <ode>
        <!-- Stiffness (N/m) - higher = less penetration -->
        <kp>1000000.0</kp>

        <!-- Damping (N·s/m) - higher = less bouncing -->
        <kd>1.0</kd>

        <!-- Maximum contact correction velocity -->
        <max_vel>0.01</max_vel>

        <!-- Minimum penetration depth before correction -->
        <min_depth>0.001</min_depth>
      </ode>
    </contact>

    <bounce>
      <!-- Coefficient of restitution (0 = no bounce, 1 = perfect bounce) -->
      <restitution_coefficient>0.0</restitution_coefficient>
      <threshold>0.1</threshold>
    </bounce>
  </surface>
</collision>
```

## Building Complex Environments

### Adding Multiple Objects

Create a more complex scene:

```xml
<world name="obstacle_course">
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <!-- Wall 1 -->
  <model name="wall_1">
    <pose>5 0 1 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>0.2 4 2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.2 4 2</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
        </material>
      </visual>
    </link>
  </model>

  <!-- Ramp -->
  <model name="ramp">
    <pose>10 0 0.5 0 0.3 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>4 2 0.1</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>4 2 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.3 0.6 0.3 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</world>
```

### Using Model Database

Gazebo includes pre-built models:

```xml
<!-- Add a table -->
<include>
  <uri>model://cafe_table</uri>
  <pose>3 2 0 0 0 0</pose>
</include>

<!-- Add a chair -->
<include>
  <uri>model://chair</uri>
  <pose>3 1 0 0 0 1.57</pose>
</include>
```

Browse available models: http://models.gazebosim.org/

## Integrating Robots with Gazebo

### Spawning a Robot from URDF

You can spawn your URDF robot (from Module 1) into Gazebo:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Read URDF file
    urdf_file = 'simple_humanoid.urdf'
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity
    ])
```

Launch with:

```bash
ros2 launch spawn_robot.launch.py
```

### Adding Gazebo-Specific Tags to URDF

For physics simulation, add Gazebo tags to your URDF:

```xml
<robot name="my_robot">

  <link name="base_link">
    <!-- Visual and collision as before -->
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <!-- Material color in Gazebo -->
    <material>Gazebo/Blue</material>

    <!-- Physics properties -->
    <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
    <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>1.0</kd>  <!-- Contact damping -->
  </gazebo>

</robot>
```

## Gazebo Plugins

Plugins extend Gazebo's functionality. Common plugins:

### Differential Drive Plugin

For wheeled mobile robots:

```xml
<gazebo>
  <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
    <!-- ROS 2 parameters -->
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <!-- Wheel configuration -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.5</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <!-- Limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <!-- Output -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

### Joint State Publisher Plugin

Publishes joint positions to ROS 2:

```xml
<gazebo>
  <plugin name="joint_states" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <remapping>joint_states:=joint_states</remapping>
    </ros>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

## Testing Physics Behavior

### Experiment: Bouncing Ball

Create a world to test coefficient of restitution:

```xml
<model name="bouncy_ball">
  <pose>0 0 5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere><radius>0.2</radius></sphere>
      </geometry>
      <surface>
        <bounce>
          <restitution_coefficient>0.9</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere><radius>0.2</radius></sphere>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
      </material>
    </visual>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.016</ixx>
        <iyy>0.016</iyy>
        <izz>0.016</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

The ball will bounce repeatedly, with height decreasing each time due to energy loss.

### Debugging Physics Issues

Common problems and solutions:

**Problem**: Objects fall through ground
**Solution**: Check collision geometry exists, verify `<static>true</static>` for ground

**Problem**: Robot vibrates or explodes
**Solution**: Reduce time step size, check inertial properties, increase contact damping

**Problem**: Joints are unstable
**Solution**: Add damping to joints, check joint limits, reduce PID gains

**Problem**: Simulation runs too slow
**Solution**: Increase time step, disable shadows, reduce collision complexity

## Performance Optimization

### Simplify Collision Geometry

Use simple shapes for collisions, complex meshes for visuals:

```xml
<link name="complex_object">
  <!-- Simple collision (fast) -->
  <collision name="collision">
    <geometry>
      <box><size>1 0.5 0.3</size></box>
    </geometry>
  </collision>

  <!-- Detailed visual (doesn't affect physics) -->
  <visual name="visual">
    <geometry>
      <mesh><uri>model://complex_mesh.dae</uri></mesh>
    </geometry>
  </visual>
</link>
```

### Adjust Update Rates

Balance accuracy and speed:

```xml
<physics>
  <!-- For fast prototyping -->
  <max_step_size>0.005</max_step_size>
  <real_time_update_rate>200</real_time_update_rate>

  <!-- For accurate simulation -->
  <max_step_size>0.0001</max_step_size>
  <real_time_update_rate>10000</real_time_update_rate>
</physics>
```

## Best Practices

1. **Start Simple**: Test with basic shapes before adding complexity
2. **Verify Inertial Properties**: Incorrect mass/inertia causes instability
3. **Use Appropriate Time Steps**: Smaller for manipulation, larger for navigation
4. **Static vs Dynamic**: Mark non-moving objects as `<static>true</static>` for performance
5. **Test Incrementally**: Add one feature at a time and verify behavior
6. **Monitor Real-Time Factor**: If it drops below 0.5, simplify your world

## Summary

In this chapter, you learned:

- How physics engines simulate forces, collisions, and dynamics
- How to configure Gazebo's physics parameters
- How to create worlds with objects and environments
- How to integrate robots from URDF into Gazebo
- How to use Gazebo plugins for robot control
- How to debug and optimize physics simulations

With these skills, you can create realistic test environments for your robots. In the next chapter, you'll learn how Unity provides photorealistic rendering and human-robot interaction capabilities.

## Further Reading

- Gazebo Tutorials: http://gazebosim.org/tutorials
- SDF Specification: http://sdformat.org/
- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*.
- Gazebo ROS 2 Integration: https://github.com/ros-simulation/gazebo_ros_pkgs

---

**Learning Check**:
- ✓ Understand how physics engines work
- ✓ Create SDF world files with custom environments
- ✓ Configure physics parameters for accurate simulation
- ✓ Spawn robots from URDF into Gazebo
- ✓ Use Gazebo plugins for robot control
