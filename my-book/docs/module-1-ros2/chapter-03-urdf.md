---
sidebar_position: 3
---

# Chapter 3: Humanoid URDF Modeling

## Introduction

So far, you've learned how to create ROS 2 nodes that communicate through topics and services. But how do you describe the physical structure of your robot to ROS 2? How does the system know where the sensors are located, how the joints move, or what the robot looks like?

The answer is the **Unified Robot Description Format (URDF)**—an XML-based format for describing robot structures (ROS Wiki, 2024). URDF files define:

- The physical structure (links and joints)
- Visual appearance for simulation
- Collision geometry for physics
- Inertial properties for dynamics

In this chapter, you'll learn how to create a URDF model of a simple humanoid robot and visualize it using ROS 2 tools.

## What is URDF?

URDF is the standard format for robot descriptions in the ROS ecosystem. Think of it as a blueprint that describes your robot's anatomy:

- **Links**: Rigid body parts (like bones) - torso, head, arms, legs
- **Joints**: Connections between links that define how they move relative to each other

A URDF model forms a kinematic tree structure. Every robot has a **root link** (usually the base or torso), and all other links connect to it through joints, directly or indirectly.

### URDF vs. Other Formats

- **URDF**: XML-based, human-readable, standard in ROS
- **SDF**: Simulation Description Format, used by Gazebo, more features but more complex
- **MJCF**: MuJoCo XML format, used by MuJoCo physics engine
- **Xacro**: A macro language that generates URDF (we'll stick to pure URDF for simplicity)

## Basic URDF Structure

A minimal URDF file has this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid body parts -->
  <link name="base_link">
    <!-- Visual: what it looks like -->
    <!-- Collision: shape for physics -->
    <!-- Inertial: mass and inertia properties -->
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="fixed">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Transform from parent to child -->
  </joint>
</robot>
```

Let's build a humanoid robot step by step.

## Step 1: Creating the Torso

We'll start with the torso, which will be our root link. Create a file named `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso: the root link of our robot -->
  <link name="torso">
    <!-- Visual properties: what the torso looks like -->
    <visual>
      <geometry>
        <!-- A box: width (x), depth (y), height (z) in meters -->
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <!-- RGBA color: red, green, blue, alpha (opacity) -->
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>

    <!-- Collision properties: simplified shape for physics -->
    <collision>
      <geometry>
        <!-- Usually matches visual, but can be simpler for performance -->
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>

    <!-- Inertial properties: mass and inertia tensor -->
    <inertial>
      <mass value="10.0"/>
      <!-- For a box, inertia depends on mass and dimensions -->
      <!-- These are simplified values -->
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.4" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

</robot>
```

This defines a simple torso as a blue box, 30cm wide, 20cm deep, and 50cm tall.

### Understanding Coordinate Frames

In ROS 2 (and robotics generally), we use the **right-hand coordinate system**:
- **X-axis** (red): Forward
- **Y-axis** (green): Left
- **Z-axis** (blue): Up

Each link has its own coordinate frame, positioned at the link's origin.

## Step 2: Adding a Head

Now let's add a head connected to the top of the torso with a revolute (rotating) joint:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.4" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <!-- Sphere with radius in meters -->
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <!-- For a sphere: I = (2/5) * m * r^2 -->
      <inertia ixx="0.0115" ixy="0.0" ixz="0.0"
               iyy="0.0115" iyz="0.0"
               izz="0.0115"/>
    </inertial>
  </link>

  <!-- Neck joint: connects head to torso -->
  <joint name="neck_joint" type="revolute">
    <!-- Parent link: torso -->
    <parent link="torso"/>
    <!-- Child link: head -->
    <child link="head"/>

    <!-- Transform from parent's origin to child's origin -->
    <!-- xyz: translation in meters (forward, left, up) -->
    <!-- The head is 0.3m above the torso's center -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>

    <!-- Axis of rotation (Z-axis = yaw/pan) -->
    <axis xyz="0 0 1"/>

    <!-- Joint limits -->
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```

### Understanding Joints

We added a **revolute joint** (a hinge that can rotate). Key properties:

- **`type`**: Joint type (`revolute`, `continuous`, `prismatic`, `fixed`)
- **`origin`**: Position and orientation of the child relative to the parent
  - `xyz`: translation in meters
  - `rpy`: rotation in radians (roll, pitch, yaw)
- **`axis`**: Axis around which the joint rotates
- **`limit`**: Movement limits
  - `lower`/`upper`: Range of motion (radians for revolute)
  - `effort`: Maximum force/torque
  - `velocity`: Maximum speed

In our neck joint:
- Head is positioned 0.3m above torso center
- Rotates around Z-axis (yaw/looking left-right)
- Limited to ±90 degrees (±1.57 radians)

## Step 3: Adding Arms

Let's add arms with multiple joints. Each arm will have:
- A shoulder joint (revolute)
- An upper arm link
- An elbow joint (revolute)
- A forearm link

```xml
  <!-- Right Upper Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <!-- Cylinder: radius and length -->
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <!-- Cylinders are along Z-axis by default, rotate 90° around Y to make horizontal -->
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.012" ixy="0.0" ixz="0.0"
               iyy="0.012" iyz="0.0"
               izz="0.0012"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <!-- Position at right side of torso, near top -->
    <origin xyz="0 -0.15 0.2" rpy="0 0 0"/>
    <!-- Rotate around Y-axis (pitch) for up/down movement -->
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="3.14" effort="20.0" velocity="1.0"/>
  </joint>

  <!-- Right Forearm -->
  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.0052" ixy="0.0" ixz="0.0"
               iyy="0.0052" iyz="0.0"
               izz="0.0006"/>
    </inertial>
  </link>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <!-- Position at end of upper arm -->
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <!-- Rotate around Y-axis for elbow bend -->
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.36" effort="15.0" velocity="1.0"/>
  </joint>
```

For a complete robot, you would add the left arm, legs, and feet similarly. Here's the complete file with all limbs:

[Download complete simple_humanoid.urdf](pathname:///examples/simple_humanoid.urdf)

## Joint Types in URDF

URDF supports several joint types:

### 1. Fixed Joint
No movement—rigidly attaches two links.

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>
```

Use for: sensors, structural connections

### 2. Revolute Joint
Rotational motion with limits (like our neck and elbows).

```xml
<joint name="hinge" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

Use for: elbows, knees, wheels with limits

### 3. Continuous Joint
Rotational motion without limits (can spin 360°+).

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base"/>
  <child link="wheel"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="5.0" velocity="10.0"/>
</joint>
```

Use for: wheels, propellers

### 4. Prismatic Joint
Linear motion (sliding) with limits.

```xml
<joint name="slider" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="1.0" effort="100.0" velocity="0.5"/>
</joint>
```

Use for: linear actuators, elevators

### 5. Floating Joint
6 degrees of freedom (rarely used in URDF).

### 6. Planar Joint
Motion in a plane (rarely used in URDF).

## Visualizing Your URDF in Rviz

Rviz is ROS 2's 3D visualization tool. Let's see our humanoid robot!

### Step 1: Check URDF Validity

First, validate your URDF file:

```bash
check_urdf simple_humanoid.urdf
```

You should see output like:

```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: torso has 4 child(ren)
    child(1):  head
    child(2):  right_upper_arm
        child(1):  right_forearm
    child(3):  left_upper_arm
        child(1):  left_forearm
```

If there are errors, the tool will tell you what's wrong.

### Step 2: Create a Launch File

Create a file named `view_humanoid.launch.py`:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Path to your URDF file
    urdf_file = 'simple_humanoid.urdf'

    # Read the URDF file content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher node
    # This node publishes the robot's kinematic tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint State Publisher GUI
    # This provides sliders to move the robot's joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Rviz node
    # This opens the 3D visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'urdf_config.rviz'] if os.path.exists('urdf_config.rviz') else []
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
```

### Step 3: Launch and Visualize

```bash
ros2 launch view_humanoid.launch.py
```

This will open:
1. **Rviz**: 3D visualization window
2. **Joint State Publisher GUI**: Sliders to control joints

### Step 4: Configure Rviz

In Rviz:

1. Change **Fixed Frame** to `torso` (in the left panel)
2. Click **Add** button (bottom left)
3. Select **RobotModel**
4. You should now see your humanoid robot!

Use the Joint State Publisher GUI sliders to move the robot's joints and see it in real-time.

## Understanding the robot_state_publisher

The `robot_state_publisher` is a crucial ROS 2 node. It:

1. Reads your URDF file
2. Builds the kinematic tree
3. Subscribes to `/joint_states` topic (joint angles)
4. Publishes transforms (`/tf`) for every link

This allows other nodes to query: "Where is the right hand relative to the torso?" The answer is computed from the kinematic tree and current joint positions.

## Practical Tips for URDF Development

### 1. Start Simple
Build your robot incrementally. Start with one or two links, verify they work, then add more.

### 2. Use Consistent Units
- Lengths: meters
- Angles: radians
- Mass: kilograms
- Time: seconds

### 3. Set Realistic Inertial Properties
Incorrect inertia can cause simulation instability. For simple shapes:

**Box** (dimensions: x, y, z):
```
Ixx = (1/12) * m * (y² + z²)
Iyy = (1/12) * m * (x² + z²)
Izz = (1/12) * m * (x² + y²)
```

**Sphere** (radius: r):
```
I = (2/5) * m * r²
```

**Cylinder** (radius: r, height: h, Z-axis aligned):
```
Ixx = Iyy = (1/12) * m * (3*r² + h²)
Izz = (1/2) * m * r²
```

### 4. Name Links and Joints Descriptively
Use clear, consistent naming:
- `base_link`, `torso`, `head`
- `left_shoulder`, `right_elbow`
- `front_left_wheel`, `back_right_wheel`

### 5. Test Frequently
After each addition, use `check_urdf` and visualize in Rviz to catch errors early.

### 6. Comment Your URDF
XML comments help you remember design decisions:

```xml
<!-- Left arm: positioned at left side of torso -->
<!-- Shoulder allows 180° range for reaching -->
```

## Advanced: Xacro for Modular URDF

For complex robots, writing raw URDF becomes tedious. **Xacro** is a macro language that generates URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="prefix reflect">
    <link name="${prefix}_upper_arm">
      <!-- Link definition -->
    </link>

    <joint name="${prefix}_shoulder" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 ${reflect*0.15} 0.2"/>
      <!-- Rest of joint definition -->
    </joint>
  </xacro:macro>

  <!-- Instantiate arms -->
  <xacro:arm prefix="right" reflect="-1"/>
  <xacro:arm prefix="left" reflect="1"/>

</robot>
```

Xacro reduces repetition and makes your robot description maintainable. Convert to URDF with:

```bash
xacro my_robot.xacro > my_robot.urdf
```

## Summary

In this chapter, you learned:

- **URDF** is the standard format for describing robot structures in ROS 2
- **Links** represent rigid body parts with visual, collision, and inertial properties
- **Joints** connect links and define how they move relative to each other
- **Joint types** include fixed, revolute, continuous, and prismatic
- **robot_state_publisher** publishes the robot's kinematic tree to the ROS 2 ecosystem
- **Rviz** allows 3D visualization of your URDF models
- **check_urdf** validates your URDF syntax and structure

You can now describe robot structures in code and visualize them in ROS 2 tools—a critical skill for robotics development. In future modules, you'll learn how to use these models in physics simulators like Gazebo and how to integrate perception and AI for intelligent robot behaviors.

## Further Reading

- URDF Specification: http://wiki.ros.org/urdf/XML
- URDF Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- Xacro Documentation: http://wiki.ros.org/xacro
- Rviz User Guide: https://github.com/ros2/rviz
- robot_state_publisher: https://github.com/ros/robot_state_publisher

---

**Learning Check**: Before finishing this module, make sure you can:
- ✓ Create a valid URDF file with multiple links and joints
- ✓ Explain the difference between revolute, continuous, and fixed joints
- ✓ Validate a URDF file using check_urdf
- ✓ Visualize a robot model in Rviz
- ✓ Understand the role of robot_state_publisher

## Congratulations!

You've completed Module 1: The Robotic Nervous System (ROS 2). You now understand:
- How ROS 2 provides communication infrastructure for robots
- How to create Python nodes that publish, subscribe, and use services
- How to model robot structures using URDF

These skills form the foundation for all ROS 2 development. In the next modules, you'll learn how to use simulation environments, advanced perception, and AI models to build truly intelligent robotic systems.

**Continue to Module 2: Gazebo & Unity Simulation (coming soon)** →
