---
sidebar_position: 1
---

# Chapter 1: Getting Started with Isaac Sim

## Introduction

NVIDIA Isaac Sim is a robotics simulation platform built on NVIDIA Omniverse, providing photorealistic rendering, accurate physics, and powerful tools for synthetic data generation. Isaac Sim is designed specifically for developing and testing AI-powered robots, with native support for deep learning workflows and ROS 2 integration (NVIDIA, 2024).

## Key Features of Isaac Sim

### Photorealistic Rendering
- **RTX Ray Tracing**: Real-time ray-traced rendering for physically accurate lighting
- **High-Quality Materials**: PBR (Physically Based Rendering) materials
- **Dynamic Lighting**: Realistic shadows, reflections, and global illumination

### Accurate Physics
- **PhysX 5**: NVIDIA's GPU-accelerated physics engine
- **Articulation Dynamics**: Precise simulation of robot joints and constraints
- **Contact Simulation**: Accurate collision detection and contact forces

### Synthetic Data Generation
- **Domain Randomization**: Automated variation of scene parameters
- **Ground Truth Labels**: Perfect annotations for object detection, segmentation, pose estimation
- **Replicator**: Built-in tool for generating massive datasets

### ROS 2 Integration
- Native ROS 2 bridge without additional middleware
- Support for all standard ROS message types
- Real-time synchronization between Isaac Sim and ROS 2 nodes

## System Requirements

**Minimum**:
- NVIDIA RTX 2060 or better (6GB+ VRAM)
- 16 GB RAM
- 50 GB disk space
- Ubuntu 20.04/22.04 or Windows 10/11

**Recommended**:
- NVIDIA RTX 3080 or better (10GB+ VRAM)
- 32 GB RAM
- 100 GB SSD
- Ubuntu 22.04

**Check GPU Compatibility**:
```bash
nvidia-smi
```

Look for "RTX" series or compute capability 7.0+.

## Installation

### Step 1: Install Omniverse Launcher

1. Go to https://www.nvidia.com/en-us/omniverse/download/
2. Download Omniverse Launcher for your OS
3. Install and launch

### Step 2: Install Isaac Sim

1. In Omniverse Launcher, go to "Exchange" tab
2. Search for "Isaac Sim"
3. Click "Install" (downloads ~20GB)
4. Wait for installation to complete

### Step 3: Verify Installation

1. In Launcher, go to "Library" tab
2. Click "Isaac Sim"
3. Click "Launch"
4. Isaac Sim should open (first launch may take several minutes)

## Isaac Sim Interface Overview

### Main Windows

**Viewport**: 3D rendering window showing your scene

**Stage**: Scene hierarchy (like Unity's Hierarchy or Gazebo's Model tree)

**Property Panel**: Inspector for selected objects

**Content Browser**: Asset library and file explorer

**Console**: Python console and log output

### Basic Navigation

- **Orbit**: Alt + Left Mouse Button
- **Pan**: Alt + Middle Mouse Button
- **Zoom**: Mouse Wheel or Alt + Right Mouse Button
- **Select**: Left Mouse Button
- **Multi-select**: Ctrl + Left Click

## Creating Your First Scene

### Step 1: Add a Ground Plane

```python
# In Isaac Sim's Script Editor (Window > Script Editor)

import omni.isaac.core.utils.stage as stage_utils
from pxr import Gf

# Create ground plane
stage_utils.create_ground_plane("/World/Ground", size=10.0, color=Gf.Vec3f(0.5, 0.5, 0.5))
```

Or use the UI:
1. Create > Physics > Ground Plane
2. Adjust size in Property Panel

### Step 2: Add a Cube

```python
from omni.isaac.core.utils import prims

# Create a cube
cube_prim = prims.create_prim(
    "/World/Cube",
    "Cube",
    position=(0, 0, 1.0),
    scale=(0.5, 0.5, 0.5)
)

# Add physics (makes it fall)
from pxr import UsdPhysics
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)
```

### Step 3: Run Physics

Press **Play** button (or spacebar)

The cube should fall and land on the ground plane.

## Importing Robots

### Method 1: Import USD File

Isaac Sim uses USD (Universal Scene Description) as its native format.

```python
from omni.isaac.core.utils import stage

# Import a USD robot
stage.add_reference_to_stage(
    "/World/MyRobot",
    "path/to/robot.usd"
)
```

### Method 2: Import URDF

Isaac Sim can convert URDF to USD:

```python
from omni.importer.urdf import _urdf as urdf_importer

# Import URDF (creates USD representation)
urdf_importer.acquire_urdf_interface().import_urdf(
    urdf_path="path/to/robot.urdf",
    import_config=urdf_importer.ImportConfig(),
    dest_path="/World/Robot"
)
```

Or use UI:
1. File > Import > URDF
2. Select your URDF file
3. Configure import options
4. Click Import

### Built-in Robots

Isaac Sim includes several robot models:

```python
# Load Carter robot (wheeled mobile robot)
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

assets_root_path = get_assets_root_path()
carter_path = assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd"

add_reference_to_stage(carter_path, "/World/Carter")
```

Available robots:
- **Carter**: Wheeled mobile robot
- **Franka**: Robotic arm
- **Universal Robots UR10**: Industrial manipulator
- **Jetbot**: Small mobile robot
- **ANYmal**: Quadruped robot

## Basic Scripting with Python

Isaac Sim uses Python 3.7+ with a rich API for scene manipulation.

### Creating Objects Programmatically

```python
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create Isaac Sim world
world = World()

# Add dynamic objects
for i in range(5):
    position = np.array([i * 0.6, 0, 2.0])
    cube = DynamicCuboid(
        prim_path=f"/World/Cube_{i}",
        position=position,
        size=0.5,
        color=np.random.rand(3)
    )
    world.scene.add(cube)

# Reset world (initializes physics)
world.reset()

# Run simulation
for _ in range(1000):
    world.step(render=True)
```

### Controlling Robot Joints

```python
from omni.isaac.core.articulations import Articulation

# Load robot
robot = Articulation("/World/Robot")

# Initialize
robot.initialize()

# Get joint information
num_joints = robot.num_dof
joint_names = robot.dof_names

print(f"Robot has {num_joints} joints: {joint_names}")

# Set joint positions (radians)
target_positions = np.array([0.5, -0.5, 1.0, 0.0, 0.5, 0.0])
robot.set_joint_positions(target_positions)

# Apply joint efforts (torques)
target_efforts = np.array([10.0, 5.0, 3.0, 1.0, 1.0, 0.5])
robot.set_joint_efforts(target_efforts)
```

## Camera and Rendering

### Adding a Camera

```python
from omni.isaac.core.utils.prims import create_prim

# Create camera
camera = create_prim(
    "/World/Camera",
    "Camera",
    position=(5, 5, 3),
    rotation=(0, 30, 45)
)

# Point camera at origin
from pxr import Gf
from omni.isaac.core.utils.rotations import lookat_to_quatf

look_at = Gf.Vec3d(0, 0, 0)
camera_pos = Gf.Vec3d(5, 5, 3)

# Calculate rotation to look at origin
# (Manual implementation or use helper functions)
```

### Capturing Images

```python
import omni.replicator.core as rep

# Create render product (defines what to render)
camera_path = "/World/Camera"
render_product = rep.create.render_product(camera_path, (1024, 768))

# Capture RGB image
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(output_dir="./output", rgb=True)
rgb_writer.attach([render_product])

# Run simulation and capture
world.reset()
for _ in range(10):
    world.step(render=True)
    # Images are automatically saved
```

## Physics Configuration

### Adjusting Gravity

```python
from pxr import UsdPhysics, Gf

# Get physics scene
stage = omni.usd.get_context().get_stage()
physicsScene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

# Set gravity (m/s²)
physicsScene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr().Set(9.81)

# For Moon gravity
physicsScene.CreateGravityMagnitudeAttr().Set(1.62)
```

### Time Step Configuration

```python
from omni.isaac.core.simulation_context import SimulationContext

# Create simulation context
sim = SimulationContext()

# Set physics parameters
sim.set_simulation_dt(physics_dt=1/60.0, rendering_dt=1/60.0)

# Smaller time steps for more accuracy
sim.set_simulation_dt(physics_dt=1/240.0, rendering_dt=1/60.0)
```

## Environments and Assets

### Nucleus Server

Omniverse Nucleus is a content server for sharing assets:

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Get Isaac Sim assets
assets_root = get_assets_root_path()
print(f"Assets located at: {assets_root}")

# Common asset paths:
# - Robots: {assets_root}/Isaac/Robots/
# - Environments: {assets_root}/Isaac/Environments/
# - Props: {assets_root}/Isaac/Props/
```

### Loading Environments

```python
# Load hospital environment
hospital_path = assets_root + "/Isaac/Environments/Hospital/hospital.usd"
add_reference_to_stage(hospital_path, "/World/Hospital")

# Load warehouse
warehouse_path = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(warehouse_path, "/World/Warehouse")
```

## Best Practices

### 1. Use Layers for Organization

USD uses layers to organize content:
```python
# Create sublayer for robots
from pxr import Sdf
layer = Sdf.Layer.CreateNew("robots.usd")
stage.GetRootLayer().subLayerPaths.append(layer.identifier)
```

### 2. Leverage GPU Acceleration

```python
# Enable GPU dynamics (faster physics)
from omni.physx import get_physx_interface

physx = get_physx_interface()
physx.update_simulation_settings(
    gpu_dynamics=True,
    gpu_found_lost_pairs_capacity=2048
)
```

### 3. Optimize Rendering

```python
# Adjust rendering quality vs speed
import carb

settings = carb.settings.get_settings()

# Fast mode (for development)
settings.set("/rtx/pathtracing/spp", 1)  # Samples per pixel

# Quality mode (for data generation)
settings.set("/rtx/pathtracing/spp", 64)
```

### 4. Save Your Scenes

```python
# Save current stage
stage = omni.usd.get_context().get_stage()
stage.Export("my_scene.usd")

# Load saved stage
from omni.isaac.core.utils import stage as stage_utils
stage_utils.open_stage("my_scene.usd")
```

## Troubleshooting

### Performance Issues

**Problem**: Low frame rate
**Solutions**:
- Reduce rendering samples
- Simplify collision geometry
- Disable shadows for non-essential objects
- Use smaller physics time steps

### Import Errors

**Problem**: URDF import fails
**Solutions**:
- Check URDF is valid (use `check_urdf`)
- Ensure all mesh files are accessible
- Use absolute paths for mesh references
- Check joint types are supported

### Physics Instability

**Problem**: Robot shakes or explodes
**Solutions**:
- Reduce physics time step
- Check inertial properties are realistic
- Increase contact stiffness
- Add joint damping

## Summary

In this chapter, you learned:

- Isaac Sim's key features and advantages
- How to install and navigate Isaac Sim
- Creating basic scenes with objects and physics
- Importing robots from URDF
- Basic Python scripting for scene manipulation
- Camera setup and rendering
- Physics configuration
- Using built-in assets and environments

Isaac Sim provides professional-grade simulation capabilities that bridge the gap between simulation and reality. In the next chapter, you'll learn how to integrate Isaac Sim with ROS 2 for seamless robot development.

## Further Reading

- Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim/
- Omniverse USD: https://docs.omniverse.nvidia.com/prod_usd/
- PhysX Documentation: https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/
- Isaac Sim Examples: https://github.com/NVIDIA-Omniverse/IsaacSimSamples

---

**Learning Check**:
- ✓ Install and launch Isaac Sim
- ✓ Navigate the Isaac Sim interface
- ✓ Create basic scenes with physics
- ✓ Import robots from URDF
- ✓ Write Python scripts to control simulation
- ✓ Configure physics parameters
