---
sidebar_position: 3
---

# Chapter 3: Synthetic Data Generation

## Introduction

Training robust AI models for robotics requires massive amounts of labeled data. Collecting real-world data is expensive, time-consuming, and limited in diversity. Isaac Sim's synthetic data generation capabilities solve this problem by creating unlimited photorealistic training data with perfect ground truth labels.

In this chapter, you'll learn how to use Isaac Sim's Replicator tool to generate synthetic datasets for object detection, segmentation, pose estimation, and more.

## Why Synthetic Data?

### Advantages

1. **Infinite Data**: Generate millions of training examples
2. **Perfect Labels**: Automatic ground truth for all tasks
3. **Domain Randomization**: Create robust models that generalize
4. **Rare Scenarios**: Test edge cases that rarely occur in reality
5. **Cost Effective**: No manual labeling required

### Successful Applications

- **NVIDIA DOPE**: 6D pose estimation trained entirely on synthetic data
- **Isaac ROS Object Detection**: Retail object detection with 95%+ accuracy
- **Warehouse Navigation**: Autonomous forklifts trained in simulation

## Replicator Overview

Isaac Sim's **Replicator** is a Python API for synthetic data generation. Key capabilities:

- **Randomization**: Automatically vary objects, lighting, materials, camera angles
- **Data Writers**: Export data in formats for PyTorch, TensorFlow, COCO, KITTI
- **Annotations**: 2D/3D bounding boxes, segmentation, depth, normals
- **Scalability**: Generate datasets in parallel

## Basic Data Generation Workflow

### Step 1: Create Scene

```python
import omni.replicator.core as rep
from pxr import Gf

# Create environment
with rep.new_layer():
    # Ground plane
    plane = rep.create.plane(
        scale=10,
        visible=True
    )

    # Create objects to detect
    cube = rep.create.cube(
        position=(0, 0, 1),
        scale=0.5,
        semantics=[("class", "cube")]
    )

    sphere = rep.create.sphere(
        position=(1, 0, 0.5),
        scale=0.3,
        semantics=[("class", "sphere")]
    )
```

### Step 2: Add Camera

```python
# Create camera
camera = rep.create.camera(
    position=(3, 3, 2),
    look_at=(0, 0, 0.5)
)

# Attach render product
render_product = rep.create.render_product(camera, (1024, 1024))
```

### Step 3: Define Randomization

```python
def randomize_scene():
    # Randomize object positions
    with cube:
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 1.5)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    with sphere:
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 1.5))
        )

    # Randomize lighting
    with rep.create.light():
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 3000))
        rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0)))

    return cube.node, sphere.node

# Register randomizer
rep.randomizer.register(randomize_scene)
```

### Step 4: Configure Output

```python
# Basic writer (RGB + annotations)
writer = rep.WriterRegistry.get("BasicWriter")

writer.initialize(
    output_dir="./synthetic_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    distance_to_camera=True
)

writer.attach([render_product])
```

### Step 5: Run Generation

```python
# Generate 1000 frames
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.randomize_scene()

# Execute (blocking)
rep.orchestrator.run()
```

## Domain Randomization Techniques

### Object Randomization

```python
# Randomize object appearance
def randomize_materials():
    # Random colors
    with cube:
        rep.randomizer.color(
            colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1))
        )

    # Random textures
    textures = [
        "wood.png",
        "metal.png",
        "plastic.png",
        "concrete.png"
    ]

    with sphere:
        rep.randomizer.texture(
            textures=rep.distribution.choice(textures)
        )
```

### Lighting Randomization

```python
def randomize_lighting():
    # Random sun direction
    sun = rep.create.light(
        light_type="Distant",
        intensity=rep.distribution.uniform(500, 2000),
        rotation=rep.distribution.uniform((0, 0, 0), (90, 360, 0))
    )

    # Random point lights
    num_lights = rep.distribution.choice([1, 2, 3])

    for _ in range(num_lights):
        rep.create.light(
            light_type="Sphere",
            position=rep.distribution.uniform((-5, -5, 2), (5, 5, 5)),
            intensity=rep.distribution.uniform(1000, 5000),
            color=rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
        )
```

### Background Randomization

```python
# Random background images
backgrounds = [
    "indoor_1.hdr",
    "outdoor_1.hdr",
    "warehouse.hdr",
    "office.hdr"
]

def randomize_background():
    bg = rep.distribution.choice(backgrounds)
    rep.settings.set_render_settings(
        dome_light_path=bg,
        dome_light_intensity=rep.distribution.uniform(500, 1500)
    )
```

### Camera Randomization

```python
def randomize_camera():
    # Random camera position (spherical coordinates)
    radius = rep.distribution.uniform(2, 5)
    theta = rep.distribution.uniform(0, 360)
    phi = rep.distribution.uniform(20, 80)

    with camera:
        rep.modify.pose(
            position=(
                radius * np.sin(np.radians(phi)) * np.cos(np.radians(theta)),
                radius * np.sin(np.radians(phi)) * np.sin(np.radians(theta)),
                radius * np.cos(np.radians(phi))
            ),
            look_at=(0, 0, 0.5)
        )
```

## Complete Object Detection Dataset

```python
import omni.replicator.core as rep
import numpy as np

# Scene setup
with rep.new_layer():
    # Environment
    ground = rep.create.plane(scale=20, visible=True)

    # Objects to detect (multiple classes)
    objects = []

    # Class 1: Cubes
    for i in range(5):
        obj = rep.create.cube(
            semantics=[("class", "box")],
            scale=rep.distribution.uniform(0.3, 0.8)
        )
        objects.append(obj)

    # Class 2: Cylinders
    for i in range(5):
        obj = rep.create.cylinder(
            semantics=[("class", "bottle")],
            scale=(0.1, 0.1, 0.3)
        )
        objects.append(obj)

    # Class 3: Spheres
    for i in range(5):
        obj = rep.create.sphere(
            semantics=[("class", "ball")],
            scale=rep.distribution.uniform(0.2, 0.5)
        )
        objects.append(obj)

    # Camera
    camera = rep.create.camera(
        position=(5, 5, 3),
        look_at=(0, 0, 0)
    )

    render_product = rep.create.render_product(camera, (640, 480))

def randomize_scene():
    # Randomize all object poses
    for obj in objects:
        with obj:
            rep.modify.pose(
                position=rep.distribution.uniform((-4, -4, 0.2), (4, 4, 2)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

            # Random colors
            rep.randomizer.color(
                colors=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))
            )

    # Randomize lighting
    with rep.create.light(light_type="Dome"):
        rep.modify.attribute("intensity", rep.distribution.uniform(800, 1500))

    # Randomize camera
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((3, 3, 2), (7, 7, 5)),
            look_at=(0, 0, 1)
        )

    return objects

rep.randomizer.register(randomize_scene)

# Configure writer for object detection
writer = rep.WriterRegistry.get("COCOWriter")  # COCO format
writer.initialize(
    output_dir="./object_detection_dataset",
    classes=["box", "bottle", "ball"]
)
writer.attach([render_product])

# Generate dataset
with rep.trigger.on_frame(num_frames=5000):
    rep.randomizer.randomize_scene()

rep.orchestrator.run()

print("Dataset generated at ./object_detection_dataset")
```

## Pose Estimation Dataset

```python
# 6D pose estimation (position + orientation)
def generate_pose_dataset():
    # Load target object (e.g., tool, product)
    target_obj = rep.create.from_usd(
        "path/to/object.usd",
        semantics=[("class", "target_object")]
    )

    camera = rep.create.camera()
    render_product = rep.create.render_product(camera, (1024, 1024))

    def randomize():
        with target_obj:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.5, -0.5, 0.5), (0.5, 0.5, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((0.5, 0.5, 0.5), (2, 2, 2)),
                look_at=target_obj
            )

    rep.randomizer.register(randomize)

    # Writer with 3D bounding box
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="./pose_dataset",
        rgb=True,
        bounding_box_3d=True,  # 6D pose
        semantic_segmentation=True
    )
    writer.attach([render_product])

    with rep.trigger.on_frame(num_frames=10000):
        rep.randomizer.randomize()

    rep.orchestrator.run()
```

## Training with Synthetic Data

### Loading Data in PyTorch

```python
import torch
from torch.utils.data import Dataset
from PIL import Image
import json

class SyntheticDataset(Dataset):
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.annotations = json.load(open(f"{data_dir}/annotations.json"))

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        ann = self.annotations[idx]

        # Load image
        img_path = f"{self.data_dir}/{ann['image_path']}"
        image = Image.open(img_path).convert('RGB')

        # Get bounding boxes
        boxes = torch.tensor(ann['bounding_boxes'])
        labels = torch.tensor(ann['labels'])

        return image, boxes, labels

# Usage
dataset = SyntheticDataset("./object_detection_dataset")
dataloader = torch.utils.data.DataLoader(dataset, batch_size=32, shuffle=True)
```

### Mixing Synthetic and Real Data

```python
# Best practice: Start with 100% synthetic, gradually add real data

# Stage 1: 100% synthetic (baseline)
synthetic_dataset = SyntheticDataset("./synthetic_data")

# Stage 2: 90% synthetic, 10% real (fine-tuning)
mixed_dataset = torch.utils.data.ConcatDataset([
    synthetic_dataset,
    real_dataset
])

# Stage 3: 50/50 or more real data (production model)
```

## Best Practices

### 1. Diversify Your Data

```python
# Multiple environments
environments = ["warehouse", "office", "outdoor", "retail"]

for env in environments:
    # Generate 1000 samples per environment
    generate_dataset(env, num_samples=1000)
```

### 2. Match Real-World Distribution

```python
# If real data shows objects mostly upright, bias randomization:
def realistic_orientation():
    # 80% upright, 20% arbitrary
    if np.random.random() < 0.8:
        rotation = rep.distribution.uniform((0, 0, 0), (15, 15, 360))
    else:
        rotation = rep.distribution.uniform((0, 0, 0), (360, 360, 360))

    return rotation
```

### 3. Start Small, Scale Up

```python
# Generate small validation set first (100 samples)
# Visually inspect quality
# Then scale to full dataset (10k-1M samples)
```

### 4. Version Your Datasets

```python
output_dir = f"./datasets/v1.0_{datetime.now().strftime('%Y%m%d')}"
```

## Summary

In this chapter, you learned:

- Why synthetic data is essential for robot AI
- Isaac Sim's Replicator tool for data generation
- Domain randomization techniques (objects, lighting, camera, background)
- Creating complete datasets for object detection and pose estimation
- Training models with synthetic data in PyTorch
- Best practices for high-quality synthetic datasets

Synthetic data generation is a game-changer for robotics AI. With Isaac Sim, you can create unlimited training data that would be impossible or prohibitively expensive to collect in the real world.

**Module 3 Complete!** You now understand Isaac Sim's powerful capabilities for simulation, ROS 2 integration, and synthetic data generation.

## Further Reading

- Replicator Documentation: https://docs.omniverse.nvidia.com/py/replicator/
- Domain Randomization Paper: Tobin et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World."
- NVIDIA DOPE: https://github.com/NVlabs/Deep_Object_Pose
- Isaac ROS: https://github.com/NVIDIA-ISAAC-ROS

---

**Learning Check**:
- ✓ Use Replicator for synthetic data generation
- ✓ Apply domain randomization techniques
- ✓ Generate object detection datasets
- ✓ Create pose estimation training data
- ✓ Train models with synthetic data
- ✓ Follow best practices for data quality
