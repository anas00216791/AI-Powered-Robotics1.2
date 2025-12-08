---
sidebar_position: 2
---

# Chapter 2: Using Pre-trained VLA Models

## Introduction

Training VLA models from scratch requires massive datasets and computational resources. Fortunately, several pre-trained models are available that you can use immediately or fine-tune for your specific tasks. In this chapter, you'll learn how to deploy and use these models in your robotics projects.

## Available Pre-trained Models

### OpenVLA (Recommended for Getting Started)

**Open-source 7B parameter model**

- Repository: https://github.com/openvla/openvla
- License: MIT (fully open)
- Training Data: Open X-Embodiment (900K+ trajectories)
- Hardware: Runs on single RTX 4090 or A100

### RT-1 Checkpoints

**Google's robotics transformer**

- Available through: TensorFlow Hub
- Pre-trained on: 130K demonstrations, 700 tasks
- Best for: Tabletop manipulation

### Octo

**Open-source foundation model**

- Repository: https://github.com/octo-models/octo
- Size: 93M parameters (efficient!)
- Strength: Fast fine-tuning with limited data

## Setting Up OpenVLA

### Installation

```bash
# Create environment
conda create -n openvla python=3.10
conda activate openvla

# Install PyTorch (adjust for your CUDA version)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Install OpenVLA
git clone https://github.com/openvla/openvla
cd openvla
pip install -e .

# Install additional dependencies
pip install transformers accelerate
```

### Download Pre-trained Weights

```bash
# Download model weights (~14 GB)
python scripts/download_model.py --model openvla-7b

# Model saved to: ./checkpoints/openvla-7b/
```

## Basic Inference

### Loading the Model

```python
import torch
from openvla import OpenVLA
from PIL import Image
import numpy as np

# Load model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = OpenVLA.from_pretrained("openvla-7b")
model = model.to(device)
model.eval()

print(f"Model loaded on {device}")
print(f"Parameters: {sum(p.numel() for p in model.parameters())/1e9:.2f}B")
```

### Single-Step Prediction

```python
def predict_action(image, instruction, robot_state):
    """
    Predict robot action from observation.

    Args:
        image: PIL Image or numpy array (H, W, 3)
        instruction: str, e.g., "pick up the red block"
        robot_state: numpy array (7,) joint positions

    Returns:
        action: numpy array (8,) [7 joint velocities + gripper]
    """
    # Preprocess image
    if isinstance(image, np.ndarray):
        image = Image.fromarray(image)

    # Model expects specific input format
    inputs = model.processor(
        images=image,
        text=instruction,
        return_tensors="pt"
    ).to(device)

    # Add robot state
    inputs['robot_state'] = torch.tensor(robot_state).unsqueeze(0).to(device)

    # Predict
    with torch.no_grad():
        outputs = model(**inputs)
        action = outputs.action.cpu().numpy()[0]

    return action

# Example usage
image = Image.open("camera_view.jpg")
instruction = "pick up the blue mug"
robot_state = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0])  # Initial pose

action = predict_action(image, instruction, robot_state)
print(f"Predicted action: {action}")
```

## Integration with ROS 2

### VLA Action Server

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerAction
from cv_bridge import CvBridge

import torch
from openvla import OpenVLA
import numpy as np

class VLAActionServer(Node):
    def __init__(self):
        super().__init__('vla_action_server')

        # Load model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = OpenVLA.from_pretrained("openvla-7b")
        self.model = self.model.to(self.device)
        self.model.eval()

        self.get_logger().info(f"VLA model loaded on {self.device}")

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.instruction_sub = self.create_subscription(
            String,
            '/vla/instruction',
            self.instruction_callback,
            10
        )

        # Publisher for commanded actions
        self.action_pub = self.create_publisher(
            JointTrajectoryControllerAction,
            '/joint_trajectory_controller/action',
            10
        )

        # State
        self.latest_image = None
        self.latest_joints = None
        self.current_instruction = None
        self.bridge = CvBridge()

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def joint_callback(self, msg):
        self.latest_joints = np.array(msg.position)

    def instruction_callback(self, msg):
        self.current_instruction = msg.data
        self.get_logger().info(f"New instruction: {self.current_instruction}")

    def control_loop(self):
        # Check we have all required inputs
        if (self.latest_image is None or
            self.latest_joints is None or
            self.current_instruction is None):
            return

        # Predict action
        action = self.predict_action(
            self.latest_image,
            self.current_instruction,
            self.latest_joints
        )

        # Publish action
        self.publish_action(action)

    def predict_action(self, image, instruction, robot_state):
        # Convert numpy image to PIL
        from PIL import Image as PILImage
        pil_image = PILImage.fromarray(image)

        # Prepare inputs
        inputs = self.model.processor(
            images=pil_image,
            text=instruction,
            return_tensors="pt"
        ).to(self.device)

        inputs['robot_state'] = torch.tensor(robot_state).unsqueeze(0).to(self.device)

        # Predict
        with torch.no_grad():
            outputs = self.model(**inputs)
            action = outputs.action.cpu().numpy()[0]

        return action

    def publish_action(self, action):
        # Convert action to ROS message
        # action: [7 joint velocities + gripper]
        # (Implementation depends on your robot's control interface)

        self.get_logger().info(f"Publishing action: {action}")

def main():
    rclpy.init()
    node = VLAActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Usage

```bash
# Terminal 1: Run VLA server
ros2 run my_robot vla_action_server

# Terminal 2: Send instruction
ros2 topic pub /vla/instruction std_msgs/String "data: 'pick up the red block'"

# Terminal 3: Monitor actions
ros2 topic echo /joint_trajectory_controller/action
```

## Real-Time Performance Optimization

### Model Quantization

Reduce model size and increase speed:

```python
from openvla import OpenVLA
import torch

# Load model
model = OpenVLA.from_pretrained("openvla-7b")

# Quantize to 8-bit (2x speedup, minimal accuracy loss)
model = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear},
    dtype=torch.qint8
)

# Save quantized model
torch.save(model.state_dict(), "openvla-7b-int8.pth")
```

### Batching for Throughput

```python
def predict_batch(images, instructions, robot_states):
    """
    Predict actions for multiple observations in parallel.

    Args:
        images: list of PIL Images
        instructions: list of strings
        robot_states: numpy array (batch, 7)

    Returns:
        actions: numpy array (batch, 8)
    """
    inputs = model.processor(
        images=images,
        text=instructions,
        return_tensors="pt",
        padding=True
    ).to(device)

    inputs['robot_state'] = torch.tensor(robot_states).to(device)

    with torch.no_grad():
        outputs = model(**inputs)
        actions = outputs.action.cpu().numpy()

    return actions

# Use when processing multiple robots or parallel rollouts
images = [image1, image2, image3]
instructions = ["pick block", "open drawer", "close gripper"]
robot_states = np.array([state1, state2, state3])

actions = predict_batch(images, instructions, robot_states)
```

## Evaluation and Testing

### Success Rate Measurement

```python
def evaluate_model(model, test_tasks, num_trials=10):
    """
    Evaluate model success rate on test tasks.

    Args:
        model: VLA model
        test_tasks: list of (image, instruction, robot_state) tuples
        num_trials: attempts per task

    Returns:
        success_rate: float
    """
    successes = 0
    total = 0

    for task in test_tasks:
        for trial in range(num_trials):
            # Reset environment
            reset_robot()

            # Execute task
            success = execute_task(model, task)

            if success:
                successes += 1
            total += 1

            print(f"Task {task['name']}, Trial {trial}: {'✓' if success else '✗'}")

    success_rate = successes / total
    print(f"\nOverall Success Rate: {success_rate:.1%}")

    return success_rate

# Example
test_tasks = [
    {"name": "pick_cube", "image": img1, "instruction": "pick up cube", "state": state1},
    {"name": "open_drawer", "image": img2, "instruction": "open drawer", "state": state2},
]

success_rate = evaluate_model(model, test_tasks, num_trials=5)
```

### Failure Analysis

```python
def analyze_failures(model, failed_tasks):
    """
    Analyze why tasks failed.
    """
    failure_modes = {
        "perception": 0,    # Wrong object detected
        "manipulation": 0,  # Execution error
        "language": 0,      # Misunderstood instruction
        "other": 0
    }

    for task in failed_tasks:
        # Check if model detected correct object
        predicted_action = model.predict(task['image'], task['instruction'], task['state'])

        # Classify failure
        if not task['object_detected']:
            failure_modes["perception"] += 1
        elif task['collision']:
            failure_modes["manipulation"] += 1
        elif task['wrong_object']:
            failure_modes["language"] += 1
        else:
            failure_modes["other"] += 1

    print("Failure Modes:")
    for mode, count in failure_modes.items():
        print(f"  {mode}: {count}")

    return failure_modes
```

## Using Multiple Models

### Ensemble Predictions

```python
def ensemble_predict(models, image, instruction, robot_state):
    """
    Average predictions from multiple models.
    """
    actions = []

    for model in models:
        action = model.predict(image, instruction, robot_state)
        actions.append(action)

    # Average predictions
    ensemble_action = np.mean(actions, axis=0)

    return ensemble_action

# Load multiple models
models = [
    OpenVLA.from_pretrained("openvla-7b"),
    OpenVLA.from_pretrained("octo-base"),
]

action = ensemble_predict(models, image, instruction, robot_state)
```

### Model Selection Based on Task

```python
def select_model(instruction):
    """
    Choose best model based on task type.
    """
    if "pick" in instruction or "grasp" in instruction:
        return grasp_specialist_model
    elif "open" in instruction or "close" in instruction:
        return manipulation_model
    else:
        return general_purpose_model

# Usage
model = select_model("pick up the mug")
action = model.predict(image, "pick up the mug", robot_state)
```

## Best Practices

1. **Start with Demo Mode**: Test model predictions without executing on real robot
2. **Safety Limits**: Constrain action space to prevent damage
3. **Gradual Deployment**: Test on simple tasks before complex ones
4. **Monitor Performance**: Track success rate over time
5. **Collect Failure Cases**: Use failures to improve via fine-tuning

## Troubleshooting

### Low Success Rate

**Problem**: Model succeeds less than 50% of time
**Solutions**:
- Check if task is in pre-training distribution
- Verify image quality (lighting, resolution)
- Ensure instruction is clear and specific
- Fine-tune on your specific environment

### Slow Inference

**Problem**: Less than 3 Hz control rate
**Solutions**:
- Use GPU (not CPU)
- Quantize model to int8
- Reduce image resolution
- Use smaller model variant

### Unexpected Actions

**Problem**: Robot makes strange movements
**Solutions**:
- Check coordinate frame conventions
- Verify action scaling
- Add action smoothing/filtering
- Clip actions to safe ranges

## Summary

In this chapter, you learned:

- How to install and load pre-trained VLA models
- Basic inference and action prediction
- Integration with ROS 2 for robot control
- Performance optimization techniques
- Evaluation and testing methodologies
- Best practices for deployment

With pre-trained VLA models, you can quickly prototype robot applications without training from scratch. In the next chapter, you'll learn how to fine-tune these models for your specific tasks and robots.

## Further Reading

- OpenVLA Documentation: https://openvla.github.io/
- Octo Models: https://octo-models.github.io/
- Open X-Embodiment: https://robotics-transformer-x.github.io/

---

**Learning Check**:
- ✓ Load and run pre-trained VLA models
- ✓ Integrate VLA with ROS 2
- ✓ Optimize model performance
- ✓ Evaluate model success rate
- ✓ Troubleshoot common issues
