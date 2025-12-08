---
sidebar_position: 3
---

# Chapter 3: Fine-tuning and Deployment

## Introduction

While pre-trained VLA models work well on common tasks, fine-tuning is essential for optimal performance on your specific robot, environment, and tasks. In this chapter, you'll learn how to collect demonstration data, fine-tune VLA models efficiently, and deploy them reliably on real robots.

## Why Fine-tune?

Pre-trained models may struggle with:
- **Robot-specific kinematics**: Different arm lengths, joint limits
- **Custom objects**: Novel tools, products not in training data
- **Unique environments**: Your lab, warehouse, or home layout
- **Specialized tasks**: Industry-specific manipulation

**Fine-tuning bridges this gap with 10-1000 demonstrations** (vs millions for training from scratch).

## Data Collection Pipeline

### 1. Teleoperation Setup

Collect demonstrations via human teleoperation:

```python
import numpy as np
import time
from datetime import datetime
import pickle

class DemonstrationRecorder:
    def __init__(self, robot, camera):
        self.robot = robot
        self.camera = camera
        self.demonstrations = []
        self.current_demo = []

    def start_recording(self, instruction):
        """Start recording a new demonstration."""
        print(f"Recording: '{instruction}'")
        self.current_demo = {
            'instruction': instruction,
            'frames': [],
            'timestamp': datetime.now()
        }

    def record_frame(self):
        """Record single timestep."""
        frame = {
            'image': self.camera.get_rgb(),
            'robot_state': self.robot.get_joint_positions(),
            'action': self.robot.get_joint_velocities(),
            'gripper': self.robot.get_gripper_state()
        }
        self.current_demo['frames'].append(frame)

    def stop_recording(self, success=True):
        """Stop and save demonstration."""
        if success:
            self.demonstrations.append(self.current_demo)
            print(f"Demo saved: {len(self.current_demo['frames'])} frames")
        else:
            print("Demo discarded (failed)")

        self.current_demo = []

    def save_dataset(self, filename):
        """Save all demonstrations to file."""
        with open(filename, 'wb') as f:
            pickle.dump(self.demonstrations, f)
        print(f"Saved {len(self.demonstrations)} demonstrations to {filename}")

# Usage
recorder = DemonstrationRecorder(robot, camera)

# Collect demonstration
recorder.start_recording("pick up the red block")
for _ in range(100):  # Record for ~10 seconds at 10 Hz
    recorder.record_frame()
    time.sleep(0.1)
recorder.stop_recording(success=True)

# Save dataset
recorder.save_dataset("my_demonstrations.pkl")
```

### 2. Quality Control

```python
def filter_demonstrations(demos, min_length=10, max_length=500):
    """
    Remove bad demonstrations.

    Criteria:
    - Too short (likely failed immediately)
    - Too long (likely got stuck)
    - Low variance (robot didn't move)
    """
    filtered = []

    for demo in demos:
        frames = demo['frames']

        # Check length
        if len(frames) < min_length or len(frames) > max_length:
            continue

        # Check robot moved
        states = np.array([f['robot_state'] for f in frames])
        variance = np.var(states, axis=0).mean()

        if variance < 0.001:  # Robot barely moved
            continue

        filtered.append(demo)

    print(f"Kept {len(filtered)}/{len(demos)} demonstrations")
    return filtered

# Apply filtering
clean_demos = filter_demonstrations(demonstrations)
```

### 3. Data Augmentation

Increase dataset size with augmentation:

```python
import imgaug.augmenters as iaa

def augment_demonstrations(demos, num_augmentations=3):
    """
    Create augmented versions of demonstrations.

    Augmentations:
    - Color jitter
    - Brightness/contrast
    - Gaussian noise
    - Random crops
    """
    augmenter = iaa.Sequential([
        iaa.Sometimes(0.5, iaa.Multiply((0.8, 1.2))),  # Brightness
        iaa.Sometimes(0.5, iaa.Add((-20, 20))),        # Contrast
        iaa.Sometimes(0.3, iaa.GaussianBlur(sigma=(0, 1.0))),
        iaa.Sometimes(0.3, iaa.AdditiveGaussianNoise(scale=(0, 0.05*255)))
    ])

    augmented = []

    for demo in demos:
        # Original
        augmented.append(demo)

        # Augmented versions
        for _ in range(num_augmentations):
            aug_demo = demo.copy()
            aug_demo['frames'] = []

            for frame in demo['frames']:
                aug_frame = frame.copy()
                aug_frame['image'] = augmenter.augment_image(frame['image'])
                aug_demo['frames'].append(aug_frame)

            augmented.append(aug_demo)

    print(f"Augmented {len(demos)} → {len(augmented)} demonstrations")
    return augmented

# Apply augmentation
augmented_demos = augment_demonstrations(clean_demos, num_augmentations=2)
```

## Fine-tuning Process

### 1. Prepare Dataset

```python
import torch
from torch.utils.data import Dataset, DataLoader
from PIL import Image

class VLADataset(Dataset):
    def __init__(self, demonstrations, processor):
        self.demonstrations = demonstrations
        self.processor = processor

        # Flatten demonstrations into (state, action) pairs
        self.samples = []
        for demo in demonstrations:
            for i, frame in enumerate(demo['frames']):
                if i < len(demo['frames']) - 1:  # Have next action
                    self.samples.append({
                        'image': frame['image'],
                        'instruction': demo['instruction'],
                        'robot_state': frame['robot_state'],
                        'action': demo['frames'][i+1]['action']  # Next action
                    })

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]

        # Process image
        image = Image.fromarray(sample['image'])
        inputs = self.processor(
            images=image,
            text=sample['instruction'],
            return_tensors="pt"
        )

        # Add robot state
        inputs['robot_state'] = torch.tensor(sample['robot_state'])
        inputs['action'] = torch.tensor(sample['action'])

        # Remove batch dimension
        for key in inputs:
            if isinstance(inputs[key], torch.Tensor):
                inputs[key] = inputs[key].squeeze(0)

        return inputs

# Create dataset
from openvla import OpenVLA

model = OpenVLA.from_pretrained("openvla-7b")
processor = model.processor

dataset = VLADataset(demonstrations, processor)
dataloader = DataLoader(dataset, batch_size=8, shuffle=True)

print(f"Dataset size: {len(dataset)} samples")
```

### 2. Fine-tuning Loop

```python
import torch
import torch.nn as nn
from torch.optim import AdamW
from tqdm import tqdm

def finetune_vla(model, dataloader, epochs=10, lr=1e-5):
    """
    Fine-tune VLA model on custom demonstrations.

    Args:
        model: Pre-trained VLA model
        dataloader: Training data
        epochs: Number of training epochs
        lr: Learning rate (small for fine-tuning)

    Returns:
        Fine-tuned model
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)
    model.train()

    # Optimizer
    optimizer = AdamW(model.parameters(), lr=lr, weight_decay=0.01)

    # Loss function
    criterion = nn.MSELoss()

    # Training loop
    for epoch in range(epochs):
        total_loss = 0
        num_batches = 0

        pbar = tqdm(dataloader, desc=f"Epoch {epoch+1}/{epochs}")
        for batch in pbar:
            # Move to device
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                     for k, v in batch.items()}

            # Forward pass
            outputs = model(
                pixel_values=batch['pixel_values'],
                input_ids=batch['input_ids'],
                attention_mask=batch['attention_mask'],
                robot_state=batch['robot_state']
            )

            # Compute loss
            predicted_action = outputs.action
            target_action = batch['action']
            loss = criterion(predicted_action, target_action)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # Logging
            total_loss += loss.item()
            num_batches += 1
            pbar.set_postfix({'loss': total_loss/num_batches})

        avg_loss = total_loss / num_batches
        print(f"Epoch {epoch+1}: Average Loss = {avg_loss:.4f}")

    return model

# Fine-tune
finetuned_model = finetune_vla(model, dataloader, epochs=10, lr=1e-5)

# Save
torch.save(finetuned_model.state_dict(), "finetuned_vla.pth")
```

### 3. Low-Rank Adaptation (LoRA)

Efficient fine-tuning with LoRA (updates only small adapters):

```python
from peft import get_peft_model, LoraConfig, TaskType

def finetune_with_lora(model, dataloader, epochs=10):
    """
    Fine-tune using LoRA (parameter-efficient).

    Advantages:
    - 10-100x fewer parameters to train
    - Faster training
    - Less memory required
    """
    # Configure LoRA
    lora_config = LoraConfig(
        task_type=TaskType.SEQ_2_SEQ_LM,
        r=8,  # Rank of adaptation matrices
        lora_alpha=32,
        lora_dropout=0.1,
        target_modules=["q_proj", "v_proj"]  # Which layers to adapt
    )

    # Wrap model with LoRA
    model = get_peft_model(model, lora_config)

    print(f"Trainable parameters: {model.print_trainable_parameters()}")

    # Train (same as before)
    model = finetune_vla(model, dataloader, epochs=epochs)

    return model

# Use LoRA for efficient fine-tuning
lora_model = finetune_with_lora(model, dataloader, epochs=20)
```

## Evaluation After Fine-tuning

### Benchmark on Test Set

```python
def evaluate_model(model, test_dataloader):
    """
    Evaluate fine-tuned model.

    Metrics:
    - Action prediction error (MSE)
    - Success rate on held-out tasks
    """
    model.eval()
    device = next(model.parameters()).device

    total_error = 0
    num_samples = 0

    with torch.no_grad():
        for batch in test_dataloader:
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                     for k, v in batch.items()}

            # Predict
            outputs = model(
                pixel_values=batch['pixel_values'],
                input_ids=batch['input_ids'],
                attention_mask=batch['attention_mask'],
                robot_state=batch['robot_state']
            )

            # Compute error
            predicted = outputs.action
            target = batch['action']
            error = torch.mean((predicted - target) ** 2)

            total_error += error.item()
            num_samples += 1

    avg_error = total_error / num_samples
    print(f"Test Set MSE: {avg_error:.4f}")

    return avg_error

# Evaluate
test_error = evaluate_model(finetuned_model, test_dataloader)
```

### Real-World Testing

```python
def test_on_robot(model, test_tasks):
    """
    Test fine-tuned model on real robot.

    Returns:
        success_rate: Fraction of successful task executions
    """
    successes = 0
    total = len(test_tasks)

    for i, task in enumerate(test_tasks):
        print(f"\nTest {i+1}/{total}: {task['instruction']}")

        # Reset environment
        reset_environment(task['initial_state'])

        # Execute with model
        success = execute_task_with_model(model, task)

        if success:
            print("✓ Success")
            successes += 1
        else:
            print("✗ Failed")

    success_rate = successes / total
    print(f"\nSuccess Rate: {success_rate:.1%}")

    return success_rate

# Test
tasks = [
    {'instruction': 'pick up red block', 'initial_state': ...},
    {'instruction': 'place block in container', 'initial_state': ...},
    {'instruction': 'open drawer', 'initial_state': ...},
]

success_rate = test_on_robot(finetuned_model, tasks)
```

## Deployment Considerations

### 1. Safety Wrappers

```python
class SafeVLAController:
    """
    Wraps VLA model with safety checks.
    """
    def __init__(self, model, robot):
        self.model = model
        self.robot = robot

        # Safety limits
        self.max_joint_velocity = 1.0  # rad/s
        self.max_gripper_force = 50.0  # N
        self.workspace_bounds = {
            'x': (-0.5, 0.5),
            'y': (-0.5, 0.5),
            'z': (0.0, 1.0)
        }

    def predict_safe_action(self, image, instruction, robot_state):
        # Get model prediction
        action = self.model.predict(image, instruction, robot_state)

        # Clip joint velocities
        action[:7] = np.clip(action[:7], -self.max_joint_velocity, self.max_joint_velocity)

        # Check workspace bounds (forward kinematics)
        end_effector_pos = self.robot.forward_kinematics(robot_state + action[:7] * 0.1)

        for axis, (min_val, max_val) in self.workspace_bounds.items():
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            if not (min_val <= end_effector_pos[idx] <= max_val):
                print(f"Warning: Action would violate {axis} workspace limit")
                action[:7] *= 0.5  # Reduce velocity

        return action

# Usage
safe_controller = SafeVLAController(finetuned_model, robot)
action = safe_controller.predict_safe_action(image, instruction, state)
```

### 2. Failure Recovery

```python
class RobustVLAController:
    """
    VLA controller with failure detection and recovery.
    """
    def __init__(self, model, robot):
        self.model = model
        self.robot = robot
        self.stuck_threshold = 10  # Frames without progress
        self.stuck_counter = 0
        self.previous_state = None

    def execute_task(self, instruction):
        print(f"Executing: {instruction}")

        for step in range(1000):  # Max 1000 steps
            # Get observation
            image = self.robot.get_camera_image()
            state = self.robot.get_joint_positions()

            # Predict action
            action = self.model.predict(image, instruction, state)

            # Execute
            self.robot.apply_action(action)

            # Check for stuck
            if self.previous_state is not None:
                movement = np.linalg.norm(state - self.previous_state)

                if movement < 0.001:  # Barely moved
                    self.stuck_counter += 1
                else:
                    self.stuck_counter = 0

            self.previous_state = state

            # Recovery if stuck
            if self.stuck_counter >= self.stuck_threshold:
                print("Robot appears stuck. Attempting recovery...")
                self.recover()
                self.stuck_counter = 0

            # Check task completion
            if self.is_task_complete(image, instruction):
                print("Task completed successfully!")
                return True

        print("Task timeout")
        return False

    def recover(self):
        """Recovery behavior when stuck."""
        # Retract arm slightly
        current_state = self.robot.get_joint_positions()
        retract_action = -0.1 * current_state  # Move toward zero position
        self.robot.apply_action(retract_action)
        time.sleep(1.0)

    def is_task_complete(self, image, instruction):
        # Use vision-language model to check completion
        # Or simple heuristics
        return False  # Implement based on your setup

# Usage
controller = RobustVLAController(finetuned_model, robot)
success = controller.execute_task("pick up the mug")
```

### 3. Continuous Learning

```python
class ContinuousLearningVLA:
    """
    VLA that improves from deployment experience.
    """
    def __init__(self, model):
        self.model = model
        self.new_demonstrations = []

    def execute_with_logging(self, instruction):
        # Execute task
        demo = self.record_demonstration(instruction)

        # Ask human: was this successful?
        success = input("Was the task successful? (y/n): ").lower() == 'y'

        if success:
            self.new_demonstrations.append(demo)
            print(f"Collected {len(self.new_demonstrations)} new demonstrations")

        # Periodically retrain
        if len(self.new_demonstrations) >= 100:
            print("Retraining model with new data...")
            self.retrain()

    def retrain(self):
        # Fine-tune on new demonstrations
        dataset = VLADataset(self.new_demonstrations, self.model.processor)
        dataloader = DataLoader(dataset, batch_size=8)

        self.model = finetune_vla(self.model, dataloader, epochs=5)

        # Clear buffer
        self.new_demonstrations = []

        print("Model updated!")

# Usage
continuous_model = ContinuousLearningVLA(finetuned_model)
```

## Best Practices Summary

1. **Data Quality > Quantity**: 100 good demos better than 1000 poor ones
2. **Start Small**: Fine-tune on 1 task, validate, then expand
3. **Regular Evaluation**: Test on robot frequently during training
4. **Safety First**: Always use safety wrappers in deployment
5. **Monitor Performance**: Track success rate over time
6. **Iterate**: Fine-tune → test → collect failures → re-fine-tune

## Summary

In this chapter, you learned:

- How to collect high-quality demonstration data
- Fine-tuning techniques (full fine-tuning and LoRA)
- Evaluation methods for fine-tuned models
- Deployment strategies with safety wrappers
- Failure recovery and continuous learning

With these skills, you can take pre-trained VLA models and adapt them to your specific robotics applications, achieving high performance with minimal training data.

**Congratulations!** You've completed the AI-Powered Robotics book. You now understand:
- ROS 2 fundamentals and Python integration
- Simulation with Gazebo, Unity, and Isaac Sim
- Sensor modeling and synthetic data generation
- Vision-Language-Action models for robot control

You're ready to build intelligent robotic systems that bridge AI and physical actuation!

## Further Reading

- LoRA Paper: Hu, E. J., et al. (2021). "LoRA: Low-Rank Adaptation of Large Language Models."
- Behavioral Cloning: Pomerleau, D. A. (1988). "ALVINN: An Autonomous Land Vehicle in a Neural Network."
- Open X-Embodiment: https://robotics-transformer-x.github.io/
- Best Practices: Google Robotics Team. "Robotics at Google: Best Practices."

---

**Final Learning Check**:
- ✓ Collect demonstration data
- ✓ Fine-tune VLA models efficiently
- ✓ Evaluate model performance
- ✓ Deploy safely on real robots
- ✓ Implement continuous learning

**Next Steps**:
- Build your own robot project
- Contribute to open-source robotics
- Join the robotics research community
- Keep learning and experimenting!
