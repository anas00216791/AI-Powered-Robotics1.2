---
sidebar_position: 3
---

# Chapter 3: Sensor Simulation

## Introduction

Sensors are the eyes and ears of a robot. Simulating sensors accurately is crucial for developing perception and navigation algorithms without requiring physical hardware. In this chapter, you'll learn how to simulate LiDAR, depth cameras, IMUs, and other sensors in both Gazebo and Unity.

## Types of Robotic Sensors

### Exteroceptive Sensors
Measure external environment:
- **LiDAR**: Laser range finding for mapping and obstacle detection
- **Cameras**: RGB images for object recognition and tracking
- **Depth Cameras**: 3D perception (RGB-D sensors like RealSense, Kinect)
- **Ultrasonic**: Short-range distance measurement
- **Radar**: Long-range detection and velocity measurement

### Proprioceptive Sensors
Measure internal robot state:
- **IMU**: Inertial Measurement Unit (acceleration, angular velocity)
- **Encoders**: Joint positions and velocities
- **Force/Torque Sensors**: Interaction forces
- **GPS**: Global positioning (outdoor navigation)

## LiDAR Simulation

### LiDAR in Gazebo

Add a 2D LiDAR to your robot URDF:

```xml
<robot name="robot_with_lidar">

  <!-- Base link (your robot) -->
  <link name="base_link">
    <!-- Your robot definition -->
  </link>

  <!-- LiDAR link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting LiDAR to robot -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo LiDAR plugin -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>

      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>

        <range>
          <min>0.12</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>

      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**Testing the LiDAR**:

```bash
# Launch your robot with LiDAR
ros2 launch my_robot robot.launch.py

# View scan data
ros2 topic echo /scan

# Visualize in Rviz
rviz2
# Add > LaserScan > Topic: /scan
```

### 3D LiDAR (Velodyne-style)

```xml
<gazebo reference="velodyne_link">
  <sensor name="velodyne" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>
          <max_angle>0.2618</max_angle>
        </vertical>
      </scan>

      <range>
        <min>0.9</min>
        <max>130.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>

    <plugin name="velodyne_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=velodyne_points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>velodyne_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Camera Simulation

### RGB Camera in Gazebo

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera (RGB-D)

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>

    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.4</near>
        <far>10.0</far>
      </clip>
    </camera>

    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=depth/image_raw</remapping>
        <remapping>depth/image_raw:=depth/image_raw</remapping>
        <remapping>points:=depth/points</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>
      <min_depth>0.4</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Simulation

### IMU in Gazebo

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Visualizing IMU Data**:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)

    def imu_callback(self, msg):
        # Extract data
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity

        self.get_logger().info(
            f'Accel: x={accel.x:.2f}, y={accel.y:.2f}, z={accel.z:.2f} | '
            f'Gyro: x={gyro.x:.2f}, y={gyro.y:.2f}, z={gyro.z:.2f}'
        )

def main():
    rclpy.init()
    node = ImuVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Simulation in Unity

### LiDAR in Unity

```csharp
// LidarSensor.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarSensor : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 10.0f;
    public float scanRate = 10.0f;
    public string topicName = "/scan";

    private ROSConnection ros;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f / scanRate)
        {
            PublishScan();
            timer = 0;
        }
    }

    void PublishScan()
    {
        float angleIncrement = 2 * Mathf.PI / numRays;
        float[] ranges = new float[numRays];

        for (int i = 0; i < numRays; i++)
        {
            float angle = -Mathf.PI + i * angleIncrement;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
            }
            else
            {
                ranges[i] = maxRange;
                Debug.DrawRay(transform.position, direction * maxRange, Color.green);
            }
        }

        LaserScanMsg msg = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = gameObject.name
            },
            angle_min = -Mathf.PI,
            angle_max = Mathf.PI,
            angle_increment = angleIncrement,
            time_increment = 0,
            scan_time = 1.0f / scanRate,
            range_min = 0.1f,
            range_max = maxRange,
            ranges = ranges
        };

        ros.Publish(topicName, msg);
    }
}
```

### Depth Camera in Unity

```csharp
// DepthCamera.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(Camera))]
public class DepthCamera : MonoBehaviour
{
    public string topicName = "/depth/image_raw";
    public float publishRate = 10.0f;
    public float maxDepth = 10.0f;

    private Camera depthCamera;
    private ROSConnection ros;
    private RenderTexture depthTexture;
    private Texture2D texture2D;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        depthCamera = GetComponent<Camera>();
        depthCamera.depthTextureMode = DepthTextureMode.Depth;

        depthTexture = new RenderTexture(640, 480, 24, RenderTextureFormat.Depth);
        depthCamera.targetTexture = depthTexture;

        texture2D = new Texture2D(640, 480, TextureFormat.RFloat, false);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f / publishRate)
        {
            PublishDepthImage();
            timer = 0;
        }
    }

    void PublishDepthImage()
    {
        RenderTexture.active = depthTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();

        byte[] depthData = texture2D.GetRawTextureData();

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = gameObject.name
            },
            height = 480,
            width = 640,
            encoding = "32FC1",
            step = 640 * 4,
            data = depthData
        };

        ros.Publish(topicName, msg);
    }
}
```

## Sensor Noise Modeling

### Why Add Noise?

Real sensors are never perfect. Adding realistic noise to simulated sensors:
1. Tests algorithm robustness
2. Prevents overfitting to perfect data
3. Improves sim-to-real transfer

### Types of Sensor Noise

**Gaussian Noise**: Random fluctuations

```python
import numpy as np

# Add Gaussian noise to range measurement
true_range = 5.0
noise_stddev = 0.05
noisy_range = true_range + np.random.normal(0, noise_stddev)
```

**Quantization Noise**: Discrete measurement levels

```python
# Simulate 8-bit depth sensor (256 levels)
max_depth = 10.0
true_depth = 3.45
quantized_depth = np.round(true_depth / max_depth * 255) / 255 * max_depth
# Result: 3.45 → 3.4510 (closest 8-bit value)
```

**Outliers**: Occasional bad measurements

```python
# 1% chance of outlier
if np.random.random() < 0.01:
    measurement = np.random.uniform(0, max_range)  # Random outlier
else:
    measurement = true_value + np.random.normal(0, noise_stddev)
```

## Multi-Sensor Fusion

### Combining LiDAR and Camera

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.latest_scan = None
        self.latest_image = None
        self.bridge = CvBridge()

    def lidar_callback(self, msg):
        self.latest_scan = msg
        self.process_fusion()

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_fusion()

    def process_fusion(self):
        if self.latest_scan is None or self.latest_image is None:
            return

        # Find closest obstacle in LiDAR
        min_range = min(self.latest_scan.ranges)
        min_index = self.latest_scan.ranges.index(min_range)

        angle = self.latest_scan.angle_min + min_index * self.latest_scan.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_range:.2f}m at {np.degrees(angle):.1f}°'
        )

        # Project to image (simplified - assumes aligned sensors)
        # In real systems, use calibrated projection matrix
        image_x = int((angle + np.pi) / (2 * np.pi) * self.latest_image.shape[1])

        # Mark on image (for visualization)
        # cv2.circle(self.latest_image, (image_x, 240), 10, (0, 0, 255), -1)

def main():
    rclpy.init()
    node = SensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Calibration

### Camera Calibration

```python
#!/usr/bin/env python3

import numpy as np
import cv2

# Camera intrinsic parameters
fx = 525.0  # Focal length x (pixels)
fy = 525.0  # Focal length y (pixels)
cx = 319.5  # Principal point x
cy = 239.5  # Principal point y

camera_matrix = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
])

# Distortion coefficients
dist_coeffs = np.array([0.1, -0.15, 0, 0, 0])

# Project 3D point to 2D image
def project_point(point_3d, camera_matrix, dist_coeffs):
    """
    Project a 3D point to 2D image coordinates.

    Args:
        point_3d: [x, y, z] in camera frame
        camera_matrix: 3x3 intrinsic matrix
        dist_coeffs: Distortion coefficients

    Returns:
        [u, v] image coordinates
    """
    point_3d = np.array(point_3d).reshape(1, 1, 3)
    rvec = np.zeros((3, 1))
    tvec = np.zeros((3, 1))

    point_2d, _ = cv2.projectPoints(
        point_3d, rvec, tvec, camera_matrix, dist_coeffs
    )

    return point_2d[0][0]

# Example usage
point_3d = [1.0, 0.5, 3.0]  # 1m right, 0.5m up, 3m forward
point_2d = project_point(point_3d, camera_matrix, dist_coeffs)
print(f"3D point {point_3d} projects to 2D point {point_2d}")
```

## Best Practices for Sensor Simulation

1. **Match Real Sensor Specs**: Use manufacturer datasheets for:
   - Field of view
   - Range limits
   - Update rates
   - Resolution

2. **Add Realistic Noise**: Don't use perfect measurements—real sensors are noisy

3. **Model Failure Modes**: Sensors fail in predictable ways:
   - LiDAR: Reflective surfaces cause bad readings
   - Cameras: Glare from bright lights
   - IMU: Drift over time

4. **Validate Against Real Data**: Compare simulated and real sensor outputs

5. **Start Simple**: Begin with perfect sensors, add complexity gradually

## Summary

In this chapter, you learned:

- How to simulate LiDAR sensors in Gazebo and Unity
- How to add RGB and depth cameras to your robots
- How to simulate IMUs for robot state estimation
- How to model realistic sensor noise and failures
- How to perform multi-sensor fusion
- Best practices for sensor calibration and validation

Sensor simulation is critical for developing robust perception systems. With accurate sensor models, you can develop and test algorithms in simulation before deploying to real robots—saving time and reducing hardware wear.

## Further Reading

- Gazebo Sensor Documentation: http://gazebosim.org/tutorials?cat=sensors
- ROS 2 Sensor Messages: https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs
- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Point Cloud Library (PCL): https://pointclouds.org/

---

**Learning Check**:
- ✓ Configure LiDAR sensors in simulation
- ✓ Add cameras (RGB and depth) to robots
- ✓ Simulate IMU sensors with realistic noise
- ✓ Understand sensor noise modeling
- ✓ Implement basic sensor fusion
- ✓ Calibrate simulated sensors

**Module 2 Complete!** You now understand physics simulation (Gazebo), rendering (Unity), and sensor modeling—the complete simulation toolkit for robotics development.
