---
sidebar_position: 2
---

# Chapter 2: Unity for High-Fidelity Rendering

## Introduction

While Gazebo excels at physics simulation, Unity brings professional-grade rendering capabilities to robotics. Unity is a powerful game engine used to create photorealistic environments, simulate human-robot interactions, and generate synthetic data for training perception models (Juliani et al., 2018).

In this chapter, you'll learn how to use Unity for robotics visualization, integrate it with ROS 2, and create compelling scenarios for testing robot behaviors.

## Why Unity for Robotics?

### Advantages of Unity

1. **Photorealistic Rendering**: High-quality graphics for perception training
2. **Asset Ecosystem**: Vast library of 3D models, materials, and environments
3. **Human Animation**: Advanced tools for simulating human behaviors
4. **Cross-Platform**: Deploy to Windows, Linux, macOS, mobile, and VR/AR
5. **Performance**: Optimized rendering pipeline with GPU acceleration
6. **UI Tools**: Easy creation of control interfaces and dashboards

### Unity vs Gazebo

Use Unity when you need:
- High-quality visuals for presentations or videos
- Human-robot interaction scenarios
- Synthetic image data for training vision models
- Consumer-facing demonstrations

Use Gazebo when you need:
- Precise physics simulation
- Traditional ROS workflow
- Headless (no-display) simulation on servers

## Setting Up Unity for Robotics

### Installation

1. **Install Unity Hub**: Download from https://unity.com/download

2. **Install Unity Editor**:
   - Recommended version: Unity 2021.3 LTS or 2022.3 LTS
   - Include modules: Linux Build Support, Documentation

3. **Install Required Packages**:
   ```bash
   # Via Unity Package Manager (in Unity Editor):
   # Window > Package Manager

   # Search and install:
   # - Robotics packages (if available)
   # - ProBuilder (for level design)
   # - Cinemachine (for camera control)
   ```

### Creating Your First Unity Robotics Project

1. Open Unity Hub
2. Click "New Project"
3. Choose "3D" template
4. Name it "RoboticsSimulation"
5. Click "Create"

## Unity-ROS 2 Integration

### Installing ROS-TCP Connector

Unity communicates with ROS 2 via TCP/IP using the ROS-TCP connector.

**In Unity**:

1. Open Package Manager (Window > Package Manager)
2. Click "+" > "Add package from git URL"
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Click "Add"

**In ROS 2 workspace**:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Configuring the Connection

**In Unity**:

1. Go to Robotics > ROS Settings
2. Set ROS IP Address: `127.0.0.1` (localhost)
3. Set ROS Port: `10000`
4. Set Protocol: `ROS 2`

**Launch ROS-TCP Endpoint**:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Creating a Robot in Unity

### Importing a Robot Model

Unity supports several 3D formats:
- FBX (recommended for robots)
- OBJ
- URDF (via URDF Importer package)

**Method 1: Using URDF Importer**

1. Install URDF Importer:
   ```
   Package Manager > Add from git URL:
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

2. Import URDF:
   ```
   Assets > Import Robot from URDF
   ```

3. Select your URDF file from Module 1
4. Click "Import"

**Method 2: Manual Import**

1. Export your robot from Blender or CAD software as FBX
2. Drag FBX file into Unity's Project window
3. Drag the model from Project to Scene

### Adding Articulation Bodies

Unity's Articulation Body component simulates robot joints:

```csharp
// ArticulationBodyController.cs
using UnityEngine;

public class ArticulationBodyController : MonoBehaviour
{
    private ArticulationBody[] articulationBodies;

    void Start()
    {
        // Get all articulation bodies in robot
        articulationBodies = GetComponentsInChildren<ArticulationBody>();

        foreach (var ab in articulationBodies)
        {
            Debug.Log($"Joint: {ab.name}, DOF: {ab.dofCount}");
        }
    }

    // Move a specific joint
    public void SetJointPosition(int jointIndex, float position)
    {
        if (jointIndex < articulationBodies.Length)
        {
            var drive = articulationBodies[jointIndex].xDrive;
            drive.target = position * Mathf.Rad2Deg; // Convert radians to degrees
            articulationBodies[jointIndex].xDrive = drive;
        }
    }
}
```

Attach this script to your robot's root GameObject.

## Building Realistic Environments

### Using ProBuilder for Level Design

ProBuilder allows you to create custom geometry directly in Unity:

1. Open ProBuilder (Tools > ProBuilder > ProBuilder Window)
2. Create basic shapes (New Shape button)
3. Available shapes: Cube, Plane, Stairs, Arch, etc.

**Example: Creating a room**:

```
1. Create Plane (10x10) for floor
2. Create 4 Cubes (10x3x0.2) for walls
3. Position walls around floor perimeter
4. Add materials and lighting
```

### Materials and Lighting

**Creating a Material**:

1. Right-click in Project > Create > Material
2. Select the material
3. Choose Shader: Standard or URP/Lit
4. Adjust properties:
   - Albedo (base color)
   - Metallic (0 = non-metal, 1 = metal)
   - Smoothness (0 = rough, 1 = smooth)

**Lighting Setup**:

```csharp
// Optimal lighting for robotics visualization
// 1. Directional Light (sun)
//    - Intensity: 1.0
//    - Shadow Type: Soft Shadows
//    - Color: Warm white (255, 244, 214)

// 2. Ambient lighting
//    Window > Rendering > Lighting
//    Environment > Source: Skybox
//    Ambient Intensity: 1.0

// 3. Reflection Probes (for realistic reflections)
//    GameObject > Light > Reflection Probe
```

### Using Asset Store

Unity's Asset Store has thousands of free and paid assets:

1. Open Asset Store (Window > Asset Store)
2. Search for:
   - "Office environment"
   - "Warehouse"
   - "Laboratory"
   - "Industrial robot"
3. Download and import into project

## Simulating Human-Robot Interaction

### Adding Humanoid Characters

**Method 1: Unity's Character Controller**

```csharp
// SimpleHumanController.cs
using UnityEngine;

public class SimpleHumanController : MonoBehaviour
{
    public float moveSpeed = 2.0f;
    public Transform target; // Robot to interact with

    private Animator animator;
    private CharacterController controller;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Move toward robot
        if (target != null)
        {
            Vector3 direction = (target.position - transform.position).normalized;

            // Stop when close enough
            if (Vector3.Distance(transform.position, target.position) > 1.5f)
            {
                controller.Move(direction * moveSpeed * Time.deltaTime);
                animator.SetBool("Walking", true);
            }
            else
            {
                animator.SetBool("Walking", false);
                animator.SetTrigger("Wave"); // Interact with robot
            }
        }
    }
}
```

**Method 2: Using Mixamo Characters**

1. Go to https://www.mixamo.com (free with Adobe account)
2. Select a character
3. Choose animations (walk, wave, pick up, etc.)
4. Download with settings:
   - Format: FBX for Unity
   - Frames per second: 30
5. Import into Unity
6. Use Animator Controller to manage animations

### Interaction Scenarios

**Scenario 1: Handoff Task**

```csharp
// HandoffScenario.cs
using UnityEngine;

public class HandoffScenario : MonoBehaviour
{
    public Transform human;
    public Transform robot;
    public GameObject objectToHandoff;

    private enum State { Approach, Reach, Handoff, Retract }
    private State currentState = State.Approach;

    void Update()
    {
        switch (currentState)
        {
            case State.Approach:
                // Human walks toward robot
                MoveHumanToward(robot.position);
                if (Vector3.Distance(human.position, robot.position) < 1.0f)
                {
                    currentState = State.Reach;
                }
                break;

            case State.Reach:
                // Extend arm with object
                // (Animate arm extension)
                currentState = State.Handoff;
                break;

            case State.Handoff:
                // Transfer object to robot
                objectToHandoff.transform.SetParent(robot);
                currentState = State.Retract;
                break;

            case State.Retract:
                // Both move away
                break;
        }
    }

    void MoveHumanToward(Vector3 target)
    {
        Vector3 direction = (target - human.position).normalized;
        human.position += direction * 1.0f * Time.deltaTime;
    }
}
```

## Publishing Data to ROS 2

### Camera Images

```csharp
// CameraPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera camera;
    public string topicName = "/camera/image_raw";
    public float publishRate = 10.0f;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Create render texture
        renderTexture = new RenderTexture(640, 480, 24);
        camera.targetTexture = renderTexture;
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1.0f / publishRate)
        {
            PublishImage();
            timer = 0;
        }
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                }
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            step = 640 * 3,
            data = texture2D.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

### Robot Joint States

```csharp
// JointStatePublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
{
    public ArticulationBody[] joints;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("/joint_states");
    }

    void FixedUpdate()
    {
        PublishJointStates();
    }

    void PublishJointStates()
    {
        JointStateMsg msg = new JointStateMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                }
            },
            name = new string[joints.Length],
            position = new double[joints.Length],
            velocity = new double[joints.Length],
            effort = new double[joints.Length]
        };

        for (int i = 0; i < joints.Length; i++)
        {
            msg.name[i] = joints[i].name;
            msg.position[i] = joints[i].jointPosition[0] * Mathf.Deg2Rad;
            msg.velocity[i] = joints[i].jointVelocity[0] * Mathf.Deg2Rad;
            msg.effort[i] = joints[i].jointForce[0];
        }

        ros.Publish("/joint_states", msg);
    }
}
```

## Subscribing to ROS 2 Commands

### Receiving Velocity Commands

```csharp
// VelocitySubscriber.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocitySubscriber : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    public float speed = 1.0f;

    private ROSConnection ros;
    private Vector3 linearVelocity;
    private float angularVelocity;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ReceiveVelocity);
    }

    void ReceiveVelocity(TwistMsg msg)
    {
        linearVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.z,
            (float)msg.linear.y
        );
        angularVelocity = (float)msg.angular.z;
    }

    void Update()
    {
        // Apply velocity to robot
        transform.Translate(linearVelocity * speed * Time.deltaTime);
        transform.Rotate(0, angularVelocity * Mathf.Rad2Deg * Time.deltaTime, 0);
    }
}
```

## Advanced Rendering Techniques

### Post-Processing for Realism

1. Install Post Processing package (Package Manager)
2. Create Post-Process Volume:
   ```
   GameObject > Volume > Global Volume
   ```
3. Add effects:
   - Bloom (glowing lights)
   - Ambient Occlusion (contact shadows)
   - Color Grading (adjust colors)
   - Depth of Field (focus effects)

### Dynamic Lighting

```csharp
// DynamicLighting.cs
using UnityEngine;

public class DynamicLighting : MonoBehaviour
{
    public Light directionalLight;
    public float dayLength = 120.0f; // 2-minute day/night cycle

    void Update()
    {
        // Rotate light to simulate sun movement
        float angle = (Time.time / dayLength) * 360.0f;
        directionalLight.transform.rotation = Quaternion.Euler(angle - 90, 0, 0);

        // Adjust intensity based on time of day
        float intensity = Mathf.Max(0, Mathf.Cos(angle * Mathf.Deg2Rad));
        directionalLight.intensity = intensity;
    }
}
```

## Performance Optimization

### Reducing Draw Calls

1. **Batch Static Objects**:
   - Select static objects
   - Check "Static" in Inspector
   - Unity will batch them automatically

2. **Use Occlusion Culling**:
   ```
   Window > Rendering > Occlusion Culling
   Bake the scene
   ```

3. **Level of Detail (LOD)**:
   ```csharp
   // Add LOD Group component
   // Assign different models for different distances:
   // LOD 0: High-poly model (0-50m)
   // LOD 1: Medium-poly model (50-100m)
   // LOD 2: Low-poly model (100m+)
   ```

### Frame Rate Management

```csharp
// FrameRateManager.cs
using UnityEngine;

public class FrameRateManager : MonoBehaviour
{
    public int targetFrameRate = 60;

    void Start()
    {
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0; // Disable VSync for consistent timing
    }
}
```

## Best Practices for Unity Robotics

1. **Use Consistent Units**: Unity uses meters by default—match ROS conventions
2. **Layer Management**: Organize objects into layers (Robot, Environment, UI)
3. **Prefab Workflow**: Create reusable prefabs for robots, sensors, obstacles
4. **Version Control**: Use Git with .gitignore configured for Unity
5. **Testing**: Build small test scenes before creating large environments
6. **Documentation**: Comment C# scripts explaining robotics-specific behavior

## Summary

In this chapter, you learned:

- How to set up Unity for robotics simulation
- How to integrate Unity with ROS 2 using TCP connector
- How to import and control robot models in Unity
- How to create realistic environments and lighting
- How to simulate human-robot interactions
- How to publish sensor data and subscribe to commands
- Performance optimization techniques

Unity's strengths in rendering make it ideal for perception training, human-robot interaction studies, and creating compelling demonstrations. Combined with Gazebo's physics accuracy (Chapter 1), you have powerful tools for comprehensive robot testing.

## Further Reading

- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Unity Learn: https://learn.unity.com/
- ROS-TCP Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Juliani, A., et al. (2018). Unity: A general platform for intelligent agents. *arXiv preprint arXiv:1809.02627*.

---

**Learning Check**:
- ✓ Set up Unity with ROS 2 integration
- ✓ Import and control robot models
- ✓ Create realistic environments
- ✓ Simulate human-robot interactions
- ✓ Publish and subscribe to ROS 2 topics
- ✓ Optimize Unity scenes for performance
