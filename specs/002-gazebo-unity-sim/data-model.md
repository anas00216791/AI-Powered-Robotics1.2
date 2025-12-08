# Data Model: Module 2: The Digital Twin (Gazebo & Unity)

**Date**: 2025-12-07
**Feature**: 002-gazebo-unity-sim
**Purpose**: Define entities, relationships, and validation rules for simulation environments and RAG chatbot

---

## 1. Simulation Entities

### 1.1 Gazebo World

**Description**: Represents a complete Gazebo simulation environment including physics, models, and sensors.

**Attributes**:
- `world_name` (string, required): Unique identifier for the world (e.g., "basic_physics_demo")
- `physics_engine` (enum, required): Physics engine type
  - Values: `"ode"`, `"bullet"`, `"simbody"`, `"dart"`
  - Default: `"ode"`
- `gravity` (vector3, required): Gravity vector [x, y, z] in m/s²
  - Default: `[0, 0, -9.81]`
  - Validation: Must be a valid 3D vector
- `time_step` (float, required): Simulation time step in seconds
  - Default: `0.001`
  - Validation: `0.0001 <= time_step <= 0.01`
- `real_time_factor` (float, optional): Target real-time speed multiplier
  - Default: `1.0`
  - Validation: `real_time_factor > 0`
- `models` (array of Model, optional): List of models in the world
- `plugins` (array of Plugin, optional): Gazebo plugins for custom behavior

**Relationships**:
- **Contains** → `Model[]` (one-to-many)
- **Contains** → `Plugin[]` (one-to-many)

**State Transitions**:
```
[Created] → [Loaded] → [Running] → [Paused] ↔ [Running] → [Stopped]
```

**Validation Rules**:
- World file must be valid SDF (Simulation Description Format) XML
- All referenced model URIs must be resolvable
- Physics engine must be available in Gazebo installation

**File Format Example**:
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="basic_physics_demo">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- models, lights, etc. -->
  </world>
</sdf>
```

---

### 1.2 Unity Scene

**Description**: Represents a Unity scene for high-fidelity rendering and human-robot interaction.

**Attributes**:
- `scene_name` (string, required): Unity scene name (e.g., "HRI_Laboratory")
- `game_objects` (array of GameObject, required): All objects in the scene
- `lighting_mode` (enum, required): Lighting calculation mode
  - Values: `"realtime"`, `"baked"`, `"mixed"`
  - Default: `"mixed"`
- `skybox` (string, optional): Skybox material name
- `physics_settings` (PhysicsSettings, required): Unity physics configuration
  - `gravity` (vector3): Default `[0, -9.81, 0]`
  - `fixed_timestep` (float): Default `0.02`
- `camera_settings` (array of Camera, required): Scene cameras

**Relationships**:
- **Contains** → `GameObject[]` (one-to-many)
- **Contains** → `Camera[]` (one-to-many)
- **References** → `Material[]` (many-to-many via GameObjects)

**State Transitions**:
```
[Editing] → [Play Mode] → [Paused] ↔ [Play Mode] → [Editing]
```

**Validation Rules**:
- Scene file must be valid Unity YAML
- All referenced prefabs and materials must exist in project
- At least one active camera must be present

---

### 1.3 Robot Model

**Description**: Represents a robot in simulation (URDF-based, shared between Gazebo and Unity).

**Attributes**:
- `model_name` (string, required): Robot identifier (e.g., "mobile_manipulator")
- `urdf_path` (string, required): Path to URDF file
- `base_link` (string, required): Name of the root link
- `links` (array of Link, required): Robot links (rigid bodies)
- `joints` (array of Joint, required): Robot joints
- `sensors` (array of Sensor, optional): Attached sensors
- `mass_total` (float, derived): Total mass in kg (computed from links)

**Relationships**:
- **Composed of** → `Link[]` (one-to-many)
- **Composed of** → `Joint[]` (one-to-many)
- **Has** → `Sensor[]` (one-to-many)

**Validation Rules**:
- URDF must pass `check_urdf` validation
- Joint parent-child relationships must form a valid tree (no cycles)
- All link masses must be positive
- All inertia tensors must be positive definite

---

### 1.4 Sensor

**Description**: Simulated sensor attached to a robot or environment. Subtypes: LiDAR, Depth Camera, IMU.

**Common Attributes**:
- `sensor_id` (string, required): Unique identifier
- `sensor_type` (enum, required): Type of sensor
  - Values: `"lidar"`, `"depth_camera"`, `"imu"`, `"camera"`
- `parent_link` (string, required): Link to which sensor is attached
- `pose` (Pose6D, required): Position and orientation relative to parent link
  - `position` (vector3): [x, y, z] in meters
  - `orientation` (quaternion): [x, y, z, w]
- `update_rate` (float, required): Sensor update frequency in Hz
  - Validation: `0.1 <= update_rate <= 1000`
- `noise_model` (NoiseModel, optional): Sensor noise configuration

---

#### 1.4.1 LiDAR Sensor (extends Sensor)

**Additional Attributes**:
- `min_range` (float, required): Minimum detection range in meters
  - Validation: `min_range > 0`
- `max_range` (float, required): Maximum detection range in meters
  - Validation: `max_range > min_range`
- `horizontal_fov` (float, required): Horizontal field of view in radians
  - Validation: `0 < horizontal_fov <= 2π`
- `vertical_fov` (float, required): Vertical field of view in radians
- `horizontal_samples` (int, required): Number of horizontal scan points
  - Validation: `horizontal_samples >= 1`
- `vertical_samples` (int, required): Number of vertical scan layers
  - Validation: `vertical_samples >= 1`
- `output_format` (enum, required): Point cloud format
  - Values: `"xyz"`, `"xyzrgb"`, `"xyzi"` (intensity)

**Output Data**:
```
PointCloud {
  header: { timestamp, frame_id }
  points: Array<{ x, y, z, [intensity] }>
}
```

**Real-World Reference**: Velodyne VLP-16 (16 layers, 360° horizontal, ±15° vertical, 100m range)

---

#### 1.4.2 Depth Camera (extends Sensor)

**Additional Attributes**:
- `resolution` (vector2int, required): Image resolution [width, height] in pixels
  - Common values: `[640, 480]`, `[1280, 720]`
- `horizontal_fov` (float, required): Horizontal field of view in radians
  - Validation: `0 < horizontal_fov < π`
- `clip_near` (float, required): Near clipping plane in meters
  - Validation: `clip_near > 0`
- `clip_far` (float, required): Far clipping plane in meters
  - Validation: `clip_far > clip_near`
- `depth_format` (enum, required): Depth encoding
  - Values: `"float32"`, `"uint16_mm"`

**Output Data**:
```
DepthImage {
  header: { timestamp, frame_id }
  rgb_image: Image<uint8>[height][width][3]
  depth_image: Array<float32>[height][width]  # in meters
}
```

**Real-World Reference**: Intel RealSense D435 (1280×720, 87° H FOV, 0.3-10m range)

---

#### 1.4.3 IMU Sensor (extends Sensor)

**Additional Attributes**:
- `accelerometer_range` (float, required): Max acceleration in m/s²
  - Common values: `±20`, `±40`, `±80` m/s²
- `gyroscope_range` (float, required): Max angular velocity in rad/s
  - Common values: `±8.7`, `±17.5` rad/s (±500°/s, ±1000°/s)
- `gravity_compensation` (bool, required): Whether to remove gravity from accelerometer
  - Default: `false`

**Output Data**:
```
IMUData {
  header: { timestamp, frame_id }
  linear_acceleration: { x, y, z }  # m/s²
  angular_velocity: { x, y, z }     # rad/s
  orientation: { x, y, z, w }       # quaternion (if available)
}
```

**Real-World Reference**: MPU-6050 (±2g to ±16g accel, ±250°/s to ±2000°/s gyro)

---

### 1.5 Noise Model

**Description**: Statistical noise model applied to sensor readings.

**Attributes**:
- `noise_type` (enum, required): Type of noise distribution
  - Values: `"none"`, `"gaussian"`, `"uniform"`
- `mean` (float, required for gaussian): Mean of the noise distribution
  - Default: `0.0`
- `stddev` (float, required for gaussian): Standard deviation
  - Validation: `stddev >= 0`
- `min` (float, required for uniform): Minimum value
- `max` (float, required for uniform): Maximum value

**Tiers** (from research.md):
- **Tier 1 (Simplified)**: `noise_type = "none"`
- **Tier 2 (Basic Noise)**: `noise_type = "gaussian"`, realistic stddev
- **Tier 3 (Advanced)**: Multiple noise components (bias, drift, etc.)

---

## 2. RAG Chatbot Entities

### 2.1 Query

**Description**: User query submitted to the RAG chatbot.

**Attributes**:
- `id` (UUID, primary key): Unique query identifier
- `query_text` (text, required): User's natural language question
  - Validation: `1 <= length <= 1000` characters
- `timestamp` (timestamp, required): When the query was submitted
  - Default: `NOW()`
- `response_text` (text, nullable): Generated answer
- `response_time_ms` (integer, nullable): Time taken to generate response
  - Validation: `response_time_ms > 0`
- `user_session_id` (string, nullable): Anonymous session identifier

**Relationships**:
- **Has** → `Retrieval[]` (one-to-many)
- **Has** → `Feedback` (one-to-one, optional)

**State Transitions**:
```
[Submitted] → [Processing] → [Answered] → [Feedback Received]
```

---

### 2.2 Retrieval

**Description**: A document chunk retrieved from Qdrant for a specific query.

**Attributes**:
- `id` (UUID, primary key): Unique retrieval identifier
- `query_id` (UUID, foreign key): Associated query
- `chunk_id` (string, required): Qdrant point ID
- `chunk_text` (text, required): Retrieved text content
- `relevance_score` (float, required): Cosine similarity score
  - Validation: `-1.0 <= relevance_score <= 1.0`
- `source_document` (string, required): Source file path
  - Example: `"module-02/chapter-01-gazebo-physics.md"`

**Relationships**:
- **Belongs to** → `Query` (many-to-one)

---

### 2.3 Feedback

**Description**: User feedback on chatbot answer quality.

**Attributes**:
- `id` (UUID, primary key): Unique feedback identifier
- `query_id` (UUID, foreign key): Associated query
- `is_helpful` (boolean, required): Thumbs up/down
- `feedback_text` (text, nullable): Optional detailed feedback
  - Validation: `0 <= length <= 500` characters
- `timestamp` (timestamp, required): When feedback was submitted
  - Default: `NOW()`

**Relationships**:
- **Belongs to** → `Query` (one-to-one)

---

### 2.4 Document Chunk

**Description**: Embedded text chunk stored in Qdrant vector database (no SQL table, metadata only).

**Metadata** (stored in Qdrant):
- `chunk_id` (string, unique): Identifier (e.g., `"module-02-ch01-sect3"`)
- `source_document` (string): Source markdown file path
- `chunk_text` (text): Actual text content
- `embedding` (vector[768]): Dense vector embedding (e.g., from OpenAI `text-embedding-ada-002` or open-source model)
- `module_id` (string): Module identifier (e.g., `"002-gazebo-unity-sim"`)
- `chapter_id` (string): Chapter identifier (e.g., `"chapter-01"`)
- `section_title` (string): Section heading

**Chunking Strategy**:
- **Method**: Recursive character splitting with overlap
- **Chunk Size**: 512-1024 tokens (configurable)
- **Overlap**: 128 tokens to preserve context across boundaries
- **Metadata Preservation**: Retain markdown headers in chunk metadata

---

## 3. Educational Content Entities

### 3.1 Module

**Description**: A complete educational module (this is Module 2).

**Attributes**:
- `module_id` (string, required): Identifier (e.g., `"002-gazebo-unity-sim"`)
- `module_title` (string, required): Full title
- `word_count` (int, derived): Total word count
  - Validation: `3000 <= word_count <= 5000` (per spec)
- `chapters` (array of Chapter, required): Ordered list of chapters
- `citations` (array of Citation, required): All cited sources
  - Validation: `length(citations) >= 10`, `peer_reviewed_ratio >= 0.5`

**Relationships**:
- **Contains** → `Chapter[]` (one-to-many, ordered)
- **Cites** → `Citation[]` (one-to-many)

---

### 3.2 Chapter

**Description**: A single chapter within a module.

**Attributes**:
- `chapter_id` (string, required): Identifier (e.g., `"chapter-01"`)
- `chapter_number` (int, required): Sequence number (1, 2, 3)
- `title` (string, required): Chapter title
- `file_path` (string, required): Markdown file path
- `sections` (array of Section, required): Chapter sections
- `code_examples` (array of CodeExample, optional): Executable code snippets
- `simulation_examples` (array of SimulationExample, optional): Simulation assets

**Relationships**:
- **Belongs to** → `Module` (many-to-one)
- **Contains** → `Section[]` (one-to-many, ordered)
- **Contains** → `CodeExample[]` (one-to-many)
- **Contains** → `SimulationExample[]` (one-to-many)

---

### 3.3 Citation

**Description**: A cited source in APA format.

**Attributes**:
- `citation_id` (string, required): Unique identifier (e.g., `"cite_gazebo_docs_2024"`)
- `citation_text` (string, required): Full APA-formatted citation
- `source_type` (enum, required): Type of source
  - Values: `"peer_reviewed"`, `"official_docs"`, `"book"`, `"whitepaper"`, `"blog"`
- `url` (string, optional): Link to the source (if available)
- `is_primary_source` (bool, derived): True if `source_type` in [`"peer_reviewed"`, `"official_docs"`]

**Validation Rules**:
- Citation must follow APA 7th edition format
- At least 50% of all citations must have `is_primary_source = true`

---

### 3.4 Code Example

**Description**: An executable code snippet or configuration file.

**Attributes**:
- `example_id` (string, required): Identifier (e.g., `"gazebo_world_basic"`)
- `title` (string, required): Descriptive title
- `language` (string, required): Programming/markup language (e.g., `"xml"`, `"python"`)
- `code` (text, required): Full code content
- `file_path` (string, optional): Path if distributed as file
- `dependencies` (array of string, optional): Required packages/versions
- `expected_output` (text, optional): Description of expected result
- `is_reproducible` (bool, required): Whether example has been validated
  - Default: `false`

**Validation Rules**:
- All code examples must have `is_reproducible = true` before module publication
- Dependencies must specify exact versions (e.g., `"gazebo==11.0"`, not `"gazebo"`)

---

### 3.5 Simulation Example

**Description**: A complete simulation scenario with assets.

**Attributes**:
- `example_id` (string, required): Identifier (e.g., `"lidar_noise_demo"`)
- `title` (string, required): Descriptive title
- `platform` (enum, required): Simulation platform
  - Values: `"gazebo"`, `"unity"`
- `asset_files` (array of string, required): List of required files
  - Gazebo: `.world`, `.sdf`, `.urdf` files
  - Unity: scene files, prefabs, scripts
- `instructions` (text, required): Step-by-step setup and execution instructions
- `expected_behavior` (text, required): Description of correct simulation output
- `validation_status` (enum, required): Testing status
  - Values: `"not_tested"`, `"passed"`, `"failed"`
  - Default: `"not_tested"`

**Validation Rules**:
- All simulation examples must have `validation_status = "passed"` before publication
- Asset files must be included in module delivery

---

## 4. Entity Relationship Diagram

```
[Module] 1 --→ * [Chapter] 1 --→ * [Section]
   |                 |
   |                 +--→ * [CodeExample]
   |                 +--→ * [SimulationExample]
   |
   +--→ * [Citation]

[RobotModel] 1 --→ * [Link]
             1 --→ * [Joint]
             1 --→ * [Sensor]

[Sensor] ← (subclass) -- [LiDAR]
         ← (subclass) -- [DepthCamera]
         ← (subclass) -- [IMU]

[Sensor] 1 --→ 0..1 [NoiseModel]

[GazeboWorld] 1 --→ * [RobotModel]
              1 --→ * [Plugin]

[UnityScene] 1 --→ * [GameObject]
             1 --→ * [Camera]

[Query] 1 --→ * [Retrieval]
        1 --→ 0..1 [Feedback]

[DocumentChunk] (Qdrant, referenced by [Retrieval])
```

---

## 5. Validation Summary

| Entity | Critical Validations |
|--------|---------------------|
| **GazeboWorld** | Valid SDF XML, resolvable model URIs, physics engine available |
| **UnityScene** | Valid Unity YAML, all assets exist, at least one camera |
| **RobotModel** | Valid URDF, tree structure (no cycles), positive masses, positive-definite inertias |
| **LiDAR** | `max_range > min_range`, `horizontal_samples >= 1` |
| **DepthCamera** | `clip_far > clip_near`, `horizontal_fov < π` |
| **IMU** | `accelerometer_range > 0`, `gyroscope_range > 0` |
| **Module** | `3000 <= word_count <= 5000`, `≥10 citations`, `≥50% primary sources` |
| **CodeExample** | `is_reproducible = true`, exact dependency versions |
| **SimulationExample** | `validation_status = "passed"`, all asset files included |
| **Query** | `1 <= length(query_text) <= 1000` |
| **Citation** | Valid APA format, `is_primary_source` ratio ≥ 0.5 |

---

## 6. Data Lifecycle

### Simulation Data Lifecycle
```
[Author Creates] → [Validation] → [Integration Testing] → [Publication] → [Student Use]
```

### RAG Chatbot Data Lifecycle
```
[Content Authored] → [Chunking] → [Embedding] → [Qdrant Storage] → [Query/Retrieval] → [Feedback Collection] → [Model Improvement]
```

### Quality Assurance Checkpoints
1. **Pre-Phase 1**: Constitution Check (principles alignment)
2. **Post-Phase 1**: Data model validation (this document)
3. **Pre-Publication**: Reproducibility testing (all examples pass)
4. **Post-Publication**: RAG accuracy validation (chatbot answers match content)

---

**Last Updated**: 2025-12-07
**Next Steps**: Generate API contracts for RAG chatbot (Phase 1 continuation)
