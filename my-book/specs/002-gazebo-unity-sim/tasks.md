# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-gazebo-unity-sim/`
**Prerequisites**: plan.md (✓), spec.md (✓), research.md (✓), data-model.md (✓), contracts/ (✓), quickstart.md (✓)

**Tests**: This is an educational content project. Testing focuses on reproducibility validation (simulation examples work) and content quality validation (word count, citations, readability).

**Organization**: Tasks are grouped by user story (chapters). Each chapter can be developed independently after foundational setup.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files/chapters, no dependencies)
- **[Story]**: Which user story/chapter this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

Per plan.md, this project uses:
- **Module content**: `docs/module-02-digital-twin/` (Markdown chapters)
- **Simulation assets**: `simulations/gazebo/`, `simulations/unity/`
- **Chatbot backend**: `chatbot/src/` (FastAPI, Python)
- **Docusaurus site**: `docusaurus/src/components/` (React/TypeScript)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure

- [ ] T001 Create module directory structure in docs/module-02-digital-twin/ with subdirectories (assets/gazebo-worlds, assets/unity-scenes, assets/diagrams)
- [ ] T002 Create simulation testing directories (simulations/gazebo/worlds, simulations/gazebo/models, simulations/unity/scenes, simulations/unity/assets)
- [ ] T003 [P] Initialize Docusaurus project structure (if not exists) with module-02 navigation configuration in docusaurus/docusaurus.config.js
- [ ] T004 [P] Create module index page in docs/module-02-digital-twin/index.md with overview, learning objectives, and navigation links

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and research that MUST be complete before ANY chapter can be authored

**⚠️ CRITICAL**: No chapter authoring can begin until this phase is complete

- [ ] T005 Research and compile 15-20 primary sources (≥50% official docs or peer-reviewed papers) on Gazebo, Unity, and robotics simulation from gazebosim.org, docs.unity3d.com, Google Scholar
- [ ] T006 Create citations database/spreadsheet tracking source type (peer_reviewed/official_docs/other), APA citation text, and URL for all sources
- [ ] T007 Set up Gazebo Harmonic development environment (Ubuntu 22.04+ VM or native) and verify installation with gz sim --version
- [ ] T008 [P] Set up Unity 2022 LTS environment and install Unity Robotics Hub packages (URDF Importer, ROS-TCP-Connector) via Package Manager
- [ ] T009 [P] Download and prepare robot models from Module 1 (URDF files) for use in Gazebo and Unity examples
- [ ] T010 [P] Create APA citation templates and Markdown snippet for inline citations to ensure consistency

**Checkpoint**: Foundation ready - chapter authoring can now begin in parallel

---

## Phase 3: User Story 1 - Learning Physics Simulation in Gazebo (Priority: P1) 🎯 MVP

**Goal**: Students can understand physics simulation concepts and create realistic Gazebo environments with gravity, collisions, and basic objects.

**Independent Test**: A student can read Chapter 1 and successfully create a Gazebo world file with ground plane, obstacles, and custom gravity settings, then explain how physics parameters affect robot behavior.

### Implementation for User Story 1 (Chapter 1: Gazebo Physics)

- [ ] T011 [US1] Create chapter-01-gazebo-physics.md outline with sections: Introduction, Physics Engines Overview, Gazebo World Files, Gravity and Time Step Configuration, Collision Properties, Robot Import, Physics Interactions, Gazebo Harmonic vs Classic 11
- [ ] T012 [US1] Write Introduction section (300-400 words) explaining role of physics engines in robotics simulation, cite official Gazebo documentation
- [ ] T013 [US1] Write Physics Engines Overview section (400-500 words) covering ODE, Bullet, Simbody, DART with comparison table, cite peer-reviewed simulation papers
- [ ] T014 [P] [US1] Create Example 1: basic_physics_demo.world in simulations/gazebo/worlds/ with ground plane, gravity [0,0,-9.81], and time step 0.001
- [ ] T015 [US1] Test Example 1 by running gz sim basic_physics_demo.world (Harmonic) and gazebo basic_physics_demo.world (Classic 11), verify both launch without errors
- [ ] T016 [US1] Write Gazebo World Files section (500-600 words) explaining SDF format, structure, and basic tags with inline Example 1 code and screenshots
- [ ] T017 [P] [US1] Create Example 2: custom_gravity.world in simulations/gazebo/worlds/ demonstrating non-standard gravity (e.g., Mars gravity [0,0,-3.7])
- [ ] T018 [US1] Write Gravity and Time Step Configuration section (400-500 words) explaining physics parameters, performance tradeoffs, with Example 2 inline
- [ ] T019 [P] [US1] Create Example 3: collision_demo.world in simulations/gazebo/worlds/ with obstacles (boxes, spheres) and collision properties (friction, restitution)
- [ ] T020 [US1] Test Example 3 by adding a simple robot model, observing collisions, and capturing screenshot for documentation
- [ ] T021 [US1] Write Collision Properties section (400-500 words) covering collision geometries, surface friction, restitution coefficients, with Example 3 inline
- [ ] T022 [US1] Write Robot Import section (300-400 words) explaining how to import URDF models from Module 1 into Gazebo worlds, include step-by-step instructions
- [ ] T023 [P] [US1] Create Example 4: robot_physics_test.world in simulations/gazebo/worlds/ importing a robot from Module 1 and demonstrating physics interactions
- [ ] T024 [US1] Write Physics Interactions section (300-400 words) explaining observation techniques, debugging physics issues, with Example 4 inline
- [ ] T025 [US1] Write Gazebo Harmonic vs Classic 11 comparison section (200-300 words) highlighting API differences, migration considerations, when to use each
- [ ] T026 [US1] Write Summary section (150-200 words) recapping key concepts and linking forward to Chapter 2
- [ ] T027 [US1] Add all APA-formatted citations (≥5 sources, ≥50% official docs or peer-reviewed) to References section at end of chapter-01-gazebo-physics.md
- [ ] T028 [US1] Copy validated simulation files from simulations/gazebo/worlds/ to docs/module-02-digital-twin/assets/gazebo-worlds/ for student distribution
- [ ] T029 [P] [US1] Create 2-3 diagrams (physics force diagrams, SDF structure diagram) in docs/module-02-digital-twin/assets/diagrams/ using draw.io or similar tool

### Validation for User Story 1

- [ ] T030 [US1] Validate Chapter 1 word count is 2500-3500 words using wc -w docs/module-02-digital-twin/chapter-01-gazebo-physics.md
- [ ] T031 [US1] Validate all 4 Gazebo examples launch without errors in both Harmonic and Classic 11 (mark validation_status="passed" in tracking)
- [ ] T032 [US1] Validate Flesch-Kincaid readability grade is 10-12 using online readability calculator or textstat Python library
- [ ] T033 [US1] Validate all citations are in correct APA format and ≥50% are official docs or peer-reviewed papers
- [ ] T034 [US1] Peer review: Have another person follow Chapter 1 instructions and successfully create a Gazebo world (acceptance scenario validation)

**Checkpoint**: At this point, User Story 1 (Chapter 1) should be complete, validated, and independently usable by students

---

## Phase 4: User Story 2 - High-Fidelity Rendering in Unity (Priority: P2)

**Goal**: Students can create high-fidelity visualizations and human-robot interaction scenarios in Unity with realistic lighting, materials, and human avatars.

**Independent Test**: A student can follow Chapter 2 examples to set up a Unity scene with a robot model, realistic lighting, materials, and human avatars for interaction testing.

### Implementation for User Story 2 (Chapter 2: Unity Rendering)

- [ ] T035 [P] [US2] Create chapter-02-unity-rendering.md outline with sections: Introduction, Unity for Robotics Visualization, Scene Creation, Robot Model Import, Materials and Lighting, Human Avatars and HRI, Unity vs Gazebo Tradeoffs
- [ ] T036 [US2] Write Introduction section (300-400 words) explaining role of high-fidelity rendering in robotics, perception testing, and HRI research, cite Unity and robotics visualization papers
- [ ] T037 [US2] Write Unity for Robotics Visualization section (400-500 words) introducing Unity Editor, game objects, components, and Unity Robotics Hub, cite official Unity documentation
- [ ] T038 [P] [US2] Create Example 1: Basic Unity scene in simulations/unity/scenes/ with environment (floor, walls, lighting) and save as "HRI_Laboratory.unity"
- [ ] T039 [US2] Test Example 1 by opening Unity 2022 LTS, loading scene, entering Play Mode, and capturing screenshot
- [ ] T040 [US2] Write Scene Creation section (400-500 words) explaining Unity scene hierarchy, lighting modes (realtime/baked/mixed), with Example 1 inline and screenshots
- [ ] T041 [P] [US2] Create Example 2: Import robot URDF from Module 1 into Unity using URDF Importer package, apply materials and textures
- [ ] T042 [US2] Test Example 2 by verifying robot model renders correctly with materials, joints are articulated, and capturing screenshot
- [ ] T043 [US2] Write Robot Model Import section (500-600 words) with step-by-step URDF import instructions, troubleshooting tips, material application, with Example 2 inline
- [ ] T044 [P] [US2] Create Example 3: Enhanced lighting scene with directional light (sun), point lights, and realistic shadows in "HRI_Laboratory.unity"
- [ ] T045 [US2] Write Materials and Lighting section (500-600 words) covering materials, shaders (Standard/URP), lighting techniques, shadow quality, with Example 3 inline
- [ ] T046 [P] [US2] Download Mixamo character (e.g., "Jasper" or "Olivia") with animations (T-pose, Walk, Wave) and import into Unity scene
- [ ] T047 [US2] Create Example 4: HRI scenario with human avatar and robot performing basic interaction (wave gesture, handoff simulation) in Unity scene
- [ ] T048 [US2] Test Example 4 by running HRI scenario in Play Mode, verifying animations play correctly, and recording video/screenshots
- [ ] T049 [US2] Write Human Avatars and HRI section (500-600 words) with Mixamo download instructions, avatar setup, animation controller basics, HRI scenario examples, with Example 4 inline
- [ ] T050 [US2] Write Unity vs Gazebo Tradeoffs section (400-500 words) comparing physics accuracy, rendering quality, ROS integration, use cases (Gazebo for physics-heavy, Unity for perception/HRI)
- [ ] T051 [US2] Write Summary section (150-200 words) recapping Unity rendering concepts and linking forward to Chapter 3
- [ ] T052 [US2] Add all APA-formatted citations (≥5 sources, ≥50% official Unity docs or peer-reviewed HRI papers) to References section at end of chapter-02-unity-rendering.md
- [ ] T053 [US2] Export Unity scene screenshots and save to docs/module-02-digital-twin/assets/unity-scenes/ for documentation

### Validation for User Story 2

- [ ] T054 [US2] Validate Chapter 2 word count is 2500-3500 words using wc -w docs/module-02-digital-twin/chapter-02-unity-rendering.md
- [ ] T055 [US2] Validate all 4 Unity examples load and run without errors in Unity 2022 LTS Play Mode (mark validation_status="passed")
- [ ] T056 [US2] Validate Flesch-Kincaid readability grade is 10-12
- [ ] T057 [US2] Validate all citations are in correct APA format and ≥50% are official docs or peer-reviewed papers
- [ ] T058 [US2] Peer review: Have another person follow Chapter 2 instructions and successfully create a Unity HRI scene (acceptance scenario validation)

**Checkpoint**: At this point, User Story 2 (Chapter 2) should be complete, validated, and independently usable by students

---

## Phase 5: User Story 3 - Sensor Simulation and Integration (Priority: P3)

**Goal**: Students can simulate and integrate realistic sensors (LiDAR, depth cameras, IMUs) in both Gazebo and Unity, visualize sensor outputs, and verify data accuracy.

**Independent Test**: A student can use Chapter 3 instructions to add LiDAR, depth camera, and IMU sensors to a simulated robot, visualize sensor outputs, and verify data accuracy.

### Implementation for User Story 3 (Chapter 3: Sensor Simulation)

- [ ] T059 [P] [US3] Create chapter-03-sensor-simulation.md outline with sections: Introduction, Sensor Simulation Overview, LiDAR Simulation (Gazebo), LiDAR Simulation (Unity), Depth Camera Simulation (Gazebo), Depth Camera Simulation (Unity), IMU Simulation (Gazebo), IMU Simulation (Unity), Sensor Noise Models, Sensor Validation
- [ ] T060 [US3] Write Introduction section (300-400 words) explaining importance of sensor simulation for perception algorithms, cite robotics sensor simulation papers
- [ ] T061 [US3] Write Sensor Simulation Overview section (300-400 words) introducing sensor types, output formats, noise models, and three-tier approach (simple → basic noise → advanced)

#### LiDAR Sensor Tasks

- [ ] T062 [P] [US3] Create Example 1 (Tier 1): Gazebo LiDAR sensor plugin in simulations/gazebo/worlds/lidar_simple.world with no noise, 360° horizontal FOV, 100m range
- [ ] T063 [US3] Test Example 1 by launching Gazebo, visualizing point cloud in RViz or Gazebo, verifying scan range and resolution
- [ ] T064 [US3] Write LiDAR Simulation (Gazebo) Tier 1 section (400-500 words) explaining sensor plugin configuration, SDF tags (<sensor type="gpu_lidar">), output topic, with Example 1 inline
- [ ] T065 [P] [US3] Create Example 2 (Tier 2): Gazebo LiDAR with Gaussian noise (stddev=0.01) in simulations/gazebo/worlds/lidar_noise.world
- [ ] T066 [US3] Write LiDAR Simulation (Gazebo) Tier 2 section (300-400 words) explaining noise model, realistic parameters based on Velodyne VLP-16 specs, with Example 2 inline
- [ ] T067 [P] [US3] Create Example 3: Unity LiDAR sensor using Perception package or custom raycasting script, visualize point cloud
- [ ] T068 [US3] Test Example 3 by running Unity scene with LiDAR, capturing point cloud data, verifying against expected format
- [ ] T069 [US3] Write LiDAR Simulation (Unity) section (400-500 words) with Unity LiDAR setup instructions, point cloud visualization, noise addition, with Example 3 inline

#### Depth Camera Sensor Tasks

- [ ] T070 [P] [US3] Create Example 4 (Tier 1): Gazebo depth camera plugin in simulations/gazebo/worlds/depth_camera_simple.world with 640x480 resolution, 0.3-10m range
- [ ] T071 [US3] Test Example 4 by launching Gazebo, capturing RGB and depth images, verifying pixel-to-distance mapping
- [ ] T072 [US3] Write Depth Camera Simulation (Gazebo) Tier 1 section (400-500 words) explaining depth camera plugin, image topics, depth encoding, with Example 4 inline
- [ ] T073 [P] [US3] Create Example 5 (Tier 2): Gazebo depth camera with noise and distortion models based on Intel RealSense D435 specs
- [ ] T074 [US3] Write Depth Camera Simulation (Gazebo) Tier 2 section (300-400 words) explaining realistic noise parameters, with Example 5 inline
- [ ] T075 [P] [US3] Create Example 6: Unity depth camera using Perception package, capture depth images and validate format
- [ ] T076 [US3] Test Example 6 by running Unity scene, capturing depth images, comparing with Gazebo depth output
- [ ] T077 [US3] Write Depth Camera Simulation (Unity) section (400-500 words) with Unity depth camera setup, Perception package usage, with Example 6 inline

#### IMU Sensor Tasks

- [ ] T078 [P] [US3] Create Example 7 (Tier 1): Gazebo IMU plugin in simulations/gazebo/worlds/imu_simple.world with ±20 m/s² accel range, ±8.7 rad/s gyro range
- [ ] T079 [US3] Test Example 7 by moving robot in simulation, reading IMU data via ROS 2 topic, verifying acceleration and angular velocity match expected values
- [ ] T080 [US3] Write IMU Simulation (Gazebo) Tier 1 section (400-500 words) explaining IMU plugin, data format (linear_acceleration, angular_velocity, orientation), with Example 7 inline
- [ ] T081 [P] [US3] Create Example 8 (Tier 2): Gazebo IMU with Gaussian noise based on MPU-6050 specs (accel noise, gyro noise, bias)
- [ ] T082 [US3] Write IMU Simulation (Gazebo) Tier 2 section (300-400 words) explaining realistic IMU noise, drift, gravity compensation, with Example 8 inline
- [ ] T083 [P] [US3] Create Example 9: Unity IMU sensor script reading Rigidbody acceleration and angular velocity, add noise model
- [ ] T084 [US3] Test Example 9 by moving robot in Unity Play Mode, logging IMU data, verifying against expected physics behavior
- [ ] T085 [US3] Write IMU Simulation (Unity) section (400-500 words) with Unity IMU implementation, noise addition, with Example 9 inline

#### Sensor Noise and Validation

- [ ] T086 [US3] Write Sensor Noise Models section (400-500 words) explaining three-tier approach, Gaussian noise parameters, advanced noise (bias, drift, multi-path), when to use each tier
- [ ] T087 [US3] Write Sensor Validation section (300-400 words) explaining how to validate sensor accuracy against real-world specs (Velodyne, RealSense, MPU-6050), debugging techniques
- [ ] T088 [US3] Write Summary section (150-200 words) recapping sensor simulation across platforms, best practices, and next steps (link to Module 3 if available)
- [ ] T089 [US3] Add all APA-formatted citations (≥7 sources including sensor datasheets, peer-reviewed sensor simulation papers, official docs) to References section at end of chapter-03-sensor-simulation.md
- [ ] T090 [US3] Copy all validated sensor simulation files to docs/module-02-digital-twin/assets/gazebo-worlds/ and docs/module-02-digital-twin/assets/unity-scenes/ for distribution

### Validation for User Story 3

- [ ] T091 [US3] Validate Chapter 3 word count is 3000-4000 words using wc -w docs/module-02-digital-twin/chapter-03-sensor-simulation.md
- [ ] T092 [US3] Validate all 9 sensor examples (3 LiDAR, 3 depth camera, 3 IMU) launch and produce correct sensor data without errors (mark validation_status="passed")
- [ ] T093 [US3] Validate sensor outputs match expected formats and accuracy specifications (LiDAR point clouds, depth images, IMU data)
- [ ] T094 [US3] Validate Flesch-Kincaid readability grade is 10-12
- [ ] T095 [US3] Validate all citations are in correct APA format and ≥50% are official docs or peer-reviewed papers
- [ ] T096 [US3] Peer review: Have another person follow Chapter 3 instructions and successfully simulate all three sensor types (acceptance scenario validation)

**Checkpoint**: At this point, User Story 3 (Chapter 3) should be complete, validated, and independently usable by students

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Module-level integration, final validation, and optional RAG chatbot integration

### Module Integration

- [ ] T097 Update module index page (docs/module-02-digital-twin/index.md) with complete chapter summaries, learning objectives, prerequisites, and navigation links
- [ ] T098 Create module-level references file aggregating all citations from all three chapters, removing duplicates, ensuring consistent APA formatting
- [ ] T099 Validate total module word count is 3000-5000 words (excluding code blocks and citations) using wc -w on all three chapters combined
- [ ] T100 Validate total unique citations count is 10-15 sources with ≥50% official docs or peer-reviewed papers across entire module
- [ ] T101 [P] Create module completion checklist for students in index.md (e.g., "Can you create a Gazebo world?", "Can you simulate sensors?")
- [ ] T102 [P] Add troubleshooting section to index.md or appendix covering common Gazebo/Unity issues and solutions

### RAG Chatbot Integration (Optional, if chatbot already exists)

> **Note**: These tasks assume chatbot backend already built. If building from scratch, reference plan.md and contracts/chatbot-api.yaml.

- [ ] T103 [P] Chunk and embed all three chapters (chapter-01, chapter-02, chapter-03) into Qdrant vector database using chatbot/src/embeddings/ingest.py script
- [ ] T104 [P] Test chatbot queries related to Module 2 content (e.g., "How do I set up gravity in Gazebo?", "How do I add a LiDAR sensor?") and validate answers cite correct chapter sections
- [ ] T105 [P] Integrate ChatbotWidget React component into Docusaurus site (if not already global) with module-specific context (current_module: "module-02")

### Final Validation and Delivery

- [ ] T106 Run full module validation: All simulation examples reproducible, word count correct, citation ratio met, readability grade 10-12
- [ ] T107 Generate PDF export of module using Docusaurus PDF plugin or Pandoc for offline distribution
- [ ] T108 Commit all module content, simulation assets, and documentation to Git repository with descriptive commit message
- [ ] T109 [P] Create module release notes documenting what was delivered, known issues, and future improvements
- [ ] T110 [P] Update project README or main documentation to link to Module 2 and mark as complete

**Checkpoint**: Module 2 is complete, validated, and ready for student use 🎉

---

## Dependencies (User Story Completion Order)

```
Phase 1 (Setup) → Phase 2 (Foundational)
                       ↓
         ┌─────────────┼─────────────┐
         ↓             ↓             ↓
    Phase 3 (US1)  Phase 4 (US2)  Phase 5 (US3)  ← Can be done in parallel after Phase 2
    [Chapter 1]    [Chapter 2]    [Chapter 3]
         │             │             │
         └─────────────┴─────────────┘
                       ↓
                 Phase 6 (Polish)
```

**Key Dependencies**:
- Phase 2 MUST complete before any chapter work (US1, US2, US3)
- US1, US2, US3 are INDEPENDENT after Phase 2 (can be done in parallel by different authors)
- Phase 6 requires all three chapters (US1, US2, US3) complete

**No Story-to-Story Dependencies**: Each chapter teaches independent concepts and can be developed/tested/delivered separately.

---

## Parallel Execution Opportunities

### During Phase 2 (Foundational):
- T007 (Gazebo setup) || T008 (Unity setup) || T009 (Robot models) || T010 (Citation templates)

### During Phase 3 (User Story 1 - Chapter 1):
- T014 (Example 1) || T017 (Example 2) || T019 (Example 3) || T023 (Example 4) || T029 (Diagrams)
- All example creation and testing tasks can run in parallel with writing tasks

### During Phase 4 (User Story 2 - Chapter 2):
- T038 (Unity scene) || T041 (Robot import) || T044 (Lighting) || T046 (Mixamo avatar)
- Example creation tasks are parallelizable

### During Phase 5 (User Story 3 - Chapter 3):
- LiDAR tasks (T062-T069) || Depth Camera tasks (T070-T077) || IMU tasks (T078-T085)
- All sensor types can be developed in parallel

### During Phase 6 (Polish):
- T101 (Checklist) || T102 (Troubleshooting) || T103 (Embeddings) || T104 (Chatbot testing) || T105 (Widget integration) || T109 (Release notes) || T110 (README)

**Estimated Parallel Efficiency**: With 2-3 team members, this module can be completed in 40-50% of the sequential time.

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Recommended MVP**: User Story 1 (Chapter 1 - Gazebo Physics) only
- Delivers foundational physics simulation knowledge
- Students can create Gazebo worlds and understand physics parameters
- Provides immediate value for robotics simulation beginners
- Tasks: T001-T034 (34 tasks)
- Estimated effort: 1-1.5 weeks for single author

### Incremental Delivery Plan

1. **Iteration 1 (MVP)**: Phase 1-3 (Setup + Foundational + US1/Chapter 1)
   - Deliverable: Students can create Gazebo simulations
   - Validation: T030-T034 pass

2. **Iteration 2**: Add Phase 4 (US2/Chapter 2 - Unity Rendering)
   - Deliverable: Students can create Unity HRI scenes
   - Validation: T054-T058 pass

3. **Iteration 3**: Add Phase 5 (US3/Chapter 3 - Sensor Simulation)
   - Deliverable: Students can simulate realistic sensors
   - Validation: T091-T096 pass

4. **Iteration 4 (Full Module)**: Add Phase 6 (Polish + Chatbot Integration)
   - Deliverable: Complete, polished module with RAG chatbot support
   - Validation: T106-T110 pass

### Task Execution Approach

**For Content Authoring Tasks**:
1. Research and compile sources first (Phase 2)
2. Create simulation examples and test them
3. Write content with inline examples and citations
4. Validate against quality criteria (word count, readability, citations)

**For Simulation Example Tasks**:
1. Create example file in simulations/ directory
2. Test in Gazebo or Unity, verify output
3. Capture screenshots/videos for documentation
4. Copy validated examples to docs/module-02-digital-twin/assets/ for distribution

**For Validation Tasks**:
1. Run automated checks (word count, citation count)
2. Manual verification (simulation reproducibility, readability)
3. Peer review (have another person follow instructions)
4. Fix any issues and re-validate

---

## Task Summary

- **Total Tasks**: 110
- **Phase 1 (Setup)**: 4 tasks
- **Phase 2 (Foundational)**: 6 tasks
- **Phase 3 (US1/Chapter 1)**: 24 tasks (19 implementation + 5 validation)
- **Phase 4 (US2/Chapter 2)**: 24 tasks (19 implementation + 5 validation)
- **Phase 5 (US3/Chapter 3)**: 38 tasks (32 implementation + 6 validation)
- **Phase 6 (Polish)**: 14 tasks

**Parallelizable Tasks**: 48 tasks marked with [P] (44% can run in parallel)

**Task Distribution by User Story**:
- User Story 1 (Chapter 1): 24 tasks (T011-T034)
- User Story 2 (Chapter 2): 24 tasks (T035-T058)
- User Story 3 (Chapter 3): 38 tasks (T059-T096)

**Independent Test Criteria**:
- US1: Student creates Gazebo world with custom physics (T034 peer review)
- US2: Student creates Unity HRI scene with avatar (T058 peer review)
- US3: Student simulates all three sensor types and verifies outputs (T096 peer review)

**Format Validation**: ✅ All 110 tasks follow the required checklist format with checkbox, task ID, optional [P] marker, [Story] label where appropriate, and file paths in descriptions.

---

**Generated**: 2025-12-07 by `/sp.tasks`
**Next Steps**: Run `/sp.implement` to execute tasks, or manually implement following the MVP → Full Module incremental delivery plan.
