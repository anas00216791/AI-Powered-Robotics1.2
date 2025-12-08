# Research: Module 2: The Digital Twin (Gazebo & Unity)

**Date**: 2025-12-07
**Feature**: 002-gazebo-unity-sim
**Purpose**: Resolve technical clarifications and establish best practices for simulation-based robotics education content

## Research Tasks

### 1. RAG Chatbot Database Schema (PostgreSQL/Neon)

**Question**: What is the exact schema for storing chatbot metadata in PostgreSQL/Neon?

**Research Approach**:
- Review RAG architecture patterns for educational chatbots
- Identify metadata requirements: user queries, retrieved chunks, response quality metrics
- Evaluate Neon-specific features (serverless, branching)

**Findings**:

**Decision**: Use the following schema for Neon PostgreSQL:

```sql
-- User query history
CREATE TABLE queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_text TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    response_text TEXT,
    response_time_ms INTEGER,
    user_session_id VARCHAR(255)
);

-- Retrieved document chunks (for debugging/analytics)
CREATE TABLE retrievals (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_id UUID REFERENCES queries(id),
    chunk_id VARCHAR(255),  -- Qdrant point ID
    chunk_text TEXT,
    relevance_score FLOAT,
    source_document VARCHAR(500)  -- e.g., "module-02/chapter-01-gazebo-physics.md"
);

-- User feedback on answers
CREATE TABLE feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_id UUID REFERENCES queries(id),
    is_helpful BOOLEAN,
    feedback_text TEXT,
    timestamp TIMESTAMP DEFAULT NOW()
);
```

**Rationale**:
- Minimal schema for MVP: track queries, retrievals, and feedback
- `user_session_id` allows analytics without requiring user accounts
- `retrievals` table enables debugging and improving retrieval quality
- `feedback` table supports iterative improvement of chatbot answers

**Alternatives Considered**:
- **No relational DB, Qdrant only**: Rejected because Qdrant is optimized for vector search, not analytics/feedback
- **MongoDB**: Rejected to maintain consistency with constitution's PostgreSQL preference and Neon's serverless benefits

---

### 2. Gazebo Version Selection (Classic 11 vs. Harmonic)

**Question**: Should the module use Gazebo Classic 11 or Gazebo Harmonic (new architecture)?

**Research Approach**:
- Review Gazebo roadmap and LTS support timelines
- Evaluate ROS 2 Humble integration with both versions
- Consider student accessibility and documentation quality

**Findings**:
- **Gazebo Classic 11**: Last release of Classic series, widely used, extensive documentation, ROS 2 Humble integration via `gazebo_ros_pkgs`
- **Gazebo Harmonic**: New architecture (Ignition → Gazebo), modern features, active development, ROS 2 integration via `ros_gz`
- **LTS Support**: Classic 11 reached EOL Jan 2025; Harmonic has LTS until 2026

**Decision**: **Use both, with Gazebo Harmonic as primary and Classic 11 as reference**

**Rationale**:
- Harmonic is the future; students need to learn modern Gazebo
- Many existing resources and tutorials use Classic 11
- Parallel examples demonstrate migration path and platform differences
- Chapter 1 structure: "Gazebo Fundamentals (Harmonic) with Classic 11 Notes"

**Alternatives Considered**:
- **Harmonic only**: Rejected because many students may encounter Classic 11 in existing projects
- **Classic 11 only**: Rejected because EOL reached; would be outdated immediately

---

### 3. Unity Robotics Packages and Integration

**Question**: What Unity packages/plugins are best for robotics simulation and ROS integration?

**Research Approach**:
- Review Unity Robotics Hub (official Unity-ROS integration)
- Evaluate URDF Importer, ROS-TCP-Connector, and Perception packages
- Check compatibility with Unity 2022 LTS and ROS 2 Humble

**Findings**:
- **Unity Robotics Hub**: Official Unity packages for ROS integration (https://github.com/Unity-Technologies/Unity-Robotics-Hub)
  - `URDF-Importer`: Import robot models from URDF format
  - `ROS-TCP-Connector`: Bidirectional ROS message communication
  - `Perception`: Synthetic data generation for ML/CV
- **Unity 2022 LTS Support**: All packages compatible
- **ROS 2 Support**: ROS-TCP-Endpoint supports ROS 2 Humble

**Decision**: Use Unity Robotics Hub packages as primary integration method

**Code/Config Example**:
```json
// Unity package.json additions
{
  "dependencies": {
    "com.unity.robotics.urdf-importer": "0.5.2",
    "com.unity.robotics.ros-tcp-connector": "0.7.0",
    "com.unity.perception": "1.0.0"
  }
}
```

**Rationale**:
- Official Unity support with ongoing maintenance
- Comprehensive documentation and tutorials
- Enables direct comparison with Gazebo's native ROS 2 integration
- Perception package supports future modules (computer vision, ML)

**Alternatives Considered**:
- **Custom ROS bridge**: Rejected due to maintenance burden and lack of student support resources
- **ROS# (third-party)**: Rejected because Unity Robotics Hub is now official and more actively maintained

---

### 4. Sensor Noise Models and Realism

**Question**: How should sensor noise be modeled to balance educational clarity with realism?

**Research Approach**:
- Review peer-reviewed papers on sensor modeling in simulation
- Examine Gazebo sensor plugins (noise models)
- Evaluate Unity sensor simulation best practices
- Reference real sensor specifications (Velodyne LiDAR, Intel RealSense, common IMUs)

**Findings**:
- **Gazebo Noise Models**: Supports Gaussian noise via `<noise>` SDF tag for sensors
  - Example: `<noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise>`
- **Unity Noise**: Manual implementation via scripts; Perception package provides some tools
- **Pedagogical Approach**: Start simple (no noise), then add progressively
- **Real-World Specs**:
  - **Velodyne VLP-16 LiDAR**: ±3cm accuracy, 100m range
  - **Intel RealSense D435**: Depth error <2% at <3m
  - **IMU (e.g., MPU-6050)**: Accelerometer ±0.01 g, Gyroscope ±0.05°/s

**Decision**: Three-tier approach for each sensor type

**Tier 1 - Simplified (Initial Examples)**:
- No noise, ideal conditions
- Focus: Understanding sensor output format and visualization

**Tier 2 - Basic Noise (Intermediate Examples)**:
- Gaussian noise with realistic parameters
- Gazebo: Use `<noise>` tags
- Unity: Add random offset/noise in sensor scripts

**Tier 3 - Advanced Realism (Optional/Advanced Sections)**:
- Multi-component noise (bias, drift, temperature effects for IMU)
- Ray-tracing artifacts for LiDAR (reflections, multi-path)
- Depth camera distortion and IR interference

**Rationale**:
- Progressive complexity matches pedagogical goals
- Students build intuition before tackling realistic complexity
- Advanced sections prepare students for real-world deployment
- Each tier is clearly labeled in content

**Alternatives Considered**:
- **Full realism from start**: Rejected because overwhelming for beginners
- **No noise modeling**: Rejected because unrealistic and doesn't prepare students for real sensors

---

### 5. Human Avatar Assets for Unity HRI Scenarios

**Question**: What human avatar assets should be recommended for human-robot interaction (HRI) scenarios?

**Research Approach**:
- Survey free/open-source Unity human avatar packages
- Evaluate rigged models with animation support
- Check licensing for educational use
- Consider animation controller complexity for students

**Findings**:
- **Unity Asset Store Free Assets**:
  - "Robot Kyle" (free, rigged, animations included) - not human but useful for testing
  - "Mixamo" (Adobe service): Free rigged characters with animations, Unity-compatible FBX export
- **Open Source Options**:
  - "ManuelbastioniLAB" (Blender addon → Unity): Detailed human models, requires Blender workflow
- **Unity ML-Agents Example Assets**: Include simple humanoid models for RL training

**Decision**: Recommend **Mixamo** as primary source with Unity ML-Agents humanoid as alternative

**Instructions for Students**:
1. Visit mixamo.com (free with Adobe account)
2. Select a rigged character (e.g., "Jasper", "Olivia")
3. Download with animations (T-pose, Walk, Wave, etc.)
4. Import FBX into Unity
5. Use Unity's Animator Controller for HRI scenarios

**Rationale**:
- Mixamo is industry-standard, free, and easy to use
- Wide variety of characters and animations
- No complex rigging/animation skills required
- Students can focus on HRI logic rather than 3D modeling

**Alternatives Considered**:
- **Custom Blender models**: Rejected due to steep learning curve for robotics students
- **Paid assets**: Rejected to keep content accessible (students may not have budget)

---

### 6. Docusaurus RAG Chatbot Integration Approach

**Question**: How should the RAG chatbot be embedded into the Docusaurus site?

**Research Approach**:
- Review Docusaurus plugin architecture
- Evaluate React component integration methods
- Consider UX patterns for educational chatbots (sidebar, modal, inline)

**Findings**:
- **Docusaurus Custom Components**: Can create React components in `src/components/` and import into MDX
- **Swizzling Theme Components**: Can customize Docusaurus theme to add chatbot globally
- **Plugin Development**: Can create custom Docusaurus plugin for more advanced integration

**Decision**: Use React component with global theme swizzling for persistent chatbot widget

**Implementation Approach**:
```javascript
// src/components/ChatbotWidget.tsx
import React, { useState } from 'react';

export default function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState('');

  const handleQuery = async () => {
    const res = await fetch('https://chatbot-api.example.com/query', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ query })
    });
    const data = await res.json();
    setResponse(data.answer);
  };

  return (
    <div className="chatbot-widget">
      {/* Floating button + modal/sidebar UI */}
    </div>
  );
}
```

**Integration**:
- Swizzle Docusaurus theme's `Root` component
- Inject `<ChatbotWidget />` globally
- Styled as floating button (bottom-right corner) with expandable chat interface

**Rationale**:
- Always accessible without interrupting reading flow
- Familiar UX pattern (similar to customer support chats)
- React component integrates naturally with Docusaurus
- Backend API remains decoupled for independent scaling

**Alternatives Considered**:
- **Inline per-chapter chatbot**: Rejected because users might not see it on every page
- **Separate chatbot page**: Rejected because reduces discoverability and breaks reading flow

---

## Summary of Key Decisions

| Topic | Decision | Impact |
|-------|----------|--------|
| **RAG Database Schema** | PostgreSQL/Neon with queries, retrievals, feedback tables | Enables analytics and iterative improvement |
| **Gazebo Version** | Harmonic primary, Classic 11 reference | Modern platform + backward compatibility |
| **Unity Integration** | Unity Robotics Hub packages (URDF Importer, ROS-TCP-Connector) | Official support, comprehensive docs |
| **Sensor Noise** | Three-tier approach (simple → basic noise → advanced) | Progressive learning curve |
| **Human Avatars** | Mixamo (free) + Unity ML-Agents humanoid | Accessible, industry-standard |
| **Chatbot Integration** | React component with theme swizzling | Persistent, non-intrusive UX |

All "NEEDS CLARIFICATION" items from Technical Context have been resolved. Ready to proceed to Phase 1 (Design & Contracts).
