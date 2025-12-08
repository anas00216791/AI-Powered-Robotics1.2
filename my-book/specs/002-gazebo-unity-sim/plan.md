# Implementation Plan: Module 2: The Digital Twin (Gazebo & Unity)

**Branch**: `002-gazebo-unity-sim` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-gazebo-unity-sim/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This module provides comprehensive educational content on robotics simulation using Gazebo and Unity, covering physics simulation, high-fidelity rendering, and sensor simulation (LiDAR, depth cameras, IMUs). The content targets AI/robotics students and delivers 3000-5000 words in Markdown format with APA citations, ensuring at least 50% of sources are peer-reviewed or official documentation. The module enables students to create realistic digital twins of robotic systems through three progressive chapters: Gazebo physics fundamentals, Unity rendering and human-robot interaction, and integrated sensor simulation across both platforms.

## Technical Context

**Language/Version**: Markdown (content authoring), Python 3.10+ (RAG chatbot backend), JavaScript/React (Docusaurus frontend)
**Primary Dependencies**: Docusaurus 3.x, Gazebo Classic 11 or Gazebo Harmonic, Unity 2022 LTS or Unity 6, ROS 2 Humble
**Storage**: Markdown files (content), Qdrant vector database (RAG embeddings), PostgreSQL/Neon (chatbot metadata - NEEDS CLARIFICATION on exact schema)
**Testing**: Manual content validation, simulation reproducibility tests, RAG chatbot answer accuracy validation against book content
**Target Platform**: Web (Docusaurus/GitHub Pages), Ubuntu 22.04+ (Gazebo/ROS 2), Windows/macOS/Linux (Unity)
**Project Type**: Documentation/Educational book with embedded web-based RAG chatbot
**Performance Goals**: Chatbot response time <2s, simulation examples load without errors, 90%+ student task completion rate
**Constraints**: 3000-5000 words per module, APA citations with ≥50% peer-reviewed/official sources, Flesch-Kincaid grade 10-12, reproducible examples
**Scale/Scope**: Single module (part of 4+ module series), 3 chapters, ~15-20 code/simulation examples, 10-15 unique citations for this module

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Evidence/Plan |
|-----------|-------------|--------|---------------|
| **I. Accuracy via Primary Sources** | All technical content from authoritative primary sources | ✅ PASS | Will cite official Gazebo docs (gazebosim.org), Unity docs (docs.unity3d.com), ROS 2 docs, and peer-reviewed robotics simulation papers |
| **II. Clarity for Technical Audiences** | Flesch-Kincaid grade 10-12, define concepts before use | ✅ PASS | Target audience is undergraduate/graduate students; technical jargon is domain-standard; each concept will be defined with examples |
| **III. Reproducibility** | All code/simulations must be reproducible with specified environments | ✅ PASS | Will specify Ubuntu 22.04, Gazebo 11/Harmonic, Unity 2022 LTS, include world files, parameter configs, step-by-step instructions |
| **IV. Rigor via Peer-Reviewed Sources** | ≥50% citations from peer-reviewed or official docs | ✅ PASS | Module spec requires 10-15 citations with ≥50% from official docs or peer-reviewed papers on simulation/sensor modeling |
| **V. Zero Plagiarism** | Original content with proper APA attribution | ✅ PASS | All content will be original; code adapted from sources will include inline attribution; APA format enforced |
| **VI. Deliverables & Tooling** | Docusaurus + Markdown, 20k-30k words (full book), RAG chatbot | ⚠️ PARTIAL | This module is 3k-5k words (part of full book). Docusaurus structure established. RAG integration at book level (not module level) |
| **VII. AI-to-Robot Pipeline Reproducibility** | Simulations use ROS 2/Gazebo/Unity/Isaac with validation protocol | ✅ PASS | Module focuses on Gazebo + Unity simulation fundamentals; sensor outputs will be validated against real-world sensor specs |

**Initial Assessment**: PASS with note on Principle VI (module is part of larger book)

**Flagged for Phase 1 Re-check**:
- Exact RAG chatbot architecture (FastAPI + Qdrant + Neon schema)
- Docusaurus plugin integration for RAG chatbot
- Citation verification process (manual vs. automated tooling)

---

### Phase 1 Re-Evaluation (Post-Design)

**Date**: 2025-12-07

All flagged items have been resolved:

| Item | Resolution | Document Reference |
|------|-----------|-------------------|
| **RAG chatbot architecture** | ✅ RESOLVED | Defined in `research.md` (Section 1: Database Schema) and `data-model.md` (Section 2: RAG Chatbot Entities). FastAPI + Qdrant + Neon PostgreSQL with queries/retrievals/feedback tables. |
| **Docusaurus plugin integration** | ✅ RESOLVED | Defined in `research.md` (Section 6: Chatbot Integration). Using React component with theme swizzling for global chatbot widget. Implementation details in `quickstart.md`. |
| **Citation verification** | ✅ RESOLVED | Manual validation process defined in `data-model.md` (Section 3.3: Citation entity). Pre-publication checklist includes APA format validation and ≥50% primary source ratio verification. |

**Updated Constitution Check**:

| Principle | Status | Phase 1 Evidence |
|-----------|--------|------------------|
| **I. Accuracy via Primary Sources** | ✅ PASS | Research identifies specific official docs (Gazebo, Unity, ROS 2) and peer-reviewed paper requirements. Citation entity enforces source typing. |
| **II. Clarity for Technical Audiences** | ✅ PASS | Quickstart provides writing templates and readability checking tools. Three-tier sensor noise approach balances clarity with realism. |
| **III. Reproducibility** | ✅ PASS | Data model defines validation rules for all code/simulation examples. SimulationExample entity requires `validation_status = "passed"` before publication. |
| **IV. Rigor via Peer-Reviewed Sources** | ✅ PASS | Citation entity with `is_primary_source` field enforces ≥50% ratio. Manual validation in pre-publication checklist. |
| **V. Zero Plagiarism** | ✅ PASS | Content authoring workflow in quickstart emphasizes original writing. Code attribution documented in data model. |
| **VI. Deliverables & Tooling** | ✅ PASS | Full project structure defined. Docusaurus setup in quickstart. RAG chatbot API contract complete (OpenAPI 3.0). Module contributes to 20k-30k word book goal. |
| **VII. AI-to-Robot Pipeline Reproducibility** | ✅ PASS | Gazebo/Unity version decisions documented in research. Sensor validation against real-world specs (Velodyne, RealSense, MPU-6050) in data model. |

**Final Assessment**: ✅ ALL GATES PASSED

Ready to proceed to Phase 2 (Task Generation via `/sp.tasks`)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus book content structure
docs/
├── module-01-ros2-basics/
│   ├── index.md
│   └── [chapters from Module 1]
├── module-02-digital-twin/              # THIS MODULE
│   ├── index.md
│   ├── chapter-01-gazebo-physics.md
│   ├── chapter-02-unity-rendering.md
│   ├── chapter-03-sensor-simulation.md
│   └── assets/
│       ├── gazebo-worlds/               # .world files
│       ├── unity-scenes/                # Scene configs/screenshots
│       └── diagrams/                    # Physics diagrams, sensor FOV
├── module-03-isaac/
│   └── [future modules]
└── module-04-vla/
    └── [future modules]

# RAG chatbot backend
chatbot/
├── src/
│   ├── api/                             # FastAPI endpoints
│   ├── embeddings/                      # Document chunking & embedding
│   ├── retrieval/                       # Qdrant integration
│   └── models/                          # Neon DB schema
├── tests/
│   ├── test_embeddings.py
│   ├── test_retrieval.py
│   └── test_accuracy.py                 # Validate answers vs. book content
└── requirements.txt

# Docusaurus site
docusaurus/
├── src/
│   ├── components/
│   │   └── ChatbotWidget.tsx            # Embedded RAG chatbot UI
│   ├── pages/
│   └── theme/
├── docusaurus.config.js
└── package.json

# Simulation assets (validation/testing)
simulations/
├── gazebo/
│   ├── worlds/                          # Test worlds from module examples
│   └── models/                          # Robot models (from Module 1)
└── unity/
    ├── scenes/                          # Unity test scenes
    └── assets/                          # 3D models, textures
```

**Structure Decision**: This is a documentation project with embedded chatbot. The module content lives in `docs/module-02-digital-twin/` using Docusaurus conventions. Simulation assets (Gazebo worlds, Unity scenes) are stored in `simulations/` for validation and in `docs/module-02-digital-twin/assets/` for distribution to readers. The RAG chatbot is a separate backend service in `chatbot/` integrated via a React component in the Docusaurus frontend.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations requiring justification. Principle VI (Deliverables & Tooling) is marked PARTIAL because this is one module of a larger book, which is expected and not a violation.
