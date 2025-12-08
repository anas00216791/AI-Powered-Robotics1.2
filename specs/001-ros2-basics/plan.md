# Implementation Plan: ROS 2 Basics Module (Book Chapter)

**Branch**: `001-ros2-basics` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-ros2-basics/spec.md`

## Summary

This plan covers the implementation of **Module 1: The Robotic Nervous System (ROS 2)**, the foundational chapter in an educational book on AI and robotics. The module introduces students to ROS 2 middleware, Python integration via rclpy, and URDF modeling for humanoid robots. The content will be authored in Markdown using Docusaurus and will be research-concurrent, meaning source gathering, drafting, and technical validation occur in parallel phases.

**Technical Approach**: Docusaurus-based book architecture with modular chapters, peer-reviewed and official source citations, reproducible Python code examples, and URDF models validated in ROS 2 tooling.

## Technical Context

**Language/Version**: Markdown (Docusaurus 3.x), Python 3.10+, ROS 2 Humble LTS
**Primary Dependencies**: Docusaurus (static site generator), rclpy (ROS 2 Python client), Rviz (visualization), robot_state_publisher, check_urdf
**Storage**: Git version control for all content, code, and URDF files; RAG chatbot indexes compiled Markdown
**Testing**: Python syntax/runtime validation, URDF validation (check_urdf), simulation verification (Rviz), citation format validation (APA), peer review
**Target Platform**: Web (Docusaurus site on GitHub Pages), PDF export, RAG chatbot embedded in site
**Project Type**: Educational book (multi-module documentation project)
**Performance Goals**: N/A (static content delivery)
**Constraints**: 3,000–5,000 words per module, ≥50% peer-reviewed/official citations, APA format, 2-week delivery
**Scale/Scope**: Module 1 of 4 total modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Accuracy via Primary Sources
- ✅ **PASS**: All ROS 2 concepts will be sourced from official ROS 2 documentation (docs.ros.org) and peer-reviewed robotics papers
- ✅ **PASS**: Python code examples will reference official rclpy API documentation
- ✅ **PASS**: URDF syntax will reference official URDF specification

### Principle II: Clarity for Technical Audiences
- ✅ **PASS**: Content targets AI/robotics students (undergraduate/graduate level)
- ✅ **PASS**: Flesch-Kincaid grade level 10-12 will be verified in Phase 3
- ✅ **PASS**: Technical jargon (Nodes, Topics, Services, URDF) will be defined before use

### Principle III: Reproducibility of Code, Simulations, and Experiments
- ✅ **PASS**: All Python examples will specify ROS 2 Humble, Python 3.10+, Ubuntu 22.04
- ✅ **PASS**: URDF models will be validated using check_urdf and visualized in Rviz
- ✅ **PASS**: Step-by-step instructions will be tested for reproducibility

### Principle IV: Rigor via Peer-Reviewed and Official Sources
- ✅ **PASS**: ≥50% of citations will be peer-reviewed or official docs (spec requirement FR-007)
- ✅ **PASS**: Source quality tracking will be maintained during research phase

### Principle V: Zero Plagiarism and Proper Attribution
- ✅ **PASS**: APA citation format enforced (spec requirement FR-006)
- ✅ **PASS**: All code adapted from examples will include attribution comments

### Principle VI: Deliverables and Tooling Constraints
- ✅ **PASS**: Docusaurus will be used for book structure
- ✅ **PASS**: Module word count: 3,000–5,000 words (aligned with constitution's 20k–30k total for 4 modules)
- ⚠️ **NOTE**: RAG chatbot integration is book-level, not module-level (deferred to book assembly phase)

### Principle VII: AI-to-Robot Pipeline Reproducibility
- ✅ **PASS**: ROS 2 examples use standard ROS 2 tools (rclpy, Rviz, robot_state_publisher)
- ✅ **PASS**: URDF models validated in simulation environment
- ⚠️ **NOTE**: Full AI-to-robot pipeline is addressed in later modules (Gazebo/Unity, Isaac Sim, VLA)

**Constitution Status**: ✅ **ALL CHECKS PASSED** - Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-basics/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (architectural plan)
├── research.md          # Phase 0: Source gathering and literature review
├── data-model.md        # Phase 1: Content outline and code structure
├── quickstart.md        # Phase 1: Setup instructions for students
├── contracts/           # Phase 1: API contracts for code examples
│   ├── publisher-node.md
│   ├── subscriber-node.md
│   └── service-node.md
├── checklists/
│   └── requirements.md  # Quality checklist (completed)
└── tasks.md             # Phase 2: Testable implementation tasks (/sp.tasks)
```

### Source Code (repository root)

```text
# Book structure (Docusaurus)
docs/
├── module-01-ros2/
│   ├── index.md                   # Module 1 landing page
│   ├── chapter-01-basics.md       # Chapter 1: ROS 2 Basics
│   ├── chapter-02-python.md       # Chapter 2: Python Agents with rclpy
│   └── chapter-03-urdf.md         # Chapter 3: Humanoid URDF Modeling
├── module-02-simulation/          # Future: Gazebo/Unity module
├── module-03-isaac/               # Future: NVIDIA Isaac module
└── module-04-vla/                 # Future: VLA module

# Code examples (reproducible, tested)
examples/
├── ros2-basics/
│   ├── publisher_node.py          # Chapter 2: Simple publisher
│   ├── subscriber_node.py         # Chapter 2: Simple subscriber
│   ├── service_server.py          # Chapter 2: Service example
│   ├── service_client.py          # Chapter 2: Client example
│   └── urdf/
│       ├── simple_humanoid.urdf   # Chapter 3: Basic humanoid model
│       └── launch_rviz.launch.py  # Chapter 3: Rviz launch file
└── tests/
    └── test_ros2_basics.py        # Automated validation tests

# Docusaurus configuration
docusaurus.config.js               # Site config, modules, sidebar
sidebars.js                        # Navigation structure
package.json                       # Node dependencies for Docusaurus
static/                            # Images, diagrams, assets
└── img/
    └── ros2-basics/               # Diagrams for Module 1
        ├── node-topic-diagram.png
        └── service-diagram.png

# RAG chatbot (book-level, deferred to integration phase)
rag-chatbot/
├── index/                         # Vector index of book content
├── chatbot.py                     # RAG chatbot implementation
└── embed_in_docusaurus.js         # Widget for Docusaurus site
```

**Structure Decision**: **Docusaurus-based multi-module book** with dedicated directories for each module. Code examples are separate from documentation for clarity and testability. Docusaurus provides:
- Sidebar navigation for easy chapter browsing
- Search functionality built-in
- Markdown support with code syntax highlighting
- PDF export via plugins
- GitHub Pages deployment

## Complexity Tracking

*No constitution violations. This section is not applicable.*

## Scope and Dependencies

### In Scope

**Chapter 1: ROS 2 Basics**
- Explanation of ROS 2 middleware architecture
- Definition and role of Nodes, Topics, Services
- Publisher/subscriber communication pattern
- Request/response (Service) communication pattern
- Diagrams illustrating data flow

**Chapter 2: Python Agents with rclpy**
- Setting up a ROS 2 Python development environment
- Creating a minimal publisher node
- Creating a minimal subscriber node
- Implementing a simple service (server and client)
- Running and testing nodes

**Chapter 3: Humanoid URDF Modeling**
- Introduction to URDF syntax and structure
- Defining links (robot body parts)
- Defining joints (connections between links)
- Creating a simple humanoid model (torso, head, limbs)
- Validating URDF with check_urdf
- Visualizing the model in Rviz

**Supporting Content**
- Prerequisites section (Python, Linux CLI, ROS 2 installation)
- Troubleshooting common setup issues
- References to official documentation for deeper learning
- APA-formatted citations (≥50% peer-reviewed/official)

### Out of Scope

- Full humanoid AI applications (planning, decision-making)
- Physics simulation in Gazebo, Unity, or Isaac Sim
- Advanced AI planning algorithms or behavior trees
- Computer vision or sensor integration
- ROS 2 performance tuning or real-time considerations
- Multi-robot systems
- Custom ROS 2 message types
- ROS 2 security (SROS2)
- ROS 2 bag files and data recording

### External Dependencies

**Software**
- ROS 2 Humble LTS (official distribution)
- Python 3.10+ (included with ROS 2)
- rclpy (ROS 2 Python client library)
- Rviz (ROS 2 visualization tool)
- robot_state_publisher (ROS 2 package)
- check_urdf (liburdfdom-tools package)

**Documentation Sources**
- Official ROS 2 documentation (docs.ros.org)
- Peer-reviewed robotics papers (IEEE, Springer, ACM)
- URDF specification (ROS Wiki/official docs)
- Python API reference for rclpy

**Tooling**
- Docusaurus 3.x (Node.js-based static site generator)
- Git (version control)
- GitHub Pages (deployment)
- Citation management (manual APA formatting or tools like Zotero)

## Key Decisions and Rationale

### Decision 1: Docusaurus for Book Architecture

**Options Considered**:
1. **Docusaurus** (React-based static site generator)
2. Sphinx (Python documentation tool)
3. GitBook (commercial documentation platform)
4. MkDocs (lightweight Python static site)

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| Docusaurus | Modern UI, versioning support, plugin ecosystem, PDF export, RAG-friendly | Requires Node.js, heavier than MkDocs |
| Sphinx | Python-native, widely used in research | Less modern UI, steeper learning curve for customization |
| GitBook | Polished UI, easy to use | Proprietary, limited offline capabilities |
| MkDocs | Lightweight, Python-based, simple | Limited plugin ecosystem, less feature-rich |

**Rationale**: **Docusaurus** is chosen because:
- Constitution Principle VI mandates Docusaurus
- Modern, responsive UI suitable for technical audiences
- Built-in search and navigation
- Plugin support for PDF export and RAG chatbot embedding
- GitHub Pages deployment is straightforward
- Versioning support for future book updates

### Decision 2: Module and Chapter Organization

**Options Considered**:
1. **Flat structure**: All chapters in one directory
2. **Module-based structure**: Chapters grouped by module (ROS 2, Gazebo, Isaac, VLA)
3. **Skill-based structure**: Chapters grouped by skill (setup, coding, simulation, deployment)

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| Flat | Simple, easy to navigate | Becomes unwieldy with 10+ chapters |
| Module-based | Logical grouping, aligns with user's learning path | Slightly more complex directory structure |
| Skill-based | Emphasizes capabilities | Doesn't align with technology progression |

**Rationale**: **Module-based structure** is chosen because:
- Aligns with the user's stated requirement: "Sections by modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)"
- Logical progression: middleware → simulation → advanced AI
- Each module can be independently navigated
- Supports future expansion (additional modules or sub-chapters)
- Sidebar navigation in Docusaurus naturally supports this hierarchy

### Decision 3: Research-Concurrent Workflow (4 Phases)

**Options Considered**:
1. **Sequential workflow**: Research → Design → Implementation → Validation
2. **Research-concurrent workflow**: Research overlaps with drafting, analysis, and synthesis
3. **Agile sprints**: Short iterations with continuous integration

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| Sequential | Clear gates, less rework | Slow, inflexible, late discovery of issues |
| Research-concurrent | Faster, issues discovered early, iterative | Requires coordination, potential for churn |
| Agile | Highly iterative, continuous feedback | Overhead for small team, less predictable |

**Rationale**: **Research-concurrent workflow** is chosen because:
- User explicitly requested: "Research-concurrent approach"
- Supports the 2-week timeline constraint
- Enables iterative refinement (research informs drafts, testing informs revisions)
- Constitution Principle III (Reproducibility) requires validation during drafting, not after
- Four phases align with the user's stated phases:
  1. **Research**: Gather sources, identify peer-reviewed papers, extract key concepts
  2. **Foundation**: Draft outlines, write code examples, create URDF models
  3. **Analysis**: Test simulations, validate code, verify citations, peer review
  4. **Synthesis**: Compile final content, integrate into Docusaurus, deploy

### Decision 4: Python Libraries and ROS 2 Nodes

**Options Considered**:
1. **rclpy only** (official ROS 2 Python client)
2. rclpy + ros2py (additional abstraction layer)
3. rclpy + custom utilities (helper functions)

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| rclpy only | Official, well-documented, minimal dependencies | Slightly verbose for simple examples |
| rclpy + ros2py | Simplified API for beginners | Adds dependency, less official support |
| rclpy + custom | Tailored to book examples | Maintenance burden, non-standard |

**Rationale**: **rclpy only** is chosen because:
- Constitution Principle I (Primary Sources): rclpy is the official ROS 2 Python client
- Constitution Principle III (Reproducibility): Using only official libraries ensures long-term compatibility
- Spec requirement FR-002: "functional Python code examples for creating ROS 2 nodes using `rclpy`"
- Simplicity: No additional abstractions to learn or maintain
- Students will use rclpy in real-world projects, so book should teach it directly

### Decision 5: URDF Validation and Visualization Tools

**Options Considered**:
1. **check_urdf + Rviz** (official ROS 2 tools)
2. check_urdf + Gazebo (physics simulation)
3. check_urdf + custom viewer

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| check_urdf + Rviz | Official, lightweight, fast feedback | No physics simulation |
| check_urdf + Gazebo | Full physics, realistic testing | Overkill for basic URDF, slower, out of scope |
| Custom viewer | Tailored UI | Reinventing the wheel, maintenance burden |

**Rationale**: **check_urdf + Rviz** is chosen because:
- Spec explicitly states: "URDF setup demonstrated" and "visualize a URDF robot model using standard ROS 2 tools"
- Scope excludes Gazebo simulation (that's Module 2)
- Rviz is included with standard ROS 2 installations (no extra dependencies)
- Fast iteration: students can quickly see their models
- Constitution Principle III (Reproducibility): Both tools are standard and well-documented

### Decision 6: RAG Chatbot Integration

**Options Considered**:
1. **OpenAI API + LangChain** (cloud-based, high-quality embeddings)
2. Local embeddings (sentence-transformers) + LangChain (privacy-preserving, offline)
3. Docusaurus search plugin only (no RAG)

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| OpenAI + LangChain | High-quality embeddings, fast | API costs, internet dependency |
| Local + LangChain | Privacy, offline, free | Slightly lower quality, requires GPU for fast inference |
| Search only | Simple, no AI overhead | No context-aware answers, limited interactivity |

**Rationale**: **Deferred to book-level integration phase** because:
- Constitution Principle VI mandates RAG chatbot, but it's a book-wide feature, not module-specific
- Module 1 focuses on content creation; RAG integration happens after all modules are complete
- Preliminary decision: **Local embeddings + LangChain** for privacy and offline access (aligns with educational use case)
- Final decision will be made during book assembly phase, considering performance and user feedback

### Decision 7: Quality Checks and Testing Strategy

**User-specified checks**:
- Code reproducibility
- Simulation validation
- RAG chatbot accuracy

**Options Considered**:
1. **Manual testing only** (human reviewer runs examples)
2. Automated tests + manual review
3. Continuous integration (CI) pipeline

**Trade-offs**:
| Option | Pros | Cons |
|--------|------|------|
| Manual | Simple, flexible | Slow, error-prone, not scalable |
| Automated + manual | Fast, reliable, repeatable | Requires test infrastructure |
| CI pipeline | Continuous validation, high confidence | Overhead for small project |

**Rationale**: **Automated tests + manual peer review** is chosen because:
- **Code reproducibility**: Automated Python syntax checks and runtime tests
- **Simulation validation**: Automated check_urdf validation, manual Rviz verification
- **RAG chatbot accuracy**: Deferred to book-level testing (not module-specific)
- **Peer review**: Human review for technical accuracy, citation quality, and readability
- Balances speed (automation) with quality (human judgment)
- Aligns with Constitution Principle III (Reproducibility) and IV (Rigor)

## Interfaces and API Contracts

### Python Code Examples

All Python examples will follow this contract:

**Publisher Node Contract** (`examples/ros2-basics/publisher_node.py`):
- **Input**: None (publishes periodically)
- **Output**: Publishes string messages to `/example_topic` at 1 Hz
- **Dependencies**: rclpy, std_msgs.msg.String
- **Environment**: ROS 2 Humble, Python 3.10+
- **Validation**: Node starts without errors, messages visible via `ros2 topic echo /example_topic`

**Subscriber Node Contract** (`examples/ros2-basics/subscriber_node.py`):
- **Input**: Subscribes to `/example_topic` (String messages)
- **Output**: Prints received messages to console
- **Dependencies**: rclpy, std_msgs.msg.String
- **Environment**: ROS 2 Humble, Python 3.10+
- **Validation**: Node starts without errors, prints messages when publisher is running

**Service Server Contract** (`examples/ros2-basics/service_server.py`):
- **Input**: AddTwoInts service requests (two integers)
- **Output**: AddTwoInts service responses (sum of integers)
- **Dependencies**: rclpy, example_interfaces.srv.AddTwoInts
- **Environment**: ROS 2 Humble, Python 3.10+
- **Validation**: Service advertised, responds correctly to `ros2 service call` commands

**Service Client Contract** (`examples/ros2-basics/service_client.py`):
- **Input**: Command-line arguments (two integers)
- **Output**: Prints service response (sum)
- **Dependencies**: rclpy, example_interfaces.srv.AddTwoInts
- **Environment**: ROS 2 Humble, Python 3.10+
- **Validation**: Client successfully calls service and prints correct result

### URDF Model Contract

**Simple Humanoid URDF Contract** (`examples/ros2-basics/urdf/simple_humanoid.urdf`):
- **Structure**:
  - `base_link` (torso)
  - `head` (fixed joint to base_link)
  - `left_arm`, `right_arm` (revolute joints to base_link)
  - `left_leg`, `right_leg` (revolute joints to base_link)
- **Validation**: Passes `check_urdf simple_humanoid.urdf`
- **Visualization**: Loads in Rviz via `robot_state_publisher` and displays all links

**Launch File Contract** (`examples/ros2-basics/urdf/launch_rviz.launch.py`):
- **Input**: URDF file path
- **Output**: Launches `robot_state_publisher` and Rviz with robot model displayed
- **Dependencies**: ROS 2 launch, robot_state_publisher, Rviz
- **Validation**: Rviz window opens, robot model visible, no errors in console

## Non-Functional Requirements

### Usability

- **Target Audience**: AI/robotics students (undergraduate/graduate level)
- **Readability**: Flesch-Kincaid grade level 10-12
- **Code Comments**: Every code example includes inline comments explaining each step
- **Progressive Complexity**: Chapter 1 (theory) → Chapter 2 (practice) → Chapter 3 (application)
- **Troubleshooting**: Common setup issues documented in quickstart guide

### Documentation Quality

- **Citation Format**: APA style, consistently applied
- **Source Quality**: ≥50% peer-reviewed or official documentation
- **Code Accuracy**: All examples syntactically correct and runtime-tested
- **Diagrams**: Clear, labeled diagrams for ROS 2 communication patterns
- **Cross-References**: Links to official docs for deeper exploration

### Maintainability

- **ROS 2 Version**: Target ROS 2 Humble LTS (long-term support until 2027)
- **Python Version**: Python 3.10+ (compatible with ROS 2 Humble)
- **Docusaurus Updates**: Use stable Docusaurus 3.x release
- **Version Control**: Git for all content, code, and configuration
- **Future-Proofing**: Focus on stable ROS 2 concepts (Nodes, Topics, Services) that won't change significantly

### Performance

- **Static Site**: Docusaurus site loads in <2 seconds on modern browsers
- **PDF Export**: Generated PDF is <10 MB for easy distribution
- **Search**: Docusaurus built-in search returns results in <500ms

### Reproducibility

- **Environment Specification**: All examples include Ubuntu 22.04, ROS 2 Humble, Python 3.10+ in comments
- **Dependency Pinning**: Specific versions documented (e.g., rclpy 3.3.x)
- **Validation Protocol**: Automated tests verify examples run without errors
- **Known Limitations**: Document platform-specific issues (e.g., macOS differences)

## Risk Analysis and Mitigation

### Risk 1: ROS 2 Installation Variability

**Description**: Students may have different operating systems, package managers, or ROS 2 distributions, leading to setup failures.

**Likelihood**: High
**Impact**: High (blocks progress)
**Mitigation**:
- Include a detailed prerequisites section in quickstart guide
- Target ROS 2 Humble LTS (most stable)
- Provide links to official installation guides for Ubuntu, Windows (WSL2), macOS
- Document common issues (e.g., sourcing setup.bash, network timeouts)
- Consider Docker-based environment as fallback

### Risk 2: Peer-Reviewed Source Availability

**Description**: Many robotics papers are behind paywalls, making it difficult to meet the ≥50% peer-reviewed citation requirement.

**Likelihood**: Medium
**Impact**: Medium (violates constitution and spec)
**Mitigation**:
- Prioritize open-access journals (e.g., IEEE Xplore open access, arXiv)
- Use institutional access if available
- Official ROS 2 documentation counts as high-quality and is freely accessible
- Focus on foundational papers that are widely cited and often accessible

### Risk 3: Code Example Bugs or Outdated APIs

**Description**: Python code or ROS 2 APIs may change, breaking examples or causing runtime errors.

**Likelihood**: Low (ROS 2 Humble is stable)
**Impact**: High (breaks reproducibility)
**Mitigation**:
- Use ROS 2 Humble LTS (supported until 2027)
- Automate syntax and runtime tests for all code examples
- Pin dependencies (rclpy, std_msgs versions)
- Include version checks in code comments
- Manual testing in fresh environment before publication

### Risk 4: URDF Validation Failures

**Description**: URDF models may have syntax errors, missing tags, or invalid joint definitions, causing check_urdf or Rviz to fail.

**Likelihood**: Medium
**Impact**: Medium (blocks Chapter 3)
**Mitigation**:
- Validate URDF files with check_urdf during drafting
- Test visualization in Rviz before finalizing content
- Provide a minimal working example first, then build complexity
- Include troubleshooting section for common URDF errors

### Risk 5: Timeline Pressure (2 Weeks)

**Description**: The 2-week timeline is tight for research, drafting, validation, and review.

**Likelihood**: High
**Impact**: Medium (may compromise quality)
**Mitigation**:
- Use research-concurrent workflow to parallelize tasks
- Prioritize core content (Chapters 1-3) over optional enhancements
- Leverage official ROS 2 documentation heavily (saves research time)
- Defer RAG chatbot integration to book-level phase
- Focus on "good enough" for MVP, iterate later if needed

### Risk 6: RAG Chatbot Accuracy

**Description**: RAG chatbot may provide incorrect or irrelevant answers, reducing educational value.

**Likelihood**: Medium
**Impact**: Medium (degrades user experience)
**Mitigation**:
- Deferred to book-level integration phase (not module-specific)
- Use high-quality embeddings (OpenAI or sentence-transformers)
- Test chatbot with common student questions
- Include fallback: "Refer to Chapter X for more details"
- Continuous improvement based on user feedback

## Evaluation and Validation

### Definition of Done (Module 1)

**Content**:
- ✅ 3,000–5,000 words written and compiled in Docusaurus
- ✅ All three chapters completed (ROS 2 Basics, Python Agents, URDF Modeling)
- ✅ APA citations included for all sources
- ✅ ≥50% of citations from peer-reviewed or official docs

**Code**:
- ✅ All Python examples (publisher, subscriber, service server, service client) pass syntax and runtime tests
- ✅ URDF model passes check_urdf validation
- ✅ URDF model visualizes correctly in Rviz
- ✅ All code includes environment specification comments

**Quality**:
- ✅ Flesch-Kincaid readability grade level 10-12
- ✅ Technical review confirms ROS 2 concepts are accurate
- ✅ Peer review confirms content is clear and accessible
- ✅ No plagiarism detected (manual review + citation checks)

**Testing**:
- ✅ Automated tests pass for all Python examples
- ✅ Manual validation: Human tester runs examples in fresh ROS 2 environment
- ✅ Manual validation: URDF model loads in Rviz without errors
- ✅ Citation format validated (APA compliance)

### Validation Protocol

**Phase 0: Research**
- [ ] Identify and document ≥10 primary sources (peer-reviewed papers, official docs)
- [ ] Extract key concepts for each chapter
- [ ] Verify source accessibility (no paywalls or provide alternatives)

**Phase 1: Foundation**
- [ ] Draft chapter outlines with section headings
- [ ] Write initial Python code examples
- [ ] Create initial URDF model
- [ ] Draft quickstart guide with prerequisites

**Phase 2: Analysis**
- [ ] Run automated tests on Python examples
- [ ] Validate URDF with check_urdf
- [ ] Visualize URDF in Rviz
- [ ] Peer review for technical accuracy
- [ ] Readability check (Flesch-Kincaid)

**Phase 3: Synthesis**
- [ ] Integrate content into Docusaurus
- [ ] Generate PDF export
- [ ] Final citation review (APA format, source quality)
- [ ] Manual end-to-end test: Fresh environment, follow guide, run examples
- [ ] Address feedback from peer review

### Output Validation

**Success Criteria Mapping**:
- **SC-001** (90% of students run examples successfully): Validated via reproducibility testing
- **SC-002** (3,000–5,000 words): Validated via word count script
- **SC-003** (APA citations, ≥50% peer-reviewed): Validated via citation audit
- **SC-004** (Technical accuracy): Validated via peer review and official doc cross-reference
- **SC-005** (URDF visualization in Rviz): Validated via manual test
- **SC-006** (Students explain Topics vs Services): Validated via content review (clarity check)

## Implementation Phases

### Phase 0: Research (Days 1-2)

**Objective**: Gather authoritative sources, extract key concepts, and validate source quality.

**Activities**:
1. Search for peer-reviewed papers on ROS 2 middleware and robotics middleware
2. Compile list of official ROS 2 documentation pages (Nodes, Topics, Services, rclpy, URDF)
3. Identify Python API reference for rclpy
4. Document URDF specification references
5. Extract key concepts for each chapter (definitions, diagrams, examples)
6. Verify ≥50% of sources are peer-reviewed or official

**Deliverables**:
- `research.md`: Annotated bibliography with source summaries
- List of key concepts per chapter
- Citation quality metrics (peer-reviewed vs. other)

**Gate**: Research must confirm sufficient high-quality sources before proceeding.

### Phase 1: Foundation (Days 3-7)

**Objective**: Draft chapter outlines, write code examples, create URDF models, and document setup instructions.

**Activities**:
1. Draft Chapter 1 outline (ROS 2 Basics): Nodes, Topics, Services, communication patterns
2. Draft Chapter 2 outline (Python Agents): Environment setup, publisher, subscriber, service examples
3. Draft Chapter 3 outline (URDF Modeling): URDF syntax, links, joints, humanoid model, Rviz
4. Write Python code examples: publisher, subscriber, service server, service client
5. Create URDF model: simple_humanoid.urdf with basic structure
6. Write quickstart guide: prerequisites, installation, troubleshooting
7. Create contracts for each code example (input, output, dependencies, validation)
8. Generate diagrams: ROS 2 communication patterns (Topics, Services)

**Deliverables**:
- `data-model.md`: Content outline and structure
- `quickstart.md`: Setup instructions for students
- `contracts/`: API contracts for code examples
- `examples/ros2-basics/`: All Python and URDF files
- Draft diagrams (PNG or ASCII art)

**Gate**: Code examples must pass basic syntax checks; URDF must pass check_urdf.

### Phase 2: Analysis (Days 8-11)

**Objective**: Test simulations, validate code, verify citations, and conduct peer review.

**Activities**:
1. Run automated tests on Python examples (syntax, runtime)
2. Validate URDF with check_urdf
3. Visualize URDF in Rviz and document results
4. Peer review: Technical expert reviews ROS 2 concepts for accuracy
5. Readability check: Calculate Flesch-Kincaid grade level
6. Citation audit: Verify APA format and source quality (≥50% peer-reviewed)
7. Manual reproducibility test: Run examples in fresh ROS 2 Humble environment
8. Address review feedback: Fix bugs, clarify unclear sections, add missing citations

**Deliverables**:
- Test results: Pass/fail for all code examples
- URDF validation report: check_urdf output, Rviz screenshots
- Peer review report: Accuracy feedback and recommended changes
- Citation audit report: APA compliance and source quality metrics
- Updated content with fixes applied

**Gate**: All tests must pass; peer review must confirm technical accuracy; citations must meet quality threshold.

### Phase 3: Synthesis (Days 12-14)

**Objective**: Compile final content, integrate into Docusaurus, generate PDF, and deploy.

**Activities**:
1. Integrate chapters into Docusaurus (`docs/module-01-ros2/`)
2. Add code examples to repository (`examples/ros2-basics/`)
3. Configure Docusaurus sidebar navigation
4. Add diagrams to `static/img/ros2-basics/`
5. Generate PDF export using Docusaurus plugin
6. Final end-to-end test: Clean environment, follow quickstart, run all examples
7. Deploy Docusaurus site to GitHub Pages (preview deployment)
8. Document known limitations and future improvements

**Deliverables**:
- Completed Module 1 content in Docusaurus
- PDF export of Module 1
- GitHub Pages preview URL
- Final validation report: All success criteria met

**Gate**: All success criteria (SC-001 to SC-006) must be validated as passing.

## Follow-Up and Next Steps

### Immediate Next Steps

1. **Run `/sp.tasks`**: Generate testable tasks from this plan
2. **Phase 0 execution**: Begin research and source gathering
3. **Docusaurus setup**: Initialize Docusaurus project (if not already done)

### Future Work (Post-Module 1)

- **Module 2: Gazebo/Unity Simulation**: Build on ROS 2 basics with physics simulation
- **Module 3: NVIDIA Isaac Sim**: Advanced simulation and reinforcement learning
- **Module 4: Vision-Language-Action (VLA)**: AI-to-robot pipeline capstone
- **RAG Chatbot Integration**: Book-level feature, index all modules
- **Book Assembly**: Combine all modules, generate full PDF, final deployment

### Open Questions for User

*None at this time. Plan is ready for review and task generation.*

---

**Plan Status**: ✅ Ready for `/sp.tasks` command
**Constitution Compliance**: ✅ All principles validated
**Next Command**: `/sp.tasks` to generate actionable, testable tasks
