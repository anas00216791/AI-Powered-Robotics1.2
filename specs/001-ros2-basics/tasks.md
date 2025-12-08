# Tasks: ROS 2 Basics Module

**Input**: Design documents from `specs/001-ros2-basics/`
**Prerequisites**: plan.md, spec.md (user stories)

**Tests**: Tests are included as automated validation is specified in the plan for code reproducibility and simulation validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is an educational book project with Docusaurus:
- **Chapter content**: `docs/module-01-ros2/`
- **Code examples**: `examples/ros2-basics/`
- **Tests**: `examples/tests/`
- **Images**: `static/img/ros2-basics/`
- **Planning docs**: `specs/001-ros2-basics/`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Docusaurus project and directory structure per plan.md

- [ ] T001 Create docs/module-01-ros2/ directory for chapter content
- [ ] T002 Create examples/ros2-basics/ directory for code examples
- [ ] T003 [P] Create examples/ros2-basics/urdf/ subdirectory for URDF files
- [ ] T004 [P] Create examples/tests/ directory for automated tests
- [ ] T005 [P] Create static/img/ros2-basics/ directory for diagrams
- [ ] T006 [P] Create specs/001-ros2-basics/contracts/ directory for API contracts
- [ ] T007 Initialize Docusaurus project if not already present (docusaurus.config.js, package.json)
- [ ] T008 [P] Configure sidebars.js to include module-01-ros2 section

---

## Phase 2: Foundational (Research & Shared Resources)

**Purpose**: Gather sources and create shared resources that ALL user stories depend on

**⚠️ CRITICAL**: No chapter work can begin until research phase is complete (Phase 0 from plan.md)

### Research (Phase 0 from plan.md: Days 1-2)

- [ ] T009 Search for peer-reviewed papers on ROS 2 middleware (≥5 sources)
- [ ] T010 [P] Compile list of official ROS 2 documentation pages for Nodes, Topics, Services
- [ ] T011 [P] Document Python rclpy API reference sources
- [ ] T012 [P] Document URDF specification references
- [ ] T013 Create specs/001-ros2-basics/research.md with annotated bibliography
- [ ] T014 Verify ≥50% of sources are peer-reviewed or official documentation
- [ ] T015 Extract key concepts for each chapter from research sources

### Shared Resources

- [ ] T016 Create specs/001-ros2-basics/data-model.md with content outline for all 3 chapters
- [ ] T017 Create specs/001-ros2-basics/quickstart.md with ROS 2 installation and prerequisites
- [ ] T018 [P] Create node-topic-diagram source file (PNG or Mermaid) in static/img/ros2-basics/
- [ ] T019 [P] Create service-diagram source file (PNG or Mermaid) in static/img/ros2-basics/

**Checkpoint**: Research complete, shared resources ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learning ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Deliver Chapter 1 explaining ROS 2 Nodes, Topics, and Services so students understand the core concepts

**Independent Test**: A student can read Chapter 1 and successfully explain the roles of Nodes, Topics, and Services in a ROS 2 system

### Content Development for US1

- [ ] T020 [US1] Draft Chapter 1 section: Introduction to ROS 2 middleware in docs/module-01-ros2/chapter-01-basics.md
- [ ] T021 [US1] Draft Chapter 1 section: What is a ROS 2 Node (definition, purpose, examples)
- [ ] T022 [US1] Draft Chapter 1 section: Topics and Publisher/Subscriber pattern with diagram reference
- [ ] T023 [US1] Draft Chapter 1 section: Services and Request/Response pattern with diagram reference
- [ ] T024 [US1] Draft Chapter 1 section: Comparison of Topics vs Services (when to use each)
- [ ] T025 [US1] Add APA citations to Chapter 1 (≥5 sources, ≥50% peer-reviewed/official)
- [ ] T026 [US1] Add cross-references to official ROS 2 docs for deeper learning

### Quality Validation for US1

- [ ] T027 [US1] Calculate word count for Chapter 1 (should contribute to 3,000-5,000 total)
- [ ] T028 [US1] Verify Flesch-Kincaid readability grade level 10-12 for Chapter 1
- [ ] T029 [US1] Peer review: Technical expert reviews Chapter 1 for accuracy
- [ ] T030 [US1] Address peer review feedback and update Chapter 1

**Checkpoint**: Chapter 1 complete and validated - students can now learn ROS 2 fundamentals

---

## Phase 4: User Story 2 - Implementing Python ROS 2 Agent (Priority: P2)

**Goal**: Deliver Chapter 2 with functional Python code examples so students can write and run ROS 2 nodes using rclpy

**Independent Test**: A student can follow the examples in Chapter 2 to write and run a Python script that publishes or subscribes to a ROS 2 topic

### API Contracts for US2

- [ ] T031 [P] [US2] Document publisher node contract in specs/001-ros2-basics/contracts/publisher-node.md
- [ ] T032 [P] [US2] Document subscriber node contract in specs/001-ros2-basics/contracts/subscriber-node.md
- [ ] T033 [P] [US2] Document service server/client contracts in specs/001-ros2-basics/contracts/service-node.md

### Code Examples for US2

- [ ] T034 [P] [US2] Implement publisher_node.py in examples/ros2-basics/ with inline comments
- [ ] T035 [P] [US2] Implement subscriber_node.py in examples/ros2-basics/ with inline comments
- [ ] T036 [P] [US2] Implement service_server.py in examples/ros2-basics/ with inline comments
- [ ] T037 [P] [US2] Implement service_client.py in examples/ros2-basics/ with inline comments
- [ ] T038 [US2] Add environment specification comments (ROS 2 Humble, Python 3.10+, Ubuntu 22.04) to all Python files
- [ ] T039 [US2] Add attribution comments for any code adapted from official examples

### Tests for US2

- [ ] T040 [P] [US2] Create test_publisher_node.py in examples/tests/ to verify node starts without errors
- [ ] T041 [P] [US2] Create test_subscriber_node.py in examples/tests/ to verify node starts without errors
- [ ] T042 [P] [US2] Create test_service_server.py in examples/tests/ to verify service advertises correctly
- [ ] T043 [P] [US2] Create test_service_client.py in examples/tests/ to verify client calls service successfully
- [ ] T044 [US2] Create test_ros2_basics.py integration test in examples/tests/ to verify pub/sub communication

### Content Development for US2

- [ ] T045 [US2] Draft Chapter 2 section: ROS 2 Python environment setup in docs/module-01-ros2/chapter-02-python.md
- [ ] T046 [US2] Draft Chapter 2 section: Creating a publisher node (with code walkthrough)
- [ ] T047 [US2] Draft Chapter 2 section: Creating a subscriber node (with code walkthrough)
- [ ] T048 [US2] Draft Chapter 2 section: Implementing a service (server and client with code walkthrough)
- [ ] T049 [US2] Draft Chapter 2 section: Running and testing nodes (commands, expected output)
- [ ] T050 [US2] Add APA citations to Chapter 2 (≥5 sources, ≥50% peer-reviewed/official)
- [ ] T051 [US2] Add troubleshooting subsection for common setup issues

### Quality Validation for US2

- [ ] T052 [US2] Run automated Python syntax checks on all code examples
- [ ] T053 [US2] Run automated tests (T040-T044) in fresh ROS 2 Humble environment
- [ ] T054 [US2] Manual validation: Human tester runs all examples and verifies expected output
- [ ] T055 [US2] Calculate word count for Chapter 2 (should contribute to 3,000-5,000 total)
- [ ] T056 [US2] Verify Flesch-Kincaid readability grade level 10-12 for Chapter 2
- [ ] T057 [US2] Peer review: Technical expert reviews Chapter 2 and code for accuracy
- [ ] T058 [US2] Address peer review feedback and update Chapter 2 and code

**Checkpoint**: Chapter 2 complete and validated - students can now write and run Python ROS 2 nodes

---

## Phase 5: User Story 3 - Modeling Robot with URDF (Priority: P3)

**Goal**: Deliver Chapter 3 with a URDF model example so students can create and visualize humanoid robot structures

**Independent Test**: A student can use the instructions in Chapter 3 to create a valid URDF file that can be visualized in Rviz

### URDF Model Development for US3

- [ ] T059 [US3] Create simple_humanoid.urdf in examples/ros2-basics/urdf/ with base_link (torso)
- [ ] T060 [US3] Add head link with fixed joint to base_link in simple_humanoid.urdf
- [ ] T061 [US3] Add left_arm and right_arm links with revolute joints in simple_humanoid.urdf
- [ ] T062 [US3] Add left_leg and right_leg links with revolute joints in simple_humanoid.urdf
- [ ] T063 [US3] Add visual properties (geometry, colors) to all links in simple_humanoid.urdf
- [ ] T064 [US3] Create launch_rviz.launch.py in examples/ros2-basics/urdf/ to launch robot_state_publisher and Rviz

### Tests for US3

- [ ] T065 [US3] Validate simple_humanoid.urdf using check_urdf command (must pass)
- [ ] T066 [US3] Manual test: Launch launch_rviz.launch.py and verify robot model appears in Rviz
- [ ] T067 [US3] Create test_urdf_validity.py in examples/tests/ to automate check_urdf validation
- [ ] T068 [US3] Document Rviz validation results (screenshots) in specs/001-ros2-basics/

### Content Development for US3

- [ ] T069 [US3] Draft Chapter 3 section: Introduction to URDF syntax in docs/module-01-ros2/chapter-03-urdf.md
- [ ] T070 [US3] Draft Chapter 3 section: Defining links (robot body parts) with examples
- [ ] T071 [US3] Draft Chapter 3 section: Defining joints (connections between links) with examples
- [ ] T072 [US3] Draft Chapter 3 section: Creating simple humanoid model (step-by-step walkthrough)
- [ ] T073 [US3] Draft Chapter 3 section: Validating URDF with check_urdf (commands, expected output)
- [ ] T074 [US3] Draft Chapter 3 section: Visualizing in Rviz (commands, expected output)
- [ ] T075 [US3] Add APA citations to Chapter 3 (≥5 sources, ≥50% peer-reviewed/official)
- [ ] T076 [US3] Add troubleshooting subsection for common URDF errors

### Quality Validation for US3

- [ ] T077 [US3] Run automated URDF validation test (T067)
- [ ] T078 [US3] Manual validation: Human tester follows Chapter 3 and creates URDF from scratch
- [ ] T079 [US3] Calculate word count for Chapter 3 (should contribute to 3,000-5,000 total)
- [ ] T080 [US3] Verify Flesch-Kincaid readability grade level 10-12 for Chapter 3
- [ ] T081 [US3] Peer review: Technical expert reviews Chapter 3 and URDF for accuracy
- [ ] T082 [US3] Address peer review feedback and update Chapter 3 and URDF

**Checkpoint**: Chapter 3 complete and validated - students can now create and visualize URDF models

---

## Phase 6: Module Integration & Final Validation

**Purpose**: Integrate all chapters into Docusaurus, perform final quality checks, and deploy

### Docusaurus Integration

- [ ] T083 Create docs/module-01-ros2/index.md as module landing page with overview
- [ ] T084 Update docusaurus.config.js to include module-01-ros2 in navigation
- [ ] T085 Update sidebars.js to list all three chapters in correct order
- [ ] T086 [P] Add all diagrams from static/img/ros2-basics/ to chapter content with proper references
- [ ] T087 [P] Add code example file references with links to GitHub repository
- [ ] T088 Test Docusaurus build locally (npm run build)

### Final Quality Checks

- [ ] T089 Calculate total module word count (must be 3,000-5,000 words)
- [ ] T090 Citation audit: Verify all sources are in APA format
- [ ] T091 Citation audit: Verify ≥50% are peer-reviewed or official docs
- [ ] T092 Run all automated tests (T040-T044, T067) and verify pass
- [ ] T093 Final manual end-to-end test: Fresh ROS 2 environment, follow all chapters, run all examples
- [ ] T094 Final peer review: Technical expert reviews entire module for accuracy and completeness
- [ ] T095 Address final feedback and make any necessary corrections

### Deployment Preparation

- [ ] T096 Generate PDF export of Module 1 using Docusaurus plugin
- [ ] T097 Deploy Docusaurus site to GitHub Pages (preview deployment)
- [ ] T098 Verify all links and images work correctly on deployed site
- [ ] T099 Create final validation report documenting all success criteria (SC-001 to SC-006)
- [ ] T100 Document known limitations and future improvements

**Checkpoint**: Module 1 complete, deployed, and ready for student use

---

## Phase 7: Polish & Documentation

**Purpose**: Final improvements and comprehensive documentation

- [ ] T101 [P] Add README.md to examples/ros2-basics/ with usage instructions
- [ ] T102 [P] Add README.md to examples/ros2-basics/urdf/ with URDF usage instructions
- [ ] T103 [P] Add comments and docstrings to launch_rviz.launch.py
- [ ] T104 Update specs/001-ros2-basics/quickstart.md based on testing feedback
- [ ] T105 [P] Create CONTRIBUTING.md guide for future module contributors
- [ ] T106 Code cleanup: Ensure consistent formatting across all Python files
- [ ] T107 Code cleanup: Ensure consistent formatting across all Markdown files
- [ ] T108 [P] Performance check: Verify Docusaurus site loads in <2 seconds
- [ ] T109 [P] Performance check: Verify PDF export is <10 MB
- [ ] T110 Create migration guide for future ROS 2 version updates

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
  - Research (T009-T015) MUST complete before any chapter work
  - Shared resources (T016-T019) should complete before chapter work
- **User Story 1 (Phase 3)**: Can start after Foundational Phase 2 - No dependencies on other stories
- **User Story 2 (Phase 4)**: Can start after Foundational Phase 2 - Independent of US1 but references Chapter 1 concepts
- **User Story 3 (Phase 5)**: Can start after Foundational Phase 2 - Independent of US1/US2
- **Integration (Phase 6)**: Depends on completion of US1, US2, US3
- **Polish (Phase 7)**: Depends on Integration completion

### User Story Independence

Each user story (chapter) is designed to be independently implementable and testable:

- **US1 (Chapter 1)**: Theory-only, no code dependencies
- **US2 (Chapter 2)**: Python examples, references Chapter 1 concepts but can be drafted in parallel
- **US3 (Chapter 3)**: URDF examples, references ROS 2 concepts but can be drafted in parallel

### Task Dependencies Within User Stories

**US1 Dependencies:**
- Research (T009-T015) must complete first
- Content drafting (T020-T026) can proceed in any order
- Quality validation (T027-T030) depends on content completion

**US2 Dependencies:**
- Research (T009-T015) must complete first
- Contracts (T031-T033) should complete before code (helpful but not blocking)
- Code examples (T034-T039) are parallelizable (different files)
- Tests (T040-T044) can be written in parallel with or after code
- Content (T045-T051) references code, so code should exist first
- Quality validation (T052-T058) depends on code and content completion

**US3 Dependencies:**
- Research (T009-T015) must complete first
- URDF model (T059-T063) must build sequentially (base → limbs → visuals)
- Launch file (T064) depends on URDF completion
- Tests (T065-T068) depend on URDF and launch file
- Content (T069-T076) references URDF, so URDF should exist first
- Quality validation (T077-T082) depends on URDF and content completion

### Parallel Opportunities

**Phase 1 (Setup)**: All tasks (T001-T008) can run in parallel - different directories

**Phase 2 (Foundational)**:
- Research tasks T010, T011, T012 can run in parallel (different sources)
- Shared resources T018, T019 can run in parallel (different diagrams)

**Phase 3 (US1)**: Content sections T020-T024 can be drafted in parallel by different writers

**Phase 4 (US2)**:
- Contracts T031, T032, T033 can run in parallel
- Code examples T034, T035, T036, T037 can run in parallel (different files)
- Tests T040, T041, T042, T043 can run in parallel (different files)

**Phase 5 (US3)**: Minimal parallelization due to sequential URDF building

**Phase 6 (Integration)**: Tasks T086, T087 can run in parallel

**Phase 7 (Polish)**: Most tasks can run in parallel (different files)

**Cross-Story Parallelization**: With multiple team members:
- After Phase 2 completes, US1, US2, US3 can be worked on in parallel
- Developer A: US1 (Chapter 1)
- Developer B: US2 (Chapter 2 + code)
- Developer C: US3 (Chapter 3 + URDF)

---

## Parallel Example: User Story 2

```bash
# Launch all API contracts together (Phase 4):
Task T031: "Document publisher node contract in specs/001-ros2-basics/contracts/publisher-node.md"
Task T032: "Document subscriber node contract in specs/001-ros2-basics/contracts/subscriber-node.md"
Task T033: "Document service server/client contracts in specs/001-ros2-basics/contracts/service-node.md"

# Launch all code examples together (Phase 4):
Task T034: "Implement publisher_node.py in examples/ros2-basics/"
Task T035: "Implement subscriber_node.py in examples/ros2-basics/"
Task T036: "Implement service_server.py in examples/ros2-basics/"
Task T037: "Implement service_client.py in examples/ros2-basics/"

# Launch all tests together (Phase 4):
Task T040: "Create test_publisher_node.py in examples/tests/"
Task T041: "Create test_subscriber_node.py in examples/tests/"
Task T042: "Create test_service_server.py in examples/tests/"
Task T043: "Create test_service_client.py in examples/tests/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational Research (T009-T019) - CRITICAL
3. Complete Phase 3: User Story 1 / Chapter 1 (T020-T030)
4. **STOP and VALIDATE**: Test that Chapter 1 explains ROS 2 concepts clearly
5. Optional: Deploy Chapter 1 only for early feedback

**MVP Deliverable**: Chapter 1 (ROS 2 Fundamentals) - students can learn theory

### Incremental Delivery

1. **Foundation Ready**: Complete Setup + Research → Research complete, ready for chapters
2. **MVP (Chapter 1)**: Add US1 → Test independently → Students can learn ROS 2 concepts
3. **v0.2 (Chapter 1 + 2)**: Add US2 → Test independently → Students can learn + code ROS 2 nodes
4. **v1.0 (Full Module)**: Add US3 → Test independently → Students can learn + code + model robots
5. **v1.1 (Polished)**: Add integration + polish → Production-ready module

Each increment adds value without breaking previous chapters.

### Parallel Team Strategy

With multiple team members:

1. **Week 1, Days 1-2**: Team completes Setup + Research together (T001-T019)
2. **Week 1, Days 3-7**: Once research is done:
   - **Writer A**: US1 / Chapter 1 (T020-T030)
   - **Developer B**: US2 / Chapter 2 + code (T031-T058)
   - **Developer C**: US3 / Chapter 3 + URDF (T059-T082)
3. **Week 2, Days 8-14**: Integration, testing, deployment (T083-T110)
   - Team works together on integration and final validation

### Sequential Strategy (Single Person)

Follow priorities strictly:

1. **Days 1-2**: Phase 1 + Phase 2 (Setup + Research)
2. **Days 3-5**: Phase 3 (US1 / Chapter 1)
3. **Days 6-9**: Phase 4 (US2 / Chapter 2 + code)
4. **Days 10-12**: Phase 5 (US3 / Chapter 3 + URDF)
5. **Days 13-14**: Phase 6 + Phase 7 (Integration + Polish)

---

## Notes

- **[P] tasks** = Parallelizable (different files, no dependencies)
- **[Story] labels** = Map tasks to user stories for traceability
  - [US1] = Chapter 1 (ROS 2 Fundamentals)
  - [US2] = Chapter 2 (Python/rclpy)
  - [US3] = Chapter 3 (URDF)
- **Each chapter is independently testable**: Can validate each chapter's learning outcomes separately
- **Commit strategy**: Commit after each task or logical group
- **Checkpoints**: Stop at any checkpoint to validate chapter independently
- **Quality gates**: Research must pass ≥50% peer-reviewed sources check before chapter work
- **Validation protocol**: All code must pass automated tests before peer review
- **Citation compliance**: APA format enforced throughout, checked at T090-T091

---

## Task Count Summary

- **Total Tasks**: 110
- **Setup (Phase 1)**: 8 tasks
- **Foundational (Phase 2)**: 11 tasks (research + shared resources)
- **User Story 1 (Phase 3)**: 11 tasks (Chapter 1 content + validation)
- **User Story 2 (Phase 4)**: 28 tasks (contracts + code + tests + Chapter 2 + validation)
- **User Story 3 (Phase 5)**: 24 tasks (URDF + tests + Chapter 3 + validation)
- **Integration (Phase 6)**: 18 tasks (Docusaurus + final validation + deployment)
- **Polish (Phase 7)**: 10 tasks (documentation + cleanup)

**Parallel Opportunities**: 35 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 = 30 tasks (Chapter 1 only)
