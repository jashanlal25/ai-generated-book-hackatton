# Tasks: ROS 2 Fundamentals for Humanoid Robotics (Module 1)

**Input**: Design documents from `/specs/001-ros2-humanoid-module1/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested. Manual validation steps are included instead.

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `my-website/docs/module-1-ros2-fundamentals/`
- **Code examples**: `my-website/static/code-examples/module-1/`
- **Docusaurus config**: `my-website/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and Docusaurus module structure

- [ ] T001 Create module directory at my-website/docs/module-1-ros2-fundamentals/
- [ ] T002 Create category configuration at my-website/docs/module-1-ros2-fundamentals/_category_.json
- [ ] T003 [P] Create code examples directory at my-website/static/code-examples/module-1/
- [ ] T004 [P] Verify Docusaurus build succeeds with new module structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before chapter content

**CRITICAL**: No chapter writing can begin until this phase is complete

- [ ] T005 Review research.md and verify all ROS 2 Humble APIs are accurate
- [ ] T006 [P] Confirm chapter outlines in contracts/ are complete and accurate
- [ ] T007 [P] Verify APA citation format requirements from constitution.md
- [ ] T008 Create references section template for citations at end of each chapter

**Checkpoint**: Foundation ready - chapter implementation can now begin

---

## Phase 3: User Story 1 - Learn ROS 2 Core Concepts (Priority: P1)

**Goal**: Student reads Chapter 1 and understands ROS 2 middleware, nodes, topics, services, QoS

**Independent Test**: Student answers 4/5 concept-check questions correctly after reading

### Implementation for User Story 1

- [ ] T009 [US1] Create chapter file at my-website/docs/module-1-ros2-fundamentals/chapter-1-ros2-overview.mdx
- [ ] T010 [US1] Write Section 1.1: Introduction - Why ROS 2? (150-200 words) in chapter-1-ros2-overview.mdx
- [ ] T011 [US1] Write Section 1.2: The ROS 2 Graph - Nodes and Communication (300-350 words) in chapter-1-ros2-overview.mdx
- [ ] T012 [US1] Write Section 1.3: Topics - Publish-Subscribe Pattern (300-350 words) in chapter-1-ros2-overview.mdx
- [ ] T013 [US1] Create ASCII diagram: ROS 2 Publish-Subscribe Architecture in chapter-1-ros2-overview.mdx
- [ ] T014 [US1] Write Section 1.4: Services - Request-Response Communication (200-250 words) in chapter-1-ros2-overview.mdx
- [ ] T015 [US1] Write Section 1.5: Quality of Service (QoS) (200-250 words) in chapter-1-ros2-overview.mdx
- [ ] T016 [US1] Write Section 1.6: Summary and Key Takeaways (100-150 words) in chapter-1-ros2-overview.mdx
- [ ] T017 [US1] Add 5 self-check questions at end of chapter-1-ros2-overview.mdx
- [ ] T018 [US1] Add RAG chatbot Q&A markers for key concepts (Node, Topic, Service, QoS, DDS) in chapter-1-ros2-overview.mdx
- [ ] T019 [US1] Verify word count is 1,200-1,400 for chapter-1-ros2-overview.mdx
- [ ] T020 [US1] Verify Flesch-Kincaid grade level 11-13 for chapter-1-ros2-overview.mdx
- [ ] T021 [US1] Verify Docusaurus renders chapter-1-ros2-overview.mdx correctly

**Checkpoint**: User Story 1 complete - Chapter 1 is readable and passes all acceptance criteria

---

## Phase 4: User Story 2 - Build Working ROS 2 Nodes with Python (Priority: P2)

**Goal**: Student creates functional publisher, subscriber, and service nodes using rclpy

**Independent Test**: Student runs publisher-subscriber pair and observes message flow

### Code Examples for User Story 2

- [ ] T022 [P] [US2] Create publisher.py at my-website/static/code-examples/module-1/publisher.py
- [ ] T023 [P] [US2] Create subscriber.py at my-website/static/code-examples/module-1/subscriber.py
- [ ] T024 [P] [US2] Create service_server.py at my-website/static/code-examples/module-1/service_server.py
- [ ] T025 [P] [US2] Create service_client.py at my-website/static/code-examples/module-1/service_client.py

### Implementation for User Story 2

- [ ] T026 [US2] Create chapter file at my-website/docs/module-1-ros2-fundamentals/chapter-2-rclpy-nodes.mdx
- [ ] T027 [US2] Write Section 2.1: Introduction to rclpy (150-200 words) in chapter-2-rclpy-nodes.mdx
- [ ] T028 [US2] Write Section 2.2: Creating a Publisher Node (300-400 words) with embedded code in chapter-2-rclpy-nodes.mdx
- [ ] T029 [US2] Write Section 2.3: Creating a Subscriber Node (250-350 words) with embedded code in chapter-2-rclpy-nodes.mdx
- [ ] T030 [US2] Write Section 2.4: Running Publisher and Subscriber Together (150-200 words) in chapter-2-rclpy-nodes.mdx
- [ ] T031 [US2] Create ASCII diagram: Humanoid Joint State Message Flow in chapter-2-rclpy-nodes.mdx
- [ ] T032 [US2] Write Section 2.5: Creating a Service Server (250-350 words) with embedded code in chapter-2-rclpy-nodes.mdx
- [ ] T033 [US2] Write Section 2.6: Creating a Service Client (200-300 words) with embedded code in chapter-2-rclpy-nodes.mdx
- [ ] T034 [US2] Write Section 2.7: Summary and Next Steps (100-150 words) in chapter-2-rclpy-nodes.mdx
- [ ] T035 [US2] Add RAG chatbot Q&A markers for key concepts (rclpy, Publisher, Subscriber, Callback) in chapter-2-rclpy-nodes.mdx
- [ ] T036 [US2] Verify word count is 1,400-1,800 for chapter-2-rclpy-nodes.mdx
- [ ] T037 [US2] Verify Flesch-Kincaid grade level 11-13 for chapter-2-rclpy-nodes.mdx

### Validation for User Story 2

- [ ] T038 [US2] Test publisher.py executes on ROS 2 Humble without errors
- [ ] T039 [US2] Test subscriber.py executes on ROS 2 Humble without errors
- [ ] T040 [US2] Test publisher-subscriber pair exchanges messages correctly
- [ ] T041 [US2] Test service_server.py executes on ROS 2 Humble without errors
- [ ] T042 [US2] Test service_client.py executes and receives response from server
- [ ] T043 [US2] Verify Docusaurus renders chapter-2-rclpy-nodes.mdx correctly

**Checkpoint**: User Story 2 complete - Chapter 2 is readable and all code examples execute

---

## Phase 5: User Story 3 - Describe Humanoid Robots with URDF (Priority: P3)

**Goal**: Student defines robot physical structure using URDF elements

**Independent Test**: Student creates minimal URDF file that passes check_urdf validation

### Code Examples for User Story 3

- [ ] T044 [P] [US3] Create humanoid_torso.urdf at my-website/static/code-examples/module-1/humanoid_torso.urdf

### Implementation for User Story 3

- [ ] T045 [US3] Create chapter file at my-website/docs/module-1-ros2-fundamentals/chapter-3-urdf-basics.mdx
- [ ] T046 [US3] Write Section 3.1: Introduction to Robot Description (150-200 words) in chapter-3-urdf-basics.mdx
- [ ] T047 [US3] Write Section 3.2: Links - The Building Blocks (250-300 words) in chapter-3-urdf-basics.mdx
- [ ] T048 [US3] Write Section 3.3: Joints - Connecting the Parts (300-350 words) with joint types table in chapter-3-urdf-basics.mdx
- [ ] T049 [US3] Write Section 3.4: Building a Humanoid Torso (350-450 words) with embedded URDF in chapter-3-urdf-basics.mdx
- [ ] T050 [US3] Create ASCII diagram: Humanoid Torso URDF Tree in chapter-3-urdf-basics.mdx
- [ ] T051 [US3] Write Section 3.5: Validating Your URDF (150-200 words) with commands in chapter-3-urdf-basics.mdx
- [ ] T052 [US3] Write Section 3.6: Summary and Extensions (100-150 words) in chapter-3-urdf-basics.mdx
- [ ] T053 [US3] Add RAG chatbot Q&A markers for key concepts (URDF, Link, Joint, Visual, Collision) in chapter-3-urdf-basics.mdx
- [ ] T054 [US3] Verify word count is 1,200-1,600 for chapter-3-urdf-basics.mdx
- [ ] T055 [US3] Verify Flesch-Kincaid grade level 11-13 for chapter-3-urdf-basics.mdx

### Validation for User Story 3

- [ ] T056 [US3] Validate humanoid_torso.urdf passes check_urdf tool
- [ ] T057 [US3] Validate humanoid_torso.urdf passes XML syntax validation
- [ ] T058 [US3] Verify Docusaurus renders chapter-3-urdf-basics.mdx correctly

**Checkpoint**: User Story 3 complete - Chapter 3 is readable and URDF validates

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and quality checks across all chapters

- [ ] T059 [P] Verify all 3 chapters are linked correctly in Docusaurus sidebar
- [ ] T060 [P] Run full Docusaurus build and verify no errors
- [ ] T061 [P] Verify chapter progression is logical (1 → 2 → 3)
- [ ] T062 Verify all technical claims are traceable to ROS 2 Humble documentation
- [ ] T063 Add APA citations for any external references used
- [ ] T064 Run plagiarism check (target: 0%)
- [ ] T065 Create module introduction/landing page at my-website/docs/module-1-ros2-fundamentals/index.mdx
- [ ] T066 Run quickstart.md validation checklist
- [ ] T067 Final review: All acceptance criteria from spec.md satisfied

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all chapters
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - Can proceed in priority order (P1 → P2 → P3)
  - Or in parallel if multiple authors available
- **Polish (Phase 6)**: Depends on all chapters being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational - References concepts from US1 but independently testable
- **User Story 3 (P3)**: Can start after Foundational - References concepts from US1 but independently testable

### Within Each User Story

- Code examples (if any) can be created in parallel [P]
- Chapter sections should be written sequentially (flow/coherence)
- Validation tasks depend on content being complete

### Parallel Opportunities

- T001-T004: All setup tasks can run in parallel
- T005-T008: Foundational verification tasks can run in parallel
- T022-T025: All Chapter 2 code examples can be written in parallel
- T044: Chapter 3 URDF can be written while Chapter 2 is in progress
- T059-T064: Many polish tasks can run in parallel

---

## Parallel Example: User Story 2 Code Examples

```bash
# Launch all code examples for User Story 2 together:
Task: "Create publisher.py at my-website/static/code-examples/module-1/publisher.py"
Task: "Create subscriber.py at my-website/static/code-examples/module-1/subscriber.py"
Task: "Create service_server.py at my-website/static/code-examples/module-1/service_server.py"
Task: "Create service_client.py at my-website/static/code-examples/module-1/service_client.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Chapter 1)
4. **STOP and VALIDATE**: Chapter 1 readable, concepts clear
5. Deploy/demo Module 1 with Chapter 1 only

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add Chapter 1 (US1) → Validate → Deploy (MVP!)
3. Add Chapter 2 (US2) + code examples → Validate → Deploy
4. Add Chapter 3 (US3) + URDF → Validate → Deploy
5. Polish → Final release

### RAG Chatbot Integration Points

Each chapter includes markers for RAG chatbot Q&A:
- Chapter 1: Node, Topic, Service, QoS, DDS definitions
- Chapter 2: rclpy, Publisher, Subscriber, Callback patterns
- Chapter 3: URDF, Link, Joint, Visual, Collision elements

These markers enable the RAG chatbot to answer student questions about specific concepts.

---

## Summary

| Phase | Task Count | Parallel Tasks |
|-------|------------|----------------|
| Setup (Phase 1) | 4 | 2 |
| Foundational (Phase 2) | 4 | 2 |
| User Story 1 (Phase 3) | 13 | 0 |
| User Story 2 (Phase 4) | 22 | 4 |
| User Story 3 (Phase 5) | 15 | 1 |
| Polish (Phase 6) | 9 | 3 |
| **Total** | **67** | **12** |

### MVP Scope

**Minimum Viable Product**: Complete Phases 1-3 (Setup + Foundational + User Story 1)
- Total tasks for MVP: 21 tasks
- Delivers: Chapter 1 with ROS 2 core concepts
- Independent test: Student can answer concept questions

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each chapter should be independently completable and testable
- Commit after each section completion
- Stop at any checkpoint to validate story independently
- Avoid: incomplete code examples, word count violations, broken Docusaurus builds
