---
description: "Task list for RAG Agent and Backend Service implementation"
---

# Tasks: RAG Agent and Backend Service

**Input**: Design documents from `/specs/001-rag-agent-service/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend project**: `backend/src/`, `backend/tests/` at repository root
- **API service**: `backend/api/`, `backend/models/`, `backend/services/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure with backend directory
- [x] T002 Initialize Python project with FastAPI, OpenAI, Qdrant, and asyncpg dependencies
- [x] T003 [P] Configure linting and formatting tools (black, flake8, mypy)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup database schema and migrations framework for Neon Postgres
- [x] T005 [P] Configure environment variables management for OpenAI, Qdrant, and Neon credentials
- [x] T006 [P] Setup API routing and middleware structure in backend/main.py
- [x] T007 Create base models/entities that all stories depend on
- [x] T008 Configure error handling and logging infrastructure
- [x] T009 Setup Qdrant client connection framework

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content and receive accurate answers based on book information

**Independent Test**: Submit a question to the chat endpoint and verify that the response contains information from the book content

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T010 [P] [US1] Contract test for POST /chat endpoint in backend/tests/test_api.py
- [x] T011 [P] [US1] Integration test for RAG question-answering flow in backend/tests/test_rag.py

### Implementation for User Story 1

- [x] T012 [P] [US1] Create Conversation model in backend/src/models/conversation.py
- [x] T013 [P] [US1] Create Message model in backend/src/models/message.py
- [x] T014 [US1] Implement OpenAI Agent initialization in backend/src/services/agent_service.py
- [x] T015 [US1] Implement Qdrant retrieval integration in backend/src/services/retrieval_service.py
- [x] T016 [US1] Create chat endpoint in backend/src/api/chat.py
- [x] T017 [US1] Add validation and error handling for RAG responses
- [x] T018 [US1] Connect agent to Qdrant for book content retrieval

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Selected Text Constrained Responses (Priority: P2)

**Goal**: Allow users to provide specific text selections and receive responses constrained only to that text

**Independent Test**: Provide user-selected text with a question and verify that the response is based only on the provided text rather than the entire book collection

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [x] T019 [P] [US2] Contract test for selected-text-only mode in backend/tests/test_api.py
- [x] T020 [P] [US2] Integration test for selected-text constrained responses in backend/tests/test_rag.py

### Implementation for User Story 2

- [x] T021 [P] [US2] Extend Message model to support selected text metadata in backend/src/models/message.py
- [x] T022 [US2] Implement selected-text-only mode in backend/src/services/agent_service.py
- [x] T023 [US2] Update chat endpoint to accept optional selected_text parameter in backend/src/api/chat.py
- [x] T024 [US2] Add validation for selected-text mode responses

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Conversation History Management (Priority: P3)

**Goal**: Preserve conversation history across interactions for contextual conversations

**Independent Test**: Conduct a multi-turn conversation and verify that the system remembers previous questions and responses in the conversation context

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [x] T025 [P] [US3] Contract test for conversation persistence in backend/tests/test_api.py
- [x] T026 [P] [US3] Integration test for multi-turn conversations in backend/tests/test_conversation.py

### Implementation for User Story 3

- [x] T027 [P] [US3] Implement conversation persistence in backend/src/services/conversation_service.py
- [x] T028 [US3] Update chat endpoint to maintain conversation context in backend/src/api/chat.py
- [x] T029 [US3] Add database operations for conversation storage in backend/src/db/conversation_db.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T030 [P] Documentation updates in docs/
- [x] T031 Code cleanup and refactoring
- [x] T032 Performance optimization across all stories
- [x] T033 [P] Additional unit tests (if requested) in backend/tests/unit/
- [x] T034 Security hardening
- [x] T035 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for POST /chat endpoint in backend/tests/test_api.py"
Task: "Integration test for RAG question-answering flow in backend/tests/test_rag.py"

# Launch all models for User Story 1 together:
Task: "Create Conversation model in backend/src/models/conversation.py"
Task: "Create Message model in backend/src/models/message.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence