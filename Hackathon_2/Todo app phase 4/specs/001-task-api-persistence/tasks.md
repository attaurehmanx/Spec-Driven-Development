---
description: "Task list for Task API & Persistence Layer feature implementation"
---

# Tasks: Task API & Persistence Layer

**Input**: Design documents from `/specs/001-task-api-persistence/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure per implementation plan in backend/
- [x] T002 Initialize Python project with FastAPI, SQLModel, and PostgreSQL dependencies in backend/
- [x] T003 [P] Configure linting and formatting tools (flake8, black, mypy) in backend/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup database schema and migrations framework in backend/src/database/
- [x] T005 [P] Create base model configuration in backend/src/models/base.py
- [x] T006 [P] Setup database engine configuration in backend/src/database/engine.py
- [x] T007 Create database session management in backend/src/database/session.py
- [x] T008 [P] Configure authentication utilities using JWT in backend/src/utils/auth.py
- [x] T009 Setup validation utilities in backend/src/utils/validation.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Creates New Task (Priority: P1) 🎯 MVP

**Goal**: Enable authenticated users to create new tasks linked to their user ID with all details intact

**Independent Test**: Create a new task and verify it is stored in the database and can be retrieved, delivering the core value of allowing users to store their tasks.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests first, ensure they fail before implementation**

- [ ] T010 [P] [US1] Contract test for POST /api/{user_id}/tasks endpoint in backend/tests/contract/test_tasks.py
- [ ] T011 [P] [US1] Integration test for task creation flow in backend/tests/integration/test_task_creation.py

### Implementation for User Story 1

- [x] T012 [P] [US1] Create Task model in backend/src/models/task.py
- [x] T013 [US1] Implement TaskService in backend/src/services/task_service.py (depends on T012)
- [x] T014 [US1] Implement task creation endpoint in backend/src/api/routers/tasks.py
- [x] T015 [US1] Add dependency injection for authentication in backend/src/api/deps.py
- [x] T016 [US1] Add validation and error handling for task creation
- [x] T017 [US1] Add logging for task creation operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - User Views Their Task List (Priority: P1)

**Goal**: Allow authenticated users to retrieve only their tasks, ensuring no cross-user data leakage

**Independent Test**: Create multiple tasks for a user and retrieve the list, verifying only that user's tasks are returned, delivering the core value of task visibility.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T018 [P] [US2] Contract test for GET /api/{user_id}/tasks endpoint in backend/tests/contract/test_tasks.py
- [ ] T019 [P] [US2] Integration test for task list retrieval in backend/tests/integration/test_task_list.py

### Implementation for User Story 2

- [x] T020 [P] [US2] Add task listing method to TaskService in backend/src/services/task_service.py
- [x] T021 [US2] Implement task list endpoint in backend/src/api/routers/tasks.py
- [x] T022 [US2] Add user-specific filtering to task queries in backend/src/services/task_service.py
- [x] T023 [US2] Add pagination support if needed for task list endpoint

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User Manages Individual Tasks (Priority: P2)

**Goal**: Enable authenticated users to interact with specific tasks by viewing, updating, marking complete, or deleting, with proper ownership validation

**Independent Test**: Perform individual task operations (retrieve, update, toggle completion, delete) and verify they work correctly for the authenticated user's tasks, delivering comprehensive task management value.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T024 [P] [US3] Contract test for GET /api/{user_id}/tasks/{id} endpoint in backend/tests/contract/test_tasks.py
- [ ] T025 [P] [US3] Contract test for PATCH /api/{user_id}/tasks/{id}/complete endpoint in backend/tests/contract/test_tasks.py
- [ ] T026 [P] [US3] Integration test for individual task operations in backend/tests/integration/test_individual_tasks.py

### Implementation for User Story 3

- [x] T027 [P] [US3] Add individual task retrieval method to TaskService in backend/src/services/task_service.py
- [x] T028 [US3] Implement task retrieval endpoint in backend/src/api/routers/tasks.py
- [x] T029 [US3] Implement task update endpoint in backend/src/api/routers/tasks.py
- [x] T030 [US3] Implement task deletion endpoint in backend/src/api/routers/tasks.py
- [x] T031 [US3] Implement task completion toggle endpoint in backend/src/api/routers/tasks.py
- [x] T032 [US3] Add ownership validation for all individual task operations in backend/src/services/task_service.py
- [x] T033 [US3] Add 403 Forbidden response for cross-user access attempts

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T034 [P] Documentation updates in docs/task-api.md
- [x] T035 Add comprehensive error handling for all endpoints
- [x] T036 Performance optimization for user-scoped queries
- [x] T037 [P] Additional unit tests in backend/tests/unit/test_task_models.py
- [x] T038 Security hardening for user isolation
- [x] T039 Run quickstart.md validation to ensure all flows work correctly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P1 → P2)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

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
Task: "Contract test for POST /api/{user_id}/tasks endpoint in backend/tests/contract/test_tasks.py"
Task: "Integration test for task creation flow in backend/tests/integration/test_task_creation.py"

# Launch all models for User Story 1 together:
Task: "Create Task model in backend/src/models/task.py"
Task: "Implement TaskService in backend/src/services/task_service.py"
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