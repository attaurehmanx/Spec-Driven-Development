---
description: "Task list for Todo In-Memory Python Console Application implementation"
---

# Tasks: Todo In-Memory Python Console Application

**Input**: Design documents from `/specs/001-todo-app/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan in src/models/todo.py
- [X] T002 Create src/services/todo_service.py for business logic
- [X] T003 [P] Create src/cli/main.py for console interface
- [X] T004 [P] Initialize Python project with UV and requirements

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T005 Create Todo data model in src/models/todo.py with id, title, description, status
- [X] T006 Create in-memory storage mechanism in src/lib/storage.py
- [X] T007 Implement ID generation system with unique incremental IDs
- [X] T008 Create basic console menu structure in src/cli/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---
## Phase 3: User Story 1 - Add Todo (Priority: P1) 🎯 MVP

**Goal**: Enable users to add a new todo with title and optional description

**Independent Test**: Can be fully tested by adding a todo and verifying it appears in the list with correct properties and status.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T009 [P] [US1] Contract test for add_todo in tests/contract/test_todo_api.py
- [X] T010 [P] [US1] Unit test for Todo model validation in tests/unit/test_todo_model.py

### Implementation for User Story 1

- [X] T011 [P] [US1] Create Todo model with validation in src/models/todo.py
- [X] T012 [US1] Implement add_todo function in src/services/todo_service.py
- [X] T013 [US1] Add console interface for adding todo in src/cli/main.py
- [X] T014 [US1] Implement ID uniqueness and auto-increment in src/lib/storage.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---
## Phase 4: User Story 2 - View Todos (Priority: P2)

**Goal**: Allow users to see all their todos in a list format showing ID, title, description, and status with clear visual indicators

**Independent Test**: Can be fully tested by adding todos and then viewing the complete list with all details displayed properly.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [X] T015 [P] [US2] Contract test for get_all_todos in tests/contract/test_todo_api.py
- [X] T016 [P] [US2] Unit test for todo display formatting in tests/unit/test_display.py

### Implementation for User Story 2

- [X] T017 [P] [US2] Implement get_all_todos function in src/services/todo_service.py
- [X] T018 [US2] Add console interface for viewing todos in src/cli/main.py
- [X] T019 [US2] Format output with clear visual indicators for status in src/cli/main.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---
## Phase 5: User Story 3 - Update Todo (Priority: P3)

**Goal**: Allow users to modify the title or description of an existing todo using its ID

**Independent Test**: Can be fully tested by updating a todo and verifying the changes are reflected when viewing.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [X] T020 [P] [US3] Contract test for update_todo in tests/contract/test_todo_api.py
- [X] T021 [P] [US3] Unit test for update validation in tests/unit/test_todo_service.py

### Implementation for User Story 3

- [X] T022 [P] [US3] Implement update_todo function in src/services/todo_service.py
- [X] T023 [US3] Add console interface for updating todo in src/cli/main.py
- [X] T024 [US3] Handle invalid IDs gracefully in src/services/todo_service.py

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---
## Phase 6: User Story 4 - Delete Todo (Priority: P4)

**Goal**: Allow users to remove a todo from their list using its ID, with confirmation required before deletion

**Independent Test**: Can be fully tested by deleting a todo and verifying it no longer appears in the list.

### Tests for User Story 4 (OPTIONAL - only if tests requested) ⚠️

- [X] T025 [P] [US4] Contract test for delete_todo in tests/contract/test_todo_api.py
- [X] T026 [P] [US4] Unit test for delete confirmation in tests/unit/test_todo_service.py

### Implementation for User Story 4

- [X] T027 [P] [US4] Implement delete_todo function in src/services/todo_service.py
- [X] T028 [US4] Add confirmation prompt in src/cli/main.py
- [X] T029 [US4] Handle invalid IDs gracefully in src/services/todo_service.py

**Checkpoint**: At this point, User Stories 1, 2, 3 AND 4 should all work independently

---
## Phase 7: User Story 5 - Mark Complete/Incomplete (Priority: P5)

**Goal**: Allow users to toggle the completion status of a todo using its ID

**Independent Test**: Can be fully tested by toggling a todo's status and verifying the change is reflected in the list.

### Tests for User Story 5 (OPTIONAL - only if tests requested) ⚠️

- [X] T030 [P] [US5] Contract test for toggle_todo_status in tests/contract/test_todo_api.py
- [X] T031 [P] [US5] Unit test for status toggle in tests/unit/test_todo_service.py

### Implementation for User Story 5

- [X] T032 [P] [US5] Implement toggle_todo_status function in src/services/todo_service.py
- [X] T033 [US5] Add console interface for toggling status in src/cli/main.py
- [X] T034 [US5] Handle invalid IDs gracefully in src/services/todo_service.py

**Checkpoint**: All user stories should now be independently functional

---
[Add more user story phases as needed, following the same pattern]

---
## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T035 [P] Input validation and error handling improvements across all functions
- [X] T036 Console UI improvements and clear messaging
- [X] T037 [P] Add comprehensive error handling for all edge cases
- [X] T038 Run quickstart.md validation to ensure all features work together

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
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3 but should be independently testable
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3/US4 but should be independently testable

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
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
   - Developer E: User Story 5
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