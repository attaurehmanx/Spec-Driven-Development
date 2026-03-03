---
description: "Task list for MCP Task Tools feature implementation"
---

# Tasks: MCP Task Tools

**Input**: Design documents from `/specs/004-mcp-task-tools/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT requested in the feature specification, so no test tasks are included. Manual testing with MCP Inspector will be used for validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **MCP Server**: `backend/mcp_server/`, `backend/mcp_server/tools/`
- **Database**: `backend/database/session.py`
- Paths follow existing backend structure from Phase 2

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install dependencies and create project structure

- [x] T001 Install MCP SDK and async PostgreSQL driver: `pip install "mcp[cli]" asyncpg pytest-asyncio`
- [x] T002 Update backend/requirements.txt with new dependencies (mcp[cli], asyncpg, pytest-asyncio)
- [x] T003 Create MCP server directory structure: backend/mcp_server/ with subdirectories tools/ and tests/
- [x] T004 Create __init__.py files in backend/mcp_server/, backend/mcp_server/tools/, backend/mcp_server/tests/
- [x] T005 Verify existing Task and User models in backend/models/task_models.py are accessible

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Modify backend/database/session.py to add async engine creation with asyncpg driver
- [x] T007 Add async_session_maker to backend/database/session.py using sessionmaker with AsyncSession
- [x] T008 Add get_async_session() function to backend/database/session.py for dependency injection
- [x] T009 Create backend/mcp_server/server.py with FastMCP instance and lifespan management
- [x] T010 Create backend/mcp_server/run_mcp.py as entry point script for MCP server
- [x] T011 [P] Define custom exception classes in backend/mcp_server/tools/task_tools.py (TaskToolError, TaskNotFoundError, UnauthorizedTaskAccessError, ValidationError)
- [x] T012 Test async database connection by running a simple query with get_async_session()
- [x] T013 Verify MCP server starts without errors: `python backend/mcp_server/run_mcp.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - AI Creates Tasks (Priority: P1) 🎯 MVP

**Goal**: Enable AI to create new tasks for users with title and optional description

**Independent Test**: Create a task via MCP Inspector, verify it appears in database with correct user ownership

### Implementation for User Story 1

- [x] T014 [US1] Implement add_task tool function signature in backend/mcp_server/tools/task_tools.py with @mcp.tool() decorator
- [x] T015 [US1] Add user validation logic to add_task: verify user exists in database
- [x] T016 [US1] Add title validation to add_task: non-empty, max 255 characters
- [x] T017 [US1] Add description validation to add_task: optional, max 10,000 characters
- [x] T018 [US1] Implement task creation logic in add_task: create Task entity with user_id, title, description
- [x] T019 [US1] Implement database persistence in add_task: session.add(), commit(), refresh()
- [x] T020 [US1] Format add_task response as JSON with task_id, status="created", title, description, completed, timestamps
- [x] T021 [US1] Add error handling to add_task: catch and wrap exceptions with descriptive messages
- [x] T022 [US1] Test add_task with MCP Inspector: create task with title only
- [x] T023 [US1] Test add_task with MCP Inspector: create task with title and description
- [x] T024 [US1] Test add_task error cases: empty title, invalid user_id, title too long
- [x] T025 [US1] Verify task persists in database after add_task call

**Checkpoint**: At this point, User Story 1 should be fully functional - AI can create tasks

---

## Phase 4: User Story 2 - AI Retrieves Tasks (Priority: P2)

**Goal**: Enable AI to list user's tasks with optional status filtering

**Independent Test**: Create multiple tasks, query with different filters, verify correct results returned

### Implementation for User Story 2

- [x] T026 [US2] Implement list_tasks tool function signature in backend/mcp_server/tools/task_tools.py with @mcp.tool() decorator
- [x] T027 [US2] Add user validation logic to list_tasks: verify user exists in database
- [x] T028 [US2] Add status filter validation to list_tasks: must be "all", "pending", or "completed"
- [x] T029 [US2] Implement query logic for "all" filter: select all tasks where user_id matches
- [x] T030 [US2] Implement query logic for "pending" filter: select tasks where user_id matches and completed=False
- [x] T031 [US2] Implement query logic for "completed" filter: select tasks where user_id matches and completed=True
- [x] T032 [US2] Format list_tasks response as JSON with tasks array, count, and filter
- [x] T033 [US2] Add error handling to list_tasks: catch and wrap exceptions
- [x] T034 [US2] Test list_tasks with MCP Inspector: list all tasks
- [x] T035 [US2] Test list_tasks with MCP Inspector: list pending tasks only
- [x] T036 [US2] Test list_tasks with MCP Inspector: list completed tasks only
- [x] T037 [US2] Test list_tasks with empty task list: verify returns empty array with count=0
- [x] T038 [US2] Verify user isolation: user A cannot see user B's tasks

**Checkpoint**: At this point, User Story 2 should be fully functional - AI can retrieve task lists

---

## Phase 5: User Story 3 - AI Marks Tasks Complete (Priority: P3)

**Goal**: Enable AI to mark tasks as completed

**Independent Test**: Create pending task, mark complete via MCP Inspector, verify status updated in database

### Implementation for User Story 3

- [x] T039 [US3] Implement complete_task tool function signature in backend/mcp_server/tools/task_tools.py with @mcp.tool() decorator
- [x] T040 [US3] Add user validation logic to complete_task: verify user exists in database
- [x] T041 [US3] Add task retrieval logic to complete_task: fetch task by task_id
- [x] T042 [US3] Add task existence check to complete_task: raise TaskNotFoundError if not found
- [x] T043 [US3] Add ownership verification to complete_task: verify task.user_id == user_id
- [x] T044 [US3] Implement completion logic: set task.completed = True, update task.updated_at
- [x] T045 [US3] Implement database persistence: session.commit(), refresh()
- [x] T046 [US3] Format complete_task response as JSON with task_id, status="completed", task details
- [x] T047 [US3] Add error handling to complete_task: TaskNotFoundError, UnauthorizedTaskAccessError
- [x] T048 [US3] Test complete_task with MCP Inspector: mark own task as complete
- [x] T049 [US3] Test complete_task error cases: non-existent task, another user's task
- [x] T050 [US3] Verify task completion persists in database

**Checkpoint**: At this point, User Story 3 should be fully functional - AI can mark tasks complete

---

## Phase 6: User Story 4 - AI Updates Task Details (Priority: P4)

**Goal**: Enable AI to update task title and/or description

**Independent Test**: Create task, update title via MCP Inspector, verify only title changed

### Implementation for User Story 4

- [x] T051 [US4] Implement update_task tool function signature in backend/mcp_server/tools/task_tools.py with @mcp.tool() decorator
- [x] T052 [US4] Add user validation logic to update_task: verify user exists in database
- [x] T053 [US4] Add task retrieval logic to update_task: fetch task by task_id
- [x] T054 [US4] Add task existence check to update_task: raise TaskNotFoundError if not found
- [x] T055 [US4] Add ownership verification to update_task: verify task.user_id == user_id
- [x] T056 [US4] Add validation: at least one of title or description must be provided
- [x] T057 [US4] Add title validation if provided: non-empty, max 255 characters
- [x] T058 [US4] Add description validation if provided: max 10,000 characters
- [x] T059 [US4] Implement partial update logic: update only provided fields, preserve others
- [x] T060 [US4] Update task.updated_at timestamp
- [x] T061 [US4] Implement database persistence: session.commit(), refresh()
- [x] T062 [US4] Format update_task response as JSON with task_id, status="updated", task details
- [x] T063 [US4] Add error handling to update_task: ValidationError, TaskNotFoundError, UnauthorizedTaskAccessError
- [x] T064 [US4] Test update_task with MCP Inspector: update title only
- [x] T065 [US4] Test update_task with MCP Inspector: update description only
- [x] T066 [US4] Test update_task with MCP Inspector: update both title and description
- [x] T067 [US4] Test update_task error cases: no fields provided, empty title, another user's task
- [x] T068 [US4] Verify partial updates work correctly: unchanged fields remain unchanged

**Checkpoint**: At this point, User Story 4 should be fully functional - AI can update task details

---

## Phase 7: User Story 5 - AI Deletes Tasks (Priority: P5)

**Goal**: Enable AI to permanently delete tasks

**Independent Test**: Create task, delete via MCP Inspector, verify task removed from database

### Implementation for User Story 5

- [x] T069 [US5] Implement delete_task tool function signature in backend/mcp_server/tools/task_tools.py with @mcp.tool() decorator
- [x] T070 [US5] Add user validation logic to delete_task: verify user exists in database
- [x] T071 [US5] Add task retrieval logic to delete_task: fetch task by task_id
- [x] T072 [US5] Add task existence check to delete_task: raise TaskNotFoundError if not found
- [x] T073 [US5] Add ownership verification to delete_task: verify task.user_id == user_id
- [x] T074 [US5] Capture task details before deletion for response
- [x] T075 [US5] Implement deletion logic: session.delete(task), session.commit()
- [x] T076 [US5] Format delete_task response as JSON with task_id, status="deleted", task details
- [x] T077 [US5] Add error handling to delete_task: TaskNotFoundError, UnauthorizedTaskAccessError
- [x] T078 [US5] Test delete_task with MCP Inspector: delete own task
- [x] T079 [US5] Test delete_task error cases: non-existent task, another user's task
- [x] T080 [US5] Verify task is permanently removed from database
- [x] T081 [US5] Verify deleted task no longer appears in list_tasks results

**Checkpoint**: All user stories should now be independently functional - complete CRUD operations available

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final verification

- [x] T082 [P] Add comprehensive docstrings to all tool functions in backend/mcp_server/tools/task_tools.py
- [x] T083 [P] Add logging statements to all tools: INFO for calls, WARNING for validation failures, ERROR for exceptions
- [x] T084 [P] Create logging configuration in backend/mcp_server/server.py with file and console handlers
- [x] T085 Verify all 5 tools are discoverable in MCP Inspector
- [x] T086 Test full task lifecycle: create → list → complete → update → delete
- [x] T087 Verify user isolation across all tools: user A cannot access user B's tasks
- [x] T088 Test performance with large dataset: create 1000 tasks, verify list_tasks < 2 seconds
- [x] T089 Test error messages are clear and AI-friendly for all error cases
- [x] T090 Verify all tools return consistent JSON response format
- [x] T091 Test concurrent tool calls: verify no race conditions or data corruption
- [x] T092 [P] Document deployment process in backend/mcp_server/README.md
- [x] T093 [P] Create example usage scripts in backend/mcp_server/examples/
- [x] T094 Verify async database connection pool is properly configured and shared
- [x] T095 Run security audit: verify no SQL injection, no cross-user access, input validation working
- [x] T096 Create quickstart verification script based on specs/004-mcp-task-tools/quickstart.md
- [x] T097 Document any deviations from original plan in specs/004-mcp-task-tools/plan.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Independent of US1 (but typically done after for testing)
  - User Story 3 (P3): Can start after Foundational - Independent of US1/US2
  - User Story 4 (P4): Can start after Foundational - Independent of US1/US2/US3
  - User Story 5 (P5): Can start after Foundational - Independent of US1/US2/US3/US4
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Technically independent, but easier to test after US1
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Technically independent, but easier to test after US1
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Technically independent, but easier to test after US1
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - Technically independent, but easier to test after US1

### Within Each User Story

- Foundational phase (Phase 2) must complete before any user story
- Within each user story: Tasks are sequential (implementation → testing → verification)
- Across user stories: All user stories can be implemented in parallel after Foundational phase

### Parallel Opportunities

- Phase 1: T001-T005 can run in parallel (different operations)
- Phase 2: T011 can run in parallel with T006-T010 (different files)
- Phase 3-7: All user story phases can run in parallel after Phase 2 completes
- Phase 8: T082, T083, T084, T092, T093 can run in parallel (different files/concerns)

---

## Parallel Example: After Foundational Phase

```bash
# After Phase 2 completes, launch all user stories in parallel:
Task: "Implement User Story 1 (add_task tool)" - T014-T025
Task: "Implement User Story 2 (list_tasks tool)" - T026-T038
Task: "Implement User Story 3 (complete_task tool)" - T039-T050
Task: "Implement User Story 4 (update_task tool)" - T051-T068
Task: "Implement User Story 5 (delete_task tool)" - T069-T081

# Each story completes independently and can be tested independently
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (install dependencies, create structure)
2. Complete Phase 2: Foundational (async database, MCP server foundation) - CRITICAL
3. Complete Phase 3: User Story 1 (add_task tool)
4. **STOP and VALIDATE**: Test task creation via MCP Inspector
5. Verify data persists and can be retrieved directly from database

### Incremental Delivery

1. Complete Setup + Foundational → Infrastructure ready
2. Add User Story 1 → Test independently → AI can create tasks (MVP!)
3. Add User Story 2 → Test independently → AI can list tasks
4. Add User Story 3 → Test independently → AI can mark tasks complete
5. Add User Story 4 → Test independently → AI can update tasks
6. Add User Story 5 → Test independently → AI can delete tasks
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (add_task)
   - Developer B: User Story 2 (list_tasks) - can start in parallel
   - Developer C: User Story 3 (complete_task) - can start in parallel
   - Developer D: User Story 4 (update_task) - can start in parallel
   - Developer E: User Story 5 (delete_task) - can start in parallel
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files/concerns, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- No test tasks included (not requested in spec, manual testing via MCP Inspector)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Summary

**Total Tasks**: 97
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 8 tasks
- Phase 3 (US1 - P1): 12 tasks
- Phase 4 (US2 - P2): 13 tasks
- Phase 5 (US3 - P3): 12 tasks
- Phase 6 (US4 - P4): 18 tasks
- Phase 7 (US5 - P5): 13 tasks
- Phase 8 (Polish): 16 tasks

**Parallel Opportunities**: 7 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phases 1-3 (25 tasks) deliver minimum viable product with task creation capability

**Independent Test Criteria**:
- US1: Create task via MCP Inspector, verify in database
- US2: List tasks with different filters, verify correct results
- US3: Mark task complete, verify status updated
- US4: Update task fields, verify only specified fields changed
- US5: Delete task, verify removed from database

**Format Validation**: ✅ All tasks follow required checklist format with checkbox, ID, optional [P] and [Story] labels, and file paths
