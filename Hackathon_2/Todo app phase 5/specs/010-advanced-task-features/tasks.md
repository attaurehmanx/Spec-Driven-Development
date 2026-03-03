---

description: "Task list for advanced task management features implementation"
---

# Tasks: Advanced Task Management Features

**Input**: Design documents from `/specs/010-advanced-task-features/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/, research.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are omitted per template guidelines.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below follow web application structure from plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency setup

- [X] T001 Install backend dependencies for Phase 1 in backend/requirements.txt (no new dependencies needed)
- [X] T002 [P] Install frontend dependencies in frontend/package.json (react-datepicker, @headlessui/react if not present)
- [X] T003 [P] Verify Neon PostgreSQL connection from backend/src/main.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core database schema that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create Alembic migration script backend/alembic/versions/010_add_advanced_task_features.py with all new columns (priority, tags, due_date, recurring, parent_task_id)
- [X] T005 Add database indexes in migration script (idx_tasks_priority, idx_tasks_tags GIN, idx_tasks_due_date, idx_tasks_user_priority, idx_tasks_user_completed, idx_tasks_user_due_completed)
- [X] T006 Add check constraints in migration script (chk_priority, chk_recurring, chk_no_self_parent)
- [X] T007 Add foreign key constraint for parent_task_id in migration script (fk_tasks_parent_task_id)
- [X] T008 Run Alembic migration to apply schema changes (alembic upgrade head)
- [X] T009 Update Task SQLModel in backend/src/models/task.py with new fields (priority: PriorityLevel, tags: List[str], due_date: Optional[datetime], recurring: RecurringPattern, parent_task_id: Optional[UUID])
- [X] T010 [P] Create PriorityLevel enum in backend/src/models/task.py (HIGH, MEDIUM, LOW)
- [X] T011 [P] Create RecurringPattern enum in backend/src/models/task.py (NONE, DAILY, WEEKLY, MONTHLY)
- [X] T012 [P] Create TaskCreate Pydantic schema in backend/src/schemas/task.py with validation for new fields
- [X] T013 [P] Create TaskUpdate Pydantic schema in backend/src/schemas/task.py with optional new fields
- [X] T014 [P] Create TaskResponse Pydantic schema in backend/src/schemas/task.py with is_overdue computed property
- [X] T015 Add tag validation in TaskCreate schema (regex pattern ^[a-zA-Z0-9-]{1,50}$, max 20 tags)
- [X] T016 Add due_date validation in TaskCreate schema (not >1 year past, not >10 years future)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Task Priorities and Basic Organization (Priority: P1) 🎯 MVP

**Goal**: Enable users to assign priority levels (high/medium/low) to tasks and filter/sort by priority

**Independent Test**: Create tasks with different priorities, filter by priority, sort by priority, verify visual indicators

### Implementation for User Story 1

- [X] T017 [P] [US1] Update GET /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept priority query parameter
- [X] T018 [P] [US1] Implement priority filtering logic in backend/src/services/task_service.py filter_tasks method
- [X] T019 [P] [US1] Implement priority sorting logic in backend/src/services/task_service.py sort_tasks method (high=1, medium=2, low=3)
- [X] T020 [US1] Update POST /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept priority in request body
- [X] T021 [US1] Update PUT /api/{user_id}/tasks/{id} endpoint in backend/src/api/tasks.py to accept priority in request body
- [X] T022 [P] [US1] Update TaskForm component in frontend/src/components/TaskForm.tsx to include priority dropdown (high/medium/low)
- [X] T023 [P] [US1] Create TaskFilters component in frontend/src/components/TaskFilters.tsx with priority filter checkboxes
- [X] T024 [P] [US1] Add priority visual indicators to TaskList component in frontend/src/components/TaskList.tsx (color coding or icons)
- [X] T025 [US1] Update taskApi.ts in frontend/src/services/taskApi.ts to send priority query parameter and request body field
- [X] T026 [US1] Integrate TaskFilters component into Dashboard page in frontend/src/pages/dashboard.tsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Tags and Search (Priority: P2)

**Goal**: Enable users to categorize tasks with tags and search by title/description

**Independent Test**: Create tasks with tags, filter by tags, search for keywords in title/description

### Implementation for User Story 2

- [X] T027 [P] [US2] Update GET /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept tags query parameter (comma-separated)
- [X] T028 [P] [US2] Update GET /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept search query parameter
- [X] T029 [P] [US2] Implement tag filtering logic in backend/src/services/task_service.py filter_tasks method (PostgreSQL array && operator)
- [X] T030 [P] [US2] Implement search logic in backend/src/services/task_service.py search_tasks method (ILIKE on title and description)
- [X] T031 [US2] Create GET /api/{user_id}/tasks/tags endpoint in backend/src/api/tasks.py to return unique tags for user
- [X] T032 [P] [US2] Update TaskForm component in frontend/src/components/TaskForm.tsx to include tags input (multi-select or comma-separated)
- [X] T033 [P] [US2] Add tag filter to TaskFilters component in frontend/src/components/TaskFilters.tsx (multi-select dropdown)
- [X] T034 [P] [US2] Create TaskSearch component in frontend/src/components/TaskSearch.tsx with debounced search input
- [X] T035 [US2] Update taskApi.ts in frontend/src/services/taskApi.ts to send tags and search query parameters
- [X] T036 [US2] Integrate TaskSearch component into Dashboard page in frontend/src/pages/dashboard.tsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Advanced Sorting and Filtering (Priority: P3)

**Goal**: Enable users to filter by completion status and sort by multiple criteria

**Independent Test**: Filter by done/not done, sort by created date, sort by due date, sort alphabetically

### Implementation for User Story 3

- [X] T037 [P] [US3] Update GET /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept status query parameter (done/not_done)
- [X] T038 [P] [US3] Update GET /api/{user_id}/tasks endpoint in backend/src/api/tasks.py to accept sort query parameter (priority/due_date/created_at/title)
- [X] T039 [P] [US3] Implement status filtering logic in backend/src/services/task_service.py filter_tasks method
- [X] T040 [P] [US3] Implement sorting logic in backend/src/services/task_service.py sort_tasks method for all sort options
- [X] T041 [P] [US3] Implement overdue-first sorting logic for due_date sort (CASE WHEN due_date < NOW())
- [X] T042 [P] [US3] Add status filter to TaskFilters component in frontend/src/components/TaskFilters.tsx (done/not done checkboxes)
- [X] T043 [P] [US3] Add sort dropdown to TaskList component in frontend/src/components/TaskList.tsx (priority/due date/created date/alphabetical)
- [X] T044 [P] [US3] Implement sort preference persistence in frontend localStorage
- [X] T045 [US3] Update taskApi.ts in frontend/src/services/taskApi.ts to send status and sort query parameters

**Checkpoint**: All user stories 1, 2, AND 3 should now be independently functional

---

## Phase 6: User Story 4 - Due Dates and Reminders (Priority: P4)

**Goal**: Enable users to set due dates and receive reminder events via Dapr pub/sub

**Independent Test**: Create tasks with due dates, verify reminder events published, verify overdue highlighting

**Note**: This phase requires infrastructure setup (Redpanda + Dapr) before implementation

### Infrastructure Setup (Phase 3-4 from plan.md)

- [X] T046 Start Minikube cluster with 1900MB memory (minikube start --memory=1900mb --cpus=2 --disk-size=16g --driver=docker)
- [X] T047 Install Redpanda Helm chart with minimal config in helm/values-dev.yaml (1 replica, no persistence, 512Mi memory)
- [X] T048 Deploy Redpanda to Minikube (helm install redpanda redpanda/redpanda -f helm/values-dev.yaml)
- [X] T049 Verify Redpanda pod is running (kubectl get pods -l app.kubernetes.io/name=redpanda)
- [X] T050 Create tasks.reminder topic in Redpanda (kubectl exec redpanda-0 -- rpk topic create tasks.reminder)
- [X] T051 Install Dapr to Kubernetes (dapr init -k --enable-ha=false --enable-mtls=false)
- [X] T052 Verify Dapr control plane is running (dapr status -k)
- [X] T053 Create Dapr Kafka pub/sub component in helm/templates/dapr-components/kafka-pubsub.yaml
- [X] T054 Apply Dapr pub/sub component (kubectl apply -f helm/templates/dapr-components/kafka-pubsub.yaml)
- [X] T055 Create Dapr Postgres state component in helm/templates/dapr-components/postgres-state.yaml
- [X] T056 Apply Dapr state component (kubectl apply -f helm/templates/dapr-components/postgres-state.yaml)
- [X] T057 Update backend deployment in helm/templates/backend-deployment.yaml with Dapr annotations (dapr.io/enabled, dapr.io/app-id, dapr.io/app-port)
- [X] T058 Deploy backend with Dapr sidecar (helm upgrade --install task-app ./helm -f helm/values-dev.yaml)
- [X] T059 Verify Dapr sidecar is injected (kubectl get pods, should show 2/2 containers)

### Backend Implementation (Phase 3-4: Temporary Kafka Client)

- [X] T060 [P] [US4] Add aiokafka dependency to backend/requirements.txt (temporary, will be removed in Phase 5)
- [X] T061 [P] [US4] Add apscheduler dependency to backend/requirements.txt for background jobs
- [X] T062 [P] [US4] Create reminder_publisher.py in backend/src/events/ with direct Kafka publishing (temporary)
- [X] T063 [P] [US4] Create reminder_job.py in backend/src/jobs/ with APScheduler periodic check for upcoming due dates
- [X] T064 [US4] Update main.py in backend/src/main.py to initialize APScheduler and schedule reminder job (every 5 minutes)
- [X] T065 [US4] Implement query in reminder_job.py to find tasks with due_date BETWEEN NOW() AND NOW() + INTERVAL '1 hour'
- [X] T066 [US4] Implement reminder event publishing in reminder_job.py (publish to tasks.reminder topic)

### Frontend Implementation

- [X] T067 [P] [US4] Update TaskForm component in frontend/src/components/TaskForm.tsx to include due date picker (react-datepicker)
- [X] T068 [P] [US4] Add overdue task highlighting to TaskList component in frontend/src/components/TaskList.tsx (red text or warning icon)
- [X] T069 [P] [US4] Implement is_overdue computed property in TaskResponse schema (due_date < NOW() AND completed = false)
- [X] T070 [US4] Update taskApi.ts in frontend/src/services/taskApi.ts to send due_date in request body

### Dapr Refactor (Phase 5 from plan.md)

- [ ] T071 Remove aiokafka from backend/requirements.txt (FUTURE: Optional refactor to pure Dapr)
- [ ] T072 Refactor reminder_publisher.py in backend/src/events/ to use Dapr HTTP API (POST http://localhost:3500/v1.0/publish/kafka-pubsub/tasks.reminder)
- [ ] T073 Update reminder_job.py in backend/src/jobs/ to use refactored Dapr-based publisher (FUTURE: Optional refactor)
- [ ] T074 Verify no direct Kafka imports remain in backend/src/ (grep -r "from aiokafka" backend/src/) (FUTURE: Optional refactor)

**Note**: T071-T074 are optional future refactoring tasks. The current implementation works with aiokafka + Dapr.

**Checkpoint**: User Story 4 should be fully functional with Dapr-first communication

---

## Phase 7: User Story 5 - Recurring Tasks (Priority: P5)

**Goal**: Enable users to create recurring tasks that auto-generate new instances on completion

**Independent Test**: Create daily/weekly/monthly recurring tasks, complete them, verify new instances created with correct due dates

### Implementation for User Story 5

- [X] T075 [P] [US5] Create recurring_service.py in backend/src/services/ with create_next_instance method
- [X] T076 [P] [US5] Implement calculate_next_due_date method in recurring_service.py for daily/weekly/monthly patterns
- [X] T077 [P] [US5] Implement month-end edge case handling in calculate_next_due_date (Jan 31 → Feb 28/29, Jan 15 → Feb 15)
- [X] T078 [US5] Update PATCH /api/{user_id}/tasks/{id}/complete endpoint in backend/src/api/tasks.py to check recurring field
- [X] T079 [US5] Call create_next_instance from completion endpoint when recurring != 'none'
- [X] T080 [US5] Ensure new recurring instance inherits title, description, priority, tags, recurring pattern
- [X] T081 [US5] Ensure new recurring instance gets new UUID, completed=false, calculated due_date, parent_task_id=original_task_id
- [X] T082 [P] [US5] Update TaskForm component in frontend/src/components/TaskForm.tsx to include recurring dropdown (none/daily/weekly/monthly)
- [X] T083 [US5] Update taskApi.ts in frontend/src/services/taskApi.ts to send recurring in request body
- [X] T084 [US5] Update completion endpoint response to include next_instance field when recurring task completed

**Checkpoint**: All user stories 1-5 should now be independently functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T085 [P] Add task count display to TaskFilters component in frontend/src/components/TaskFilters.tsx
- [X] T086 [P] Add "Clear all filters" button to TaskFilters component in frontend/src/components/TaskFilters.tsx
- [X] T087 [P] Implement "No tasks found" message in TaskList component in frontend/src/components/TaskList.tsx
- [X] T088 [P] Implement "No tasks with this tag" message in TaskFilters component in frontend/src/components/TaskFilters.tsx
- [X] T089 [P] Add loading states to TaskList component in frontend/src/components/TaskList.tsx
- [X] T090 [P] Add error handling for API failures in taskApi.ts in frontend/src/services/taskApi.ts
- [X] T091 [P] Implement pagination for large task lists (limit=1000, offset parameter) in backend/src/api/tasks.py
- [X] T092 [P] Add rate limiting headers to API responses in backend/src/api/tasks.py
- [X] T093 [P] Create GET /api/{user_id}/tasks/overdue endpoint in backend/src/api/tasks.py
- [X] T094 [P] Implement overdue tasks query in backend/src/services/task_service.py (due_date < NOW() AND completed = false)
- [X] T095 Code cleanup and remove unused imports across backend/src/
- [X] T096 Code cleanup and remove unused imports across frontend/src/

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (Phase 4)**: Depends on Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (Phase 5)**: Depends on Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (Phase 6)**: Depends on Foundational (Phase 2) + Infrastructure setup (T046-T059) - No dependencies on other stories
- **User Story 5 (Phase 7)**: Depends on Foundational (Phase 2) - No dependencies on other stories (but logically builds on US4 due dates)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) + Infrastructure (T046-T059) - No dependencies on other stories
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - No dependencies on other stories (but benefits from US4 due dates)

### Within Each User Story

- Backend API changes before frontend UI changes (API must exist for frontend to call)
- Models/schemas before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1, 2, 3, 5 can start in parallel (US4 requires infrastructure first)
- Within each user story, tasks marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
Task T017: Update GET endpoint for priority query parameter
Task T018: Implement priority filtering logic
Task T019: Implement priority sorting logic
Task T022: Update TaskForm with priority dropdown
Task T023: Create TaskFilters component
Task T024: Add priority visual indicators

# Then sequential tasks:
Task T020: Update POST endpoint (depends on T017 pattern)
Task T021: Update PUT endpoint (depends on T017 pattern)
Task T025: Update taskApi.ts (depends on T022-T024 UI components)
Task T026: Integrate into Dashboard (depends on T023, T025)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Priority management)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo (requires infrastructure)
6. Add User Story 5 → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Priority management)
   - Developer B: User Story 2 (Tags and search)
   - Developer C: User Story 3 (Sorting and filtering)
3. After infrastructure setup (T046-T059):
   - Developer D: User Story 4 (Due dates and reminders)
   - Developer E: User Story 5 (Recurring tasks)
4. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Infrastructure tasks (T046-T059) are prerequisites for User Story 4 only
- Dapr refactor tasks (T071-T074) achieve Principle XIV (Dapr-first communication)
