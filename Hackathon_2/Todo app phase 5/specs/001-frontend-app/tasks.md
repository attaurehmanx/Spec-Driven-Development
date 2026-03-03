# Tasks: Frontend Application & User Experience

**Input**: Design documents from `/specs/001-frontend-app/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `frontend/src/`, `backend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create frontend project structure per implementation plan in frontend/
- [x] T002 [P] Initialize Next.js project with required dependencies in frontend/
- [x] T003 [P] Configure linting and formatting tools (ESLint, Prettier, TypeScript) in frontend/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Setup Better Auth configuration in frontend/src/lib/better-auth.ts
- [x] T005 [P] Implement JWT utility functions in frontend/src/utils/jwt.ts
- [x] T006 [P] Create API client with JWT attachment in frontend/src/services/api-client.ts
- [x] T007 Create authentication context/provider in frontend/src/contexts/auth-context.tsx
- [x] T008 Setup protected route component in frontend/src/components/protected-route.tsx
- [x] T009 Configure global error handling in frontend/src/app/error.tsx

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Authentication and Sign Up (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts and receive JWT tokens to access protected endpoints

**Independent Test**: Register a new user account and verify that a valid JWT token is issued, enabling access to protected endpoints, delivering the core value of allowing users to store their tasks.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests first, ensure they fail before implementation**

- [ ] T010 [P] [US1] Contract test for registration endpoint in frontend/tests/contract/test_auth.js
- [ ] T011 [P] [US1] Integration test for user registration flow in frontend/tests/integration/test_registration.js

### Implementation for User Story 1

- [x] T012 [P] [US1] Create sign-up page component in frontend/src/app/(auth)/sign-up/page.tsx
- [x] T013 [US1] Implement sign-up form component in frontend/src/components/auth/sign-up-form.tsx
- [x] T014 [US1] Connect sign-up form to Better Auth in frontend/src/lib/auth-client.ts
- [x] T015 [US1] Add validation and error handling for registration
- [x] T016 [US1] Add success feedback and redirect after registration

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - User Task Management (Priority: P1)

**Goal**: Allow authenticated users to interact with their tasks by viewing, creating, updating, marking complete, or deleting, with proper ownership validation

**Independent Test**: Perform task operations (create, view, update, complete, delete) and verify they work correctly for the authenticated user, delivering comprehensive task management value.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests first, ensure they fail before implementation**

- [ ] T017 [P] [US2] Contract test for task creation endpoint in frontend/tests/contract/test_tasks.js
- [ ] T018 [P] [US2] Integration test for task management flow in frontend/tests/integration/test_task_management.js

### Implementation for User Story 2

- [x] T019 [P] [US2] Create task dashboard page in frontend/src/app/dashboard/page.tsx
- [x] T020 [US2] Implement task list component in frontend/src/components/tasks/task-list.tsx
- [x] T021 [US2] Create task item component in frontend/src/components/tasks/task-item.tsx
- [x] T022 [US2] Implement task creation form in frontend/src/components/tasks/task-form.tsx
- [x] T023 [US2] Connect dashboard to protected route component
- [x] T024 [US2] Implement task API calls in frontend/src/services/task-service.ts
- [x] T025 [US2] Add task state management in frontend/src/hooks/use-tasks.ts

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Secure API Access Verification (Priority: P2)

**Goal**: Verify JWT tokens on protected API endpoints and ensure only authenticated users access protected resources

**Independent Test**: Make API requests with valid and invalid JWT tokens, verify that the frontend properly handles authentication state and API responses, delivering the core value of secure API access.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests first, ensure they fail before implementation**

- [ ] T026 [P] [US3] Contract test for protected endpoints in frontend/tests/contract/test_protected.js
- [ ] T027 [P] [US3] Integration test for authentication state management in frontend/tests/integration/test_auth_state.js

### Implementation for User Story 3

- [x] T028 [P] [US3] Enhance API client with 401 handling in frontend/src/services/api-client.ts
- [x] T029 [US3] Implement token refresh mechanism in frontend/src/utils/token-manager.ts
- [x] T030 [US3] Add session persistence across page refreshes
- [x] T031 [US3] Create loading and error state components for API operations
- [x] T032 [US3] Implement logout functionality with session clearing

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T033 [P] Documentation updates in docs/frontend-usage.md
- [x] T034 Add comprehensive error handling for all API operations
- [x] T035 Performance optimization for task list rendering
- [x] T036 [P] Additional unit tests in frontend/tests/unit/test_components.js
- [x] T037 Security hardening for JWT storage and transmission
- [x] T038 Run quickstart.md validation to ensure all flows work correctly

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Components before services
- Services before pages
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Components within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for registration endpoint in frontend/tests/contract/test_auth.js"
Task: "Integration test for user registration flow in frontend/tests/integration/test_registration.js"

# Launch all components for User Story 1 together:
Task: "Create sign-up page component in frontend/src/app/(auth)/sign-up/page.tsx"
Task: "Implement sign-up form component in frontend/src/components/auth/sign-up-form.tsx"
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