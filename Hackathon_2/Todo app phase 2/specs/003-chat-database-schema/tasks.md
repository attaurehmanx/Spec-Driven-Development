---
description: "Task list for Chat Conversation Persistence feature implementation"
---

# Tasks: Chat Conversation Persistence

**Input**: Design documents from `/specs/003-chat-database-schema/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT requested in the feature specification, so no test tasks are included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/models/`, `backend/database/`, `backend/api/`
- Paths follow existing backend structure from Phase 2

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing infrastructure and prepare for new models

- [x] T001 Verify existing backend structure in backend/ directory
- [x] T002 Verify SQLModel and database dependencies are installed
- [x] T003 Verify database connection works using backend/database/session.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core database models that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 [P] Create MessageRole enum in backend/models/message.py
- [x] T005 [P] Create Conversation model in backend/models/conversation.py
- [x] T006 Create Message model in backend/models/message.py (depends on T004)
- [x] T007 Update backend/models/__init__.py to import Conversation and Message models
- [x] T008 Run database migrations using backend/database/migrations.py to create tables
- [x] T009 Verify conversation and message tables exist in database with correct schema

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Persistent Chat Conversations (Priority: P1) 🎯 MVP

**Goal**: Enable basic conversation and message persistence so users never lose their chat history

**Independent Test**: Create a conversation, add messages, verify they persist in database and can be retrieved

### Implementation for User Story 1

- [x] T010 [P] [US1] Create helper function to create new conversation in backend/models/conversation.py
- [x] T011 [P] [US1] Create helper function to add message to conversation in backend/models/message.py
- [x] T012 [US1] Implement conversation creation with user_id validation
- [x] T013 [US1] Implement message insertion with conversation_id validation
- [x] T014 [US1] Implement automatic updated_at timestamp update when message added
- [x] T015 [US1] Verify foreign key constraints work (conversation -> user, message -> conversation)
- [x] T016 [US1] Test creating conversation with multiple messages in correct chronological order
- [x] T017 [US1] Verify messages persist after database connection closes and reopens

**Checkpoint**: At this point, User Story 1 should be fully functional - conversations and messages can be created and persisted

---

## Phase 4: User Story 2 - Conversation History Access (Priority: P2)

**Goal**: Enable users to list and retrieve their past conversations with all messages

**Independent Test**: Create multiple conversations, query user's conversation list, retrieve specific conversation with messages

### Implementation for User Story 2

- [x] T018 [P] [US2] Implement query to list user's conversations ordered by updated_at DESC in backend/models/conversation.py
- [x] T019 [P] [US2] Implement query to get single conversation by ID with user_id validation in backend/models/conversation.py
- [x] T020 [US2] Implement query to get all messages for a conversation ordered by created_at ASC in backend/models/message.py
- [x] T021 [US2] Implement pagination support for conversation list (limit/offset)
- [x] T022 [US2] Verify user isolation - users can only see their own conversations
- [x] T023 [US2] Verify messages are returned in correct chronological order
- [x] T024 [US2] Test retrieving conversation with 1000+ messages (performance verification)
- [x] T025 [US2] Test empty state - user with no conversations returns empty list

**Checkpoint**: At this point, User Story 2 should be fully functional - users can list and retrieve their conversation history

---

## Phase 5: User Story 3 - Conversation Organization with Titles (Priority: P3)

**Goal**: Enable users to identify conversations by meaningful titles

**Independent Test**: Create conversation with title, update title, verify title appears in conversation list

### Implementation for User Story 3

- [x] T026 [P] [US3] Implement function to update conversation title in backend/models/conversation.py
- [x] T027 [P] [US3] Add validation for title length (max 255 characters)
- [x] T028 [US3] Implement automatic updated_at timestamp update when title changed
- [x] T029 [US3] Verify title is optional (nullable) and conversations can be created without title
- [x] T030 [US3] Test updating title for existing conversation
- [x] T031 [US3] Verify title appears in conversation list query results

**Checkpoint**: All user stories should now be independently functional - conversations have titles for organization

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final verification

- [x] T032 [P] Add docstrings to all model classes and methods
- [x] T033 [P] Verify all indexes are created (user_id, updated_at, conversation_id, created_at)
- [x] T034 Run quickstart.md verification scripts to test all functionality
- [x] T035 Verify performance targets met (<2s conversation retrieval, 1000+ messages supported)
- [x] T036 Verify user isolation enforcement (cross-user access blocked)
- [x] T037 Document any deviations from original plan in plan.md
- [x] T038 Update backend/models/__init__.py with proper __all__ exports

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Builds on US1 models but independently testable
  - User Story 3 (P3): Can start after Foundational - Builds on US1 models but independently testable
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Uses models from US1 but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Uses models from US1 but independently testable

### Within Each User Story

- Foundational models (Phase 2) must complete before any user story
- Within US1: Helper functions can be parallel, then implementation tasks sequential
- Within US2: Query functions can be parallel, then verification sequential
- Within US3: Update and validation can be parallel, then testing sequential

### Parallel Opportunities

- Phase 1: All setup tasks can run in parallel (T001, T002, T003)
- Phase 2: T004 and T005 can run in parallel (different files)
- Phase 3 (US1): T010 and T011 can run in parallel (different files)
- Phase 4 (US2): T018 and T019 can run in parallel (different query functions)
- Phase 5 (US3): T026 and T027 can run in parallel (different concerns)
- Phase 6: T032 and T033 can run in parallel (documentation vs verification)

---

## Parallel Example: Foundational Phase

```bash
# Launch model creation tasks in parallel:
Task: "Create MessageRole enum in backend/models/message.py"
Task: "Create Conversation model in backend/models/conversation.py"

# Then sequentially:
Task: "Create Message model in backend/models/message.py" (needs MessageRole)
Task: "Update backend/models/__init__.py" (needs both models)
Task: "Run migrations" (needs all models imported)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (verify infrastructure)
2. Complete Phase 2: Foundational (create models) - CRITICAL
3. Complete Phase 3: User Story 1 (basic persistence)
4. **STOP and VALIDATE**: Test conversation and message creation independently
5. Verify data persists and can be retrieved

### Incremental Delivery

1. Complete Setup + Foundational → Models ready
2. Add User Story 1 → Test independently → Basic persistence works (MVP!)
3. Add User Story 2 → Test independently → Can list and retrieve history
4. Add User Story 3 → Test independently → Can organize with titles
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (basic persistence)
   - Developer B: User Story 2 (history access) - can start in parallel
   - Developer C: User Story 3 (titles) - can start in parallel
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- No test tasks included (not requested in spec)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Summary

**Total Tasks**: 38
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (US1 - P1): 8 tasks
- Phase 4 (US2 - P2): 8 tasks
- Phase 5 (US3 - P3): 6 tasks
- Phase 6 (Polish): 7 tasks

**Parallel Opportunities**: 11 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phases 1-3 (17 tasks) deliver minimum viable product with basic conversation persistence

**Independent Test Criteria**:
- US1: Create conversation, add messages, verify persistence
- US2: List conversations, retrieve specific conversation with messages
- US3: Create/update conversation title, verify in list

**Format Validation**: ✅ All tasks follow required checklist format with checkbox, ID, optional [P] and [Story] labels, and file paths
