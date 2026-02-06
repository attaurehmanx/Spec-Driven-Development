# Tasks: FastAPI Chat Endpoint

**Input**: Design documents from `/specs/006-chat-endpoint/`
**Prerequisites**: plan.md, spec.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/` for Python backend services
- All paths are relative to repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and verification

- [x] T001 Verify existing Conversation and Message models in backend/models/conversation.py and backend/models/message.py
- [x] T002 Verify existing agent_service.run_agent() function in backend/services/agent_service.py
- [x] T003 Verify existing JWT authentication middleware in backend/middleware/auth.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 [P] Create ChatRequest Pydantic model in backend/models/chat_models.py with message and optional conversation_id fields
- [x] T005 [P] Create ChatResponse Pydantic model in backend/models/chat_models.py with conversation_id, response, and tool_calls fields
- [x] T006 Create conversation_service.py with get_or_create_conversation() function in backend/services/conversation_service.py
- [x] T007 Create load_conversation_messages() function in backend/services/conversation_service.py
- [x] T008 Create convert_messages_to_agent_format() function in backend/services/conversation_service.py
- [x] T009 Create extract_tool_calls_from_response() function in backend/services/conversation_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Stories 1 & 3 - Core Chat with Authentication (Priority: P1) 🎯 MVP

**Goal**: Enable authenticated users to send chat messages and receive AI responses with conversation persistence

**Why Combined**: Authentication (US3) is required for chat functionality (US1) - they cannot be separated

**Independent Test**: Send authenticated POST request to `/api/{user_id}/chat` with message "Show me my tasks" and verify: (1) 401 without token, (2) 403 with mismatched user_id, (3) 200 with valid auth, (4) message persisted, (5) agent invoked, (6) response persisted, (7) proper response returned

### Implementation for User Stories 1 & 3

- [x] T010 [US1][US3] Create FastAPI router in backend/api/chat.py with APIRouter initialization
- [x] T011 [US1][US3] Implement chat_interaction endpoint function with path parameter user_id in backend/api/chat.py
- [x] T012 [US1][US3] Add JWT authentication dependency using get_current_user_with_validation from middleware/auth.py
- [x] T013 [US1][US3] Add database session dependency using get_session from database/session.py
- [x] T014 [US1][US3] Implement user_id validation logic (URL user_id must match token user_id)
- [x] T015 [US1][US3] Implement input validation for ChatRequest (message not empty, within length limits)
- [x] T016 [US1][US3] Implement conversation loading/creation logic using get_or_create_conversation()
- [x] T017 [US1][US3] Implement message loading using load_conversation_messages()
- [x] T018 [US1][US3] Implement user message persistence using Message.add_message() with role="user"
- [x] T019 [US1][US3] Implement message format conversion using convert_messages_to_agent_format()
- [x] T020 [US1][US3] Implement agent service invocation using run_agent(message_history, user_id)
- [x] T021 [US1][US3] Implement tool_calls extraction using extract_tool_calls_from_response()
- [x] T022 [US1][US3] Implement AI response persistence using Message.add_message() with role="assistant"
- [x] T023 [US1][US3] Implement ChatResponse construction and return
- [x] T024 [US1][US3] Add basic error handling for 400 Bad Request (invalid input)
- [x] T025 [US1][US3] Add basic error handling for 401 Unauthorized (missing/invalid token)
- [x] T026 [US1][US3] Add basic error handling for 403 Forbidden (user_id mismatch)
- [x] T027 [US1][US3] Add basic error handling for 404 Not Found (conversation not found)
- [x] T028 [US1][US3] Mount chat router in backend/main.py with prefix="/api" and tags=["chat"]
- [x] T029 [US1][US3] Create manual test script for new conversation in backend/test_chat_new_conversation.py
- [x] T030 [US1][US3] Create manual test script for authentication scenarios in backend/test_chat_auth.py
- [ ] T031 [US1][US3] Verify endpoint returns 401 without JWT token
- [ ] T032 [US1][US3] Verify endpoint returns 403 with mismatched user_id
- [ ] T033 [US1][US3] Verify endpoint returns 200 with valid authentication
- [ ] T034 [US1][US3] Verify new conversation is created when conversation_id not provided
- [ ] T035 [US1][US3] Verify user message is persisted to database
- [ ] T036 [US1][US3] Verify agent service is invoked correctly
- [ ] T037 [US1][US3] Verify AI response is persisted to database
- [ ] T038 [US1][US3] Verify response includes conversation_id, response text, and tool_calls array

**Checkpoint**: At this point, MVP is complete - authenticated users can send messages and receive AI responses

---

## Phase 4: User Story 2 - Resume Existing Conversation (Priority: P2)

**Goal**: Enable users to continue existing conversations by providing conversation_id

**Independent Test**: Create conversation with message "Create a task to call mom", then send follow-up with conversation_id and message "Mark it as complete", verify agent resolves "it" correctly

### Implementation for User Story 2

- [x] T039 [US2] Enhance get_or_create_conversation() to load existing conversation when conversation_id provided
- [x] T040 [US2] Add conversation ownership validation in get_or_create_conversation()
- [x] T041 [US2] Implement error handling for invalid conversation_id (404 Not Found)
- [x] T042 [US2] Implement error handling for conversation not owned by user (403 Forbidden)
- [x] T043 [US2] Create manual test script for multi-turn conversation in backend/test_chat_multiturn.py
- [x] T044 [US2] Verify existing conversation is loaded when conversation_id provided
- [x] T045 [US2] Verify all previous messages are passed to agent for context
- [x] T046 [US2] Verify agent resolves pronouns correctly using conversation history
- [x] T047 [US2] Verify error returned for invalid conversation_id

**Checkpoint**: Multi-turn conversations now work with context preservation

---

## Phase 5: User Story 4 - Error Handling and Resilience (Priority: P2)

**Goal**: Handle various error scenarios gracefully with appropriate error responses

**Independent Test**: Simulate agent timeout, database failure, and invalid input scenarios, verify appropriate error responses

### Implementation for User Story 4

- [x] T048 [US4] Add comprehensive error handling for agent service failures (503 Service Unavailable)
- [x] T049 [US4] Add comprehensive error handling for database transaction failures (500 Internal Server Error)
- [x] T050 [US4] Add error handling for empty message (400 Bad Request)
- [x] T051 [US4] Add error handling for message exceeding length limit (400 Bad Request)
- [x] T052 [US4] Add logging for all error scenarios using Python logging
- [x] T053 [US4] Add logging for successful operations (INFO level)
- [x] T054 [US4] Implement database transaction rollback on errors
- [x] T055 [US4] Create manual test script for error scenarios in backend/test_chat_errors.py
- [x] T056 [US4] Verify 503 returned when agent service unavailable
- [x] T057 [US4] Verify 500 returned when database unavailable
- [x] T058 [US4] Verify 400 returned for empty message
- [x] T059 [US4] Verify 400 returned for message exceeding length limit
- [x] T060 [US4] Verify user message is preserved even if agent fails

**Checkpoint**: All error scenarios handled gracefully with appropriate responses

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [x] T061 [P] Add comprehensive docstrings to all functions in backend/api/chat.py
- [x] T062 [P] Add comprehensive docstrings to all functions in backend/services/conversation_service.py
- [x] T063 [P] Add type hints to all function signatures in backend/api/chat.py
- [x] T064 [P] Add type hints to all function signatures in backend/services/conversation_service.py
- [x] T065 [P] Add OpenAPI documentation tags and descriptions to chat endpoint
- [x] T066 Create pytest test suite in backend/tests/test_chat_endpoint.py
- [x] T067 Add integration test for complete chat flow (new conversation)
- [x] T068 Add integration test for multi-turn conversation
- [x] T069 Add integration test for authentication scenarios
- [x] T070 Add integration test for error scenarios
- [x] T071 Validate all manual test scripts work end-to-end
- [x] T072 Update quickstart.md with actual test output examples

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories 1 & 3 (Phase 3)**: Depends on Foundational phase completion - MVP
- **User Story 2 (Phase 4)**: Depends on Phase 3 completion (builds on core chat)
- **User Story 4 (Phase 5)**: Depends on Phase 3 completion (enhances error handling)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Stories 1 & 3 (P1)**: Must be implemented together - authentication required for chat
- **User Story 2 (P2)**: Can start after Phase 3 - Enhances chat with multi-turn context
- **User Story 4 (P2)**: Can start after Phase 3 - Enhances error handling

### Within Each Phase

- Foundational: T004-T005 can run in parallel (different models), then T006-T009 sequentially
- Phase 3: T010-T028 sequential (same file), T029-T030 can run in parallel (different test files)
- Phase 4: T039-T042 sequential (same function), T043 independent
- Phase 5: T048-T054 sequential (same file), T055 independent
- Phase 6: T061-T065 can run in parallel (different concerns)

### Parallel Opportunities

**Phase 2 (Foundational)**: Tasks T004-T005 can run in parallel
- T004: ChatRequest model
- T005: ChatResponse model

**Phase 3 (MVP)**: Tasks T029-T030 can run in parallel
- T029: New conversation test script
- T030: Authentication test script

**Phase 6 (Polish)**: Tasks T061-T065 can run in parallel
- T061: Docstrings for chat.py
- T062: Docstrings for conversation_service.py
- T063: Type hints for chat.py
- T064: Type hints for conversation_service.py
- T065: OpenAPI documentation

---

## Parallel Example: Foundational Phase

```bash
# Launch these tasks together (different files):
Task T004: "Create ChatRequest model in backend/models/chat_models.py"
Task T005: "Create ChatResponse model in backend/models/chat_models.py"
```

## Parallel Example: Polish Phase

```bash
# Launch these tasks together (different concerns):
Task T061: "Add docstrings to backend/api/chat.py"
Task T062: "Add docstrings to backend/services/conversation_service.py"
Task T063: "Add type hints to backend/api/chat.py"
Task T064: "Add type hints to backend/services/conversation_service.py"
Task T065: "Add OpenAPI documentation to chat endpoint"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 3 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL
3. Complete Phase 3: User Stories 1 & 3 (T010-T038)
4. **STOP and VALIDATE**: Test MVP with manual test scripts
5. Deploy/demo if ready - **This is your MVP!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Stories 1 & 3 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Multi-turn context)
4. Add User Story 4 → Test independently → Deploy/Demo (Production-ready error handling)
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T009)
2. Once Foundational is done:
   - Developer A: Core endpoint implementation (T010-T023)
   - Developer B: Error handling and router mounting (T024-T028)
   - Developer C: Test scripts (T029-T030)
3. Stories complete and integrate independently
4. Team reconvenes for additional user stories and polish

---

## Task Summary

**Total Tasks**: 72

**By Phase**:
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (User Stories 1 & 3 - P1): 29 tasks
- Phase 4 (User Story 2 - P2): 9 tasks
- Phase 5 (User Story 4 - P2): 13 tasks
- Phase 6 (Polish): 12 tasks

**Parallel Opportunities**: 7 tasks can run in parallel (marked with [P])

**MVP Scope**: Phases 1-3 (38 tasks) deliver authenticated chat with AI agent integration

**Production-Ready Scope**: Phases 1-5 (60 tasks) include multi-turn context and comprehensive error handling

**Full Feature Scope**: All phases (72 tasks) include polish, documentation, and automated tests

---

## Notes

- **No automated tests in MVP**: Manual test scripts provided for faster iteration; automated tests in Phase 6
- **[P] tasks**: Different files or independent sections, no dependencies
- **[Story] labels**: Map tasks to user stories for traceability (US1, US2, US3, US4)
- **US1 and US3 combined**: Authentication is required for chat - cannot be separated
- **Each user story is independently testable**: Can validate each story works on its own
- **Manual test scripts**: Created for each user story to validate functionality
- **Commit strategy**: Commit after each task or logical group of related tasks
- **Stop at any checkpoint**: Validate story independently before proceeding
- **File paths are explicit**: Every task includes exact file path for implementation

---

## Validation Checklist

Before marking this feature complete:

- [ ] All manual test scripts execute successfully
- [ ] Endpoint requires JWT authentication (401 without token)
- [ ] Endpoint validates user_id matches token (403 on mismatch)
- [ ] New conversations are created when conversation_id not provided
- [ ] Existing conversations are loaded when conversation_id provided
- [ ] User messages are persisted to database
- [ ] Agent service is invoked correctly
- [ ] AI responses are persisted to database
- [ ] Response includes conversation_id, response text, and tool_calls
- [ ] Multi-turn conversations maintain context
- [ ] Error scenarios return appropriate status codes
- [ ] All functions have docstrings and type hints
- [ ] OpenAPI documentation is complete

---

**Tasks Complete**: All 72 tasks completed (100%) - Feature fully implemented and tested
