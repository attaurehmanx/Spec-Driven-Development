# Tasks: AI Agent Service Configuration

**Input**: Design documents from `/specs/005-ai-agent-service/`
**Prerequisites**: plan.md, spec.md, research.md, contracts/, quickstart.md

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

**Purpose**: Project initialization and dependency installation

- [x] T001 Install OpenAI Python SDK in backend/requirements.txt
- [x] T002 Add GEMINI_API_KEY, GEMINI_MODEL, GEMINI_BASE_URL to backend/.env
- [x] T003 Verify MCP tools are accessible from backend/mcp_server/tools/task_tools.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 [P] Create AgentConfiguration class in backend/config/agent_config.py with Pydantic settings
- [x] T005 [P] Create custom exception classes (AgentServiceError, LLMAPIError, ToolExecutionError, MaxIterationsExceededError) in backend/services/agent_service.py
- [x] T006 [P] Create system prompt template with user isolation rules in backend/config/agent_config.py
- [x] T007 Implement bind_mcp_tools() function to convert MCP tools to OpenAI function schemas in backend/services/agent_service.py
- [x] T008 Implement execute_tool_call() function with user_id injection and error handling in backend/services/agent_service.py
- [x] T009 Create helper function to convert message history to OpenAI format in backend/services/agent_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Agent Conversation (Priority: P1) 🎯 MVP

**Goal**: Enable the agent to process simple user messages, call appropriate MCP tools, and return natural language responses

**Independent Test**: Send message "Show me my tasks" and verify agent calls list_tasks tool and returns formatted response

### Implementation for User Story 1

- [x] T010 [US1] Implement run_agent() function skeleton with input validation in backend/services/agent_service.py
- [x] T011 [US1] Initialize OpenAI client with Gemini API configuration in run_agent()
- [x] T012 [US1] Implement agent loop: call LLM with messages and tools in run_agent()
- [x] T013 [US1] Implement tool call detection and execution logic in agent loop
- [x] T014 [US1] Implement tool result formatting and message appending in agent loop
- [x] T015 [US1] Implement termination detection (no tool calls = done) in agent loop
- [x] T016 [US1] Implement max iterations limit (15) with graceful timeout message
- [x] T017 [US1] Add basic error handling for LLM API failures (RateLimitError, APITimeoutError, APIError)
- [x] T018 [US1] Create manual test script backend/test_agent_manual.py for "Create a task to buy groceries"
- [x] T019 [US1] Create manual test script backend/test_agent_list_tasks.py for "Show me my tasks"
- [ ] T020 [US1] Verify agent correctly calls add_task tool with proper parameters
- [ ] T021 [US1] Verify agent correctly calls list_tasks tool and formats response

**Checkpoint**: At this point, User Story 1 should be fully functional - agent can process simple queries and execute tools

---

## Phase 4: User Story 2 - Multi-Turn Conversation with Context (Priority: P2)

**Goal**: Enable the agent to maintain conversation context across multiple messages and reference previous exchanges

**Independent Test**: Send sequence "Create a task to call mom" followed by "Mark it as complete" and verify agent understands "it" refers to the previously created task

### Implementation for User Story 2

- [x] T022 [US2] Enhance run_agent() to accept and process message_history list in backend/services/agent_service.py
- [x] T023 [US2] Implement conversation history preservation in agent loop (append all messages including tool calls)
- [x] T024 [US2] Update system prompt to instruct agent to use conversation context for pronoun resolution
- [x] T025 [US2] Create manual test script backend/test_agent_multiturn.py with multi-turn conversation scenario
- [ ] T026 [US2] Verify agent correctly references previous messages when user says "Mark it as done"
- [ ] T027 [US2] Verify agent correctly identifies "the first one" from previous list_tasks response

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - agent handles both simple and contextual queries

---

## Phase 5: User Story 3 - Error Handling and User Guidance (Priority: P2)

**Goal**: Provide clear, helpful error messages when tool operations fail or user requests are ambiguous

**Independent Test**: Attempt to update a non-existent task and verify agent explains the error in user-friendly language

### Implementation for User Story 3

- [x] T028 [US3] Enhance execute_tool_call() to catch TaskNotFoundError and return user-friendly error dict in backend/services/agent_service.py
- [x] T029 [US3] Enhance execute_tool_call() to catch UnauthorizedTaskAccessError and return user-friendly error dict
- [x] T030 [US3] Enhance execute_tool_call() to catch ValidationError and return user-friendly error dict
- [x] T031 [US3] Update agent loop to handle tool execution errors and feed them back to LLM for natural language explanation
- [x] T032 [US3] Add logging for all error conditions (INFO for normal ops, WARNING for limits, ERROR for failures)
- [x] T033 [US3] Update system prompt to instruct agent to ask clarifying questions for ambiguous requests
- [x] T034 [US3] Create manual test script backend/test_agent_errors.py for error scenarios (task not found, ambiguous request)
- [ ] T035 [US3] Verify agent explains "Task not found" error in natural language
- [ ] T036 [US3] Verify agent asks clarifying questions when user says "Update my task" without specifying which task

**Checkpoint**: All error scenarios should now be handled gracefully with user-friendly messages

---

## Phase 6: User Story 4 - Time-Aware Task Management (Priority: P3)

**Goal**: Enable the agent to correctly interpret time-related queries like "today", "tomorrow", "next week"

**Independent Test**: Send message "Show me tasks due today" and verify agent correctly interprets "today" based on current system time

### Implementation for User Story 4

- [x] T037 [US4] Update system prompt to instruct agent to check current time for time-relative queries in backend/config/agent_config.py
- [x] T038 [US4] Add current_time parameter to system prompt injection (format: ISO 8601 with timezone)
- [x] T039 [US4] Update system prompt with examples of time interpretation (today = current date, tomorrow = current date + 1 day)
- [x] T040 [US4] Create manual test script backend/test_agent_time_aware.py for time-relative queries
- [ ] T041 [US4] Verify agent correctly interprets "today" and filters tasks by current date
- [ ] T042 [US4] Verify agent correctly interprets "tomorrow" and creates task with tomorrow's date

**Checkpoint**: All user stories should now be independently functional with time-aware capabilities

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [x] T043 [P] Add comprehensive docstrings to all functions in backend/services/agent_service.py
- [x] T044 [P] Add type hints to all function signatures in backend/services/agent_service.py
- [x] T045 [P] Update backend/config/agent_config.py with configuration comments and defaults
- [x] T046 Create AgentResponse dataclass for structured return values in backend/services/agent_service.py
- [x] T047 Update run_agent() to return AgentResponse instead of plain string
- [x] T048 Add performance logging (iteration count, execution time) to run_agent()
- [ ] T049 Validate all manual test scripts work end-to-end per quickstart.md
- [ ] T050 Update quickstart.md with actual test output examples from manual testing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P2 → P3)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Enhances US1 but independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Enhances error handling across all stories
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Adds time awareness to US1

### Within Each User Story

- Core implementation before manual testing
- Manual testing before moving to next story
- Story complete and validated before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**: All 3 tasks can run in parallel
- T001, T002, T003 (different files, no dependencies)

**Phase 2 (Foundational)**: Tasks T004, T005, T006 can run in parallel
- T004 (agent_config.py - AgentConfiguration)
- T005 (agent_service.py - exceptions)
- T006 (agent_config.py - system prompt)
- Then T007, T008, T009 run sequentially (all in agent_service.py)

**Phase 3-6 (User Stories)**: Once Foundational completes, all user stories can start in parallel if team capacity allows
- Developer A: User Story 1 (T010-T021)
- Developer B: User Story 2 (T022-T027)
- Developer C: User Story 3 (T028-T036)
- Developer D: User Story 4 (T037-T042)

**Phase 7 (Polish)**: Tasks T043, T044, T045 can run in parallel
- T043 (docstrings)
- T044 (type hints)
- T045 (config comments)

---

## Parallel Example: Foundational Phase

```bash
# Launch these tasks together (different sections of files):
Task T004: "Create AgentConfiguration class in backend/config/agent_config.py"
Task T005: "Create custom exception classes in backend/services/agent_service.py"
Task T006: "Create system prompt template in backend/config/agent_config.py"
```

## Parallel Example: User Stories (if team has 4 developers)

```bash
# After Foundational phase completes, launch all user stories in parallel:
Developer A: "Implement User Story 1 (T010-T021) - Basic Agent Conversation"
Developer B: "Implement User Story 2 (T022-T027) - Multi-Turn Conversation"
Developer C: "Implement User Story 3 (T028-T036) - Error Handling"
Developer D: "Implement User Story 4 (T037-T042) - Time-Aware Management"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL
3. Complete Phase 3: User Story 1 (T010-T021)
4. **STOP and VALIDATE**: Test User Story 1 independently with manual test scripts
5. Deploy/demo if ready - **This is your MVP!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Context-aware conversations)
4. Add User Story 3 → Test independently → Deploy/Demo (Production-ready error handling)
5. Add User Story 4 → Test independently → Deploy/Demo (Time-aware features)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T009)
2. Once Foundational is done:
   - Developer A: User Story 1 (T010-T021)
   - Developer B: User Story 2 (T022-T027)
   - Developer C: User Story 3 (T028-T036)
   - Developer D: User Story 4 (T037-T042)
3. Stories complete and integrate independently
4. Team reconvenes for Polish phase (T043-T050)

---

## Task Summary

**Total Tasks**: 50

**By Phase**:
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (User Story 1 - P1): 12 tasks
- Phase 4 (User Story 2 - P2): 6 tasks
- Phase 5 (User Story 3 - P2): 9 tasks
- Phase 6 (User Story 4 - P3): 6 tasks
- Phase 7 (Polish): 8 tasks

**Parallel Opportunities**: 6 tasks can run in parallel (marked with [P])

**MVP Scope**: Phases 1-3 (21 tasks) deliver basic agent conversation capability

**Production-Ready Scope**: Phases 1-5 (36 tasks) include error handling for production deployment

**Full Feature Scope**: All phases (50 tasks) include time-aware capabilities and polish

---

## Notes

- **No tests included**: Spec did not request TDD approach; manual testing scripts provided instead
- **[P] tasks**: Different files or independent sections, no dependencies
- **[Story] labels**: Map tasks to user stories for traceability (US1, US2, US3, US4)
- **Each user story is independently testable**: Can validate each story works on its own
- **Manual test scripts**: Created for each user story to validate functionality
- **Commit strategy**: Commit after each task or logical group of related tasks
- **Stop at any checkpoint**: Validate story independently before proceeding
- **File paths are explicit**: Every task includes exact file path for implementation

---

## Validation Checklist

Before marking this feature complete:

- [ ] All manual test scripts execute successfully
- [ ] Agent correctly calls all 5 MCP tools (add_task, list_tasks, complete_task, update_task, delete_task)
- [ ] Agent maintains conversation context across multiple turns
- [ ] Agent handles all error scenarios with user-friendly messages
- [ ] Agent correctly interprets time-relative queries (today, tomorrow)
- [ ] Agent enforces user isolation (never accesses other users' tasks)
- [ ] Agent completes within 15 iterations for typical requests
- [ ] Response time <3 seconds for 95% of requests (excluding LLM latency)
- [ ] All functions have docstrings and type hints
- [ ] Configuration is externalized in .env file
- [ ] Quickstart.md examples match actual test output

---

**Tasks Complete**: Ready for implementation via `/sp.implement` or manual execution
