# Tasks: Frontend Chat Interface

**Input**: Design documents from `/specs/007-frontend-chat-interface/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `frontend-app/` at repository root
- All paths are absolute from repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project structure and dependencies

- [x] T001 Verify Next.js 16.1.1 project structure in frontend-app/
- [x] T002 Verify TypeScript configuration and path aliases (@/* mapping)
- [x] T003 Verify Tailwind CSS 4.x configuration in frontend-app/tailwind.config.js

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 [P] Create token-storage module in frontend-app/lib/token-storage.ts with JWT token management (getAccessToken, setAccessToken, getRefreshToken, setRefreshToken, clearTokens, isTokenExpired, verifyAndValidateToken methods)
- [x] T005 [P] Create task-service module in frontend-app/lib/api/task-service.ts with task API wrapper (getUserTasks, getTaskStats, createTask, updateTask, deleteTask, toggleTaskCompletion methods)
- [x] T006 [P] Create utils module in frontend-app/lib/utils.ts with cn() function for className merging using clsx and tailwind-merge
- [x] T007 Add chat-related TypeScript interfaces to frontend-app/types.ts (ChatMessage, ConversationState, ChatRequest, ChatResponse, UseChatReturn)
- [x] T008 Add postChat method to frontend-app/services/api-client.js for POST /api/{user_id}/chat with JWT authentication and error handling

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Chat Interaction (Priority: P1) 🎯 MVP

**Goal**: User can open chat interface, send messages, and receive AI responses with conversation history displayed

**Independent Test**: Open chat interface from dashboard, send "What tasks do I have?", verify AI response appears in chat history

### Implementation for User Story 1

- [x] T009 [P] [US1] Create use-chat custom hook in frontend-app/hooks/use-chat.ts with state management (messages, conversationId, isLoading, error, sendMessage, startNewConversation, clearError functions)
- [x] T010 [P] [US1] Create ChatMessage component in frontend-app/components/chat/chat-message.tsx to display individual message bubbles with role-based styling (user vs assistant)
- [x] T011 [P] [US1] Create ChatLoading component in frontend-app/components/chat/chat-loading.tsx with typing indicator using LoadingSpinner
- [x] T012 [P] [US1] Create ChatHeader component in frontend-app/components/chat/chat-header.tsx with title and close button
- [x] T013 [US1] Create ChatMessageList component in frontend-app/components/chat/chat-message-list.tsx to display message history with scroll-to-bottom behavior
- [x] T014 [US1] Create ChatInput component in frontend-app/components/chat/chat-input.tsx with message input form, validation (non-empty, max 10,000 chars), and submit handling
- [x] T015 [US1] Create ChatInterface main component in frontend-app/components/chat/chat-interface.tsx integrating all chat subcomponents with use-chat hook
- [x] T016 [US1] Implement optimistic updates in use-chat hook (show user message immediately with 'sending' status before API response)
- [x] T017 [US1] Implement error handling in use-chat hook for network errors, auth errors (401), and server errors (500, 503)
- [x] T018 [US1] Add message timestamps and format display in ChatMessage component

**Checkpoint**: At this point, User Story 1 should be fully functional - user can send messages and receive AI responses in a chat interface

---

## Phase 4: User Story 2 - Multi-Turn Conversation Persistence (Priority: P1)

**Goal**: Conversation context is maintained across multiple messages so AI understands references to previous messages

**Independent Test**: Send "Create a task to buy milk", then send "Mark it as complete", verify AI understands "it" refers to the milk task

### Implementation for User Story 2

- [x] T019 [US2] Enhance use-chat hook to manage conversation_id state (initialize as null, update from backend response)
- [x] T020 [US2] Update sendMessage function in use-chat hook to include conversation_id in API requests (null for first message, existing ID for follow-ups)
- [x] T021 [US2] Add "New Conversation" button to ChatHeader component that calls startNewConversation to reset conversation_id and clear messages
- [x] T022 [US2] Implement optional conversation_id persistence in localStorage (save on update, load on mount) in use-chat hook
- [x] T023 [US2] Add conversation context indicator in ChatHeader showing conversation status (new vs continuing)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - users can have multi-turn conversations with context

---

## Phase 5: User Story 3 - Automatic Task List Refresh (Priority: P1)

**Goal**: Task list automatically refreshes when AI modifies tasks through chat (create, update, delete, complete)

**Independent Test**: Send "Create a task to buy milk" in chat, verify task list automatically updates to show new task within 1 second

### Implementation for User Story 3

- [x] T024 [P] [US3] Create use-task-refresh custom hook in frontend-app/hooks/use-task-refresh.ts that listens for 'tasks-updated' event and calls refresh callback with proper cleanup
- [x] T025 [US3] Update ChatInterface component (or use-chat hook) to dispatch 'tasks-updated' CustomEvent when response.tool_calls includes task modification operations (add_task, update_task, delete_task, complete_task)
- [x] T026 [US3] Modify task list page in frontend-app/app/dashboard/tasks/page.tsx to use use-task-refresh hook and refresh tasks when event is received
- [x] T027 [US3] Add visual feedback in ChatInterface when task modification is detected (e.g., "Task list updated" message or icon)
- [x] T028 [US3] Test event dispatch and listener cleanup to prevent memory leaks on component unmount

**Checkpoint**: All P1 user stories complete - chat works with full context and auto-refreshes task list

---

## Phase 6: User Story 4 - Chat Interface Layout and Accessibility (Priority: P2)

**Goal**: Chat interface is well-integrated into dashboard, accessible, and works on all screen sizes

**Independent Test**: Open chat on desktop (sidebar), tablet (modal), and mobile (320px width), verify usability on all sizes

### Implementation for User Story 4

- [x] T029 [US4] Modify dashboard layout in frontend-app/app/dashboard/layout.tsx to add chat toggle button in header or sidebar
- [x] T030 [US4] Add chat open/close state management to dashboard layout using useState
- [x] T031 [US4] Integrate ChatInterface component into dashboard layout with conditional rendering based on open state
- [x] T032 [US4] Implement responsive behavior: sidebar on desktop (md: and above), modal on mobile (below md:)
- [x] T033 [US4] Style chat sidebar with fixed positioning, slide-in animation, and proper z-index to not obstruct task list
- [x] T034 [US4] Style chat modal for mobile with full-screen overlay and slide-up animation
- [x] T035 [US4] Add keyboard accessibility (Escape to close, focus management when opening/closing)
- [x] T036 [US4] Test responsive layout on screen sizes from 320px to 1920px width
- [x] T037 [US4] Add ARIA labels and roles for screen reader accessibility

**Checkpoint**: All user stories complete - chat is fully integrated and accessible

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and enhance overall quality

- [x] T038 [P] Add empty state message in ChatMessageList when no messages exist ("Start a conversation with your AI assistant")
- [x] T039 [P] Add error message display in ChatInterface for failed API requests with user-friendly messages
- [x] T040 [P] Prevent sending empty or whitespace-only messages in ChatInput validation
- [x] T041 [P] Add scroll-to-bottom on new messages in ChatMessageList using useEffect and scrollIntoView
- [x] T042 [P] Add loading state to send button in ChatInput (disabled while isLoading is true)
- [x] T043 [P] Style message bubbles with distinct colors for user (blue) vs assistant (gray) messages
- [x] T044 [P] Add message status indicators in ChatMessage (sending spinner, sent checkmark, error icon)
- [x] T045 Add performance optimization for long conversations (100+ messages) using React.memo on ChatMessage
- [x] T046 Add error boundary around ChatInterface to catch and display component errors gracefully
- [x] T047 Test all error scenarios from quickstart.md (network errors, expired tokens, backend errors)
- [x] T048 Test all user scenarios from spec.md acceptance criteria
- [x] T049 Verify performance goals: chat load < 2s, message send/receive < 5s, task refresh < 1s
- [x] T050 Run quickstart.md validation and update documentation with any findings

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (Phase 3): Can start after Foundational - No dependencies on other stories
  - User Story 2 (Phase 4): Depends on User Story 1 completion (enhances conversation management)
  - User Story 3 (Phase 5): Can start after Foundational - Independent of US1/US2 but typically done after for testing
  - User Story 4 (Phase 6): Depends on User Story 1 completion (integrates chat into dashboard)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Depends on User Story 1 (enhances existing chat with conversation persistence)
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - Independent but typically done after US1 for integration testing
- **User Story 4 (P2)**: Depends on User Story 1 (integrates chat into dashboard layout)

### Within Each User Story

- **User Story 1**: Hook and components can be built in parallel, then integrated
- **User Story 2**: Sequential enhancements to existing US1 implementation
- **User Story 3**: Hook and event dispatch can be built in parallel, then integrated
- **User Story 4**: Layout modifications are sequential (state → integration → responsive → accessibility)

### Parallel Opportunities

- **Phase 2 (Foundational)**: T004, T005, T006 can run in parallel (different files)
- **Phase 3 (US1)**: T009, T010, T011, T012 can run in parallel (different components)
- **Phase 5 (US3)**: T024, T025 can run in parallel (different files)
- **Phase 7 (Polish)**: T038-T044 can run in parallel (different concerns)

---

## Parallel Example: User Story 1

```bash
# Launch foundational services in parallel:
Task T004: "Create token-storage module in frontend-app/lib/token-storage.ts"
Task T005: "Create task-service module in frontend-app/lib/api/task-service.ts"
Task T006: "Create utils module in frontend-app/lib/utils.ts"

# Launch US1 components in parallel:
Task T009: "Create use-chat hook in frontend-app/hooks/use-chat.ts"
Task T010: "Create ChatMessage component in frontend-app/components/chat/chat-message.tsx"
Task T011: "Create ChatLoading component in frontend-app/components/chat/chat-loading.tsx"
Task T012: "Create ChatHeader component in frontend-app/components/chat/chat-header.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (verify structure)
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (basic chat interaction)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Open chat interface
   - Send message "What tasks do I have?"
   - Verify AI response appears
   - Verify conversation history displays
5. Deploy/demo if ready

### Incremental Delivery (Recommended)

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP! ✅)
3. Add User Story 2 → Test independently → Deploy/Demo (Context persistence ✅)
4. Add User Story 3 → Test independently → Deploy/Demo (Auto-refresh ✅)
5. Add User Story 4 → Test independently → Deploy/Demo (Full UX ✅)
6. Add Polish → Final testing → Production release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (T009-T018)
   - Developer B: User Story 3 (T024-T028) - can work in parallel
3. After US1 complete:
   - Developer A: User Story 2 (T019-T023)
   - Developer B: User Story 4 (T029-T037)
4. Team: Polish together (T038-T050)

---

## Task Summary

**Total Tasks**: 50
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 5 tasks (BLOCKING)
- Phase 3 (US1 - Basic Chat): 10 tasks
- Phase 4 (US2 - Conversation Persistence): 5 tasks
- Phase 5 (US3 - Auto-Refresh): 5 tasks
- Phase 6 (US4 - Layout & Accessibility): 9 tasks
- Phase 7 (Polish): 13 tasks

**Parallel Opportunities**: 15 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phases 1-3 (18 tasks) deliver basic chat interaction

**Full P1 Scope**: Phases 1-5 (28 tasks) deliver all P1 user stories

**Complete Feature**: All 50 tasks deliver full feature with polish

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Backend chat endpoint (spec 006) must be running for testing
- Use existing UI components (Button, Card, LoadingSpinner) from frontend-app/components/ui/
- Follow existing patterns from use-auth.ts for hook structure
- All styling uses Tailwind CSS 4.x utilities
- No automated tests (manual testing per spec)

---

## Testing Checklist (from quickstart.md)

After implementation, verify:

- [ ] Basic chat: Send message, receive response
- [ ] Multi-turn: Send follow-up with pronoun reference, AI understands context
- [ ] Task creation: Create task via chat, task list auto-refreshes
- [ ] Task update: Update task via chat, changes appear in list
- [ ] Task deletion: Delete task via chat, task removed from list
- [ ] Empty message: Cannot send empty message
- [ ] Network error: Backend down, error message displayed
- [ ] Expired token: Token expires, automatic refresh or redirect to sign-in
- [ ] Responsive: Works on desktop (1920px), tablet (768px), mobile (320px)
- [ ] Accessibility: Keyboard navigation, screen reader support
- [ ] Performance: Chat loads < 2s, messages < 5s, refresh < 1s
- [ ] Long conversation: 100+ messages without degradation
