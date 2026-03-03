# Implementation Plan: FastAPI Chat Endpoint

**Branch**: `006-chat-endpoint` | **Date**: 2026-01-29 | **Spec**: [spec.md](./spec.md)

## Summary

Implement a REST endpoint at `/api/{user_id}/chat` that connects the frontend to the AI agent service with conversation persistence. The endpoint handles the complete stateless request/response cycle: authenticate user → load conversation context → persist user message → invoke AI agent → persist AI response → return response to client.

**Primary Requirement**: Create a FastAPI router that orchestrates conversation persistence and AI agent invocation while enforcing authentication and user isolation.

**Technical Approach**: Use existing FastAPI patterns (router, dependencies, Pydantic models), leverage existing Conversation/Message models, integrate with agent service from Spec 5, and enforce JWT authentication with user_id validation.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: FastAPI (already installed), SQLModel (already installed), Pydantic (already installed), agent_service (from Spec 5)
**Storage**: PostgreSQL via SQLModel (Conversation and Message models already exist)
**Testing**: Manual testing with curl/Postman, pytest for automated tests
**Target Platform**: Linux server (FastAPI backend)
**Project Type**: Web application backend API
**Performance Goals**: <5 seconds response time for 95% of requests (excluding AI agent processing time)
**Constraints**: Synchronous processing (no streaming), JWT authentication required, user isolation enforced
**Scale/Scope**: Single POST endpoint, integrates with existing auth middleware and agent service

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Check

- ✅ **Principle I (Spec-first)**: Approved spec exists at `specs/006-chat-endpoint/spec.md`
- ✅ **Principle II (Single responsibility)**: Chat endpoint has single responsibility - orchestrate conversation persistence and agent invocation
- ✅ **Principle III (Explicit contracts)**: Endpoint contract defined - POST /api/{user_id}/chat with ChatRequest/ChatResponse
- ✅ **Principle IV (Security by default)**: JWT authentication required, user_id validation enforced
- ✅ **Principle V (Determinism)**: Same inputs produce same outputs (AI agent non-determinism acknowledged)
- ✅ **Principle VI (Agentic discipline)**: All code generated via Claude Code
- ✅ **Principle VII (Stateless AI)**: Endpoint is stateless - loads context from database each request
- ✅ **Principle VIII (Conversation persistence)**: All messages persisted to database
- ✅ **Principle IX (User data isolation)**: User_id validation prevents cross-user access

**Status**: ✅ PASSED - No constitutional violations

### Post-Design Check

*To be completed after Phase 1*

## Project Structure

### Documentation (this feature)

```text
specs/006-chat-endpoint/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (minimal - most decisions clear)
├── data-model.md        # Phase 1 output (request/response models)
├── quickstart.md        # Phase 1 output (testing guide)
├── contracts/           # Phase 1 output (API contract)
│   └── chat_endpoint_contract.md
└── checklists/
    └── requirements.md  # Specification quality checklist
```

### Source Code (repository root)

```text
backend/
├── api/
│   ├── __init__.py
│   ├── auth.py              # Existing
│   ├── users.py             # Existing
│   ├── tasks.py             # Existing
│   ├── protected.py         # Existing
│   └── chat.py              # NEW - Chat endpoint router
├── models/
│   ├── task_models.py       # Existing
│   ├── conversation.py      # Existing (Spec 3)
│   ├── message.py           # Existing (Spec 3)
│   └── chat_models.py       # NEW - Request/response Pydantic models
├── services/
│   ├── agent_service.py     # Existing (Spec 5)
│   └── conversation_service.py  # NEW - Conversation business logic
├── middleware/
│   └── auth.py              # Existing - JWT authentication
├── database/
│   └── session.py           # Existing - Database sessions
├── main.py                  # UPDATE - Mount chat router
└── tests/
    └── test_chat_endpoint.py  # NEW - Endpoint tests
```

**Structure Decision**: Web application structure with FastAPI router pattern. Chat endpoint follows existing patterns in `backend/api/` directory. Business logic separated into `conversation_service.py` for maintainability.

## Complexity Tracking

> **No constitutional violations - this section is empty**

## Phase 0: Research & Discovery

### Research Questions

Most technical decisions are clear from existing codebase exploration. Minimal research needed:

1. **Agent Service Integration**: How to call `run_agent()` and extract tool_calls from AgentResponse?
2. **Transaction Management**: Best practices for database transactions with external service calls?
3. **Error Handling**: How to handle various failure scenarios gracefully?

### Research Deliverable

Create `specs/006-chat-endpoint/research.md` with:
- Agent service integration pattern (from Spec 5 implementation)
- Transaction management strategy (commit user message first, then call agent)
- Error handling patterns (HTTP status codes for different scenarios)
- Response format decisions (extract tool_calls from AgentResponse)

## Phase 1: Design & Contracts

### Data Model Design

Create `specs/006-chat-endpoint/data-model.md`:

#### Entity: ChatRequest
**Purpose**: Request payload for chat endpoint

**Attributes**:
- `message`: str - User's message content (required, 1-10,000 characters)
- `conversation_id`: Optional[int] - Existing conversation ID (optional)

**Validation Rules**:
- message must not be empty
- message must not exceed 10,000 characters
- conversation_id must be positive integer if provided

#### Entity: ChatResponse
**Purpose**: Response payload from chat endpoint

**Attributes**:
- `conversation_id`: int - ID of the conversation (new or existing)
- `response`: str - AI agent's response text
- `tool_calls`: List[str] - Names of tools called by agent

**Validation Rules**:
- conversation_id must be positive integer
- response must not be empty
- tool_calls must be list of strings (can be empty)

### API Contracts

Create `specs/006-chat-endpoint/contracts/chat_endpoint_contract.md`:

#### Endpoint: POST /api/{user_id}/chat

**Purpose**: Send a chat message and receive AI response with conversation persistence

**Path Parameters**:
- `user_id`: str - User ID (must match authenticated user from JWT token)

**Request Headers**:
- `Authorization`: Bearer {jwt_token} (required)

**Request Body** (ChatRequest):
```json
{
  "message": "Create a task to buy milk",
  "conversation_id": 123
}
```

**Response Body** (ChatResponse):
```json
{
  "conversation_id": 123,
  "response": "I've created a task titled 'Buy milk' for you.",
  "tool_calls": ["add_task"]
}
```

**Status Codes**:
- `200 OK`: Success
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: user_id mismatch
- `404 Not Found`: conversation_id not found
- `500 Internal Server Error`: Database or internal error
- `503 Service Unavailable`: Agent service unavailable

### Quickstart Guide

Create `specs/006-chat-endpoint/quickstart.md` with:
- Testing instructions using curl/Postman
- Example requests and responses
- Error scenario testing
- Integration with frontend

### Agent Context Update

Run agent context update script:
```bash
powershell.exe -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

## Phase 2: Task Breakdown

*Task breakdown will be generated by `/sp.tasks` command - NOT part of this plan*

The `/sp.tasks` command will create `specs/006-chat-endpoint/tasks.md` with:
- Detailed implementation tasks
- Test cases for each task
- Dependency ordering
- Acceptance criteria

## Post-Design Constitution Check

### Re-evaluation After Phase 1

- ✅ **Principle I (Spec-first)**: Design artifacts trace back to spec requirements
- ✅ **Principle II (Single responsibility)**: Chat endpoint maintains single responsibility
- ✅ **Principle III (Explicit contracts)**: `chat_endpoint_contract.md` defines all interfaces
- ✅ **Principle IV (Security by default)**: JWT authentication and user_id validation enforced
- ✅ **Principle V (Determinism)**: Endpoint behavior is deterministic
- ✅ **Principle VI (Agentic discipline)**: All implementation via Claude Code
- ✅ **Principle VII (Stateless AI)**: Endpoint is stateless
- ✅ **Principle VIII (Conversation persistence)**: All messages persisted to database
- ✅ **Principle IX (User data isolation)**: User_id validation prevents cross-user access

**Status**: ✅ PASSED - Design maintains constitutional compliance

## Dependencies & Integration Points

### Internal Dependencies
- **Spec 3 (Chat Database Schema)**: Conversation and Message models (already implemented)
- **Spec 4 (MCP Task Tools)**: Agent service calls these tools (already implemented)
- **Spec 5 (AI Agent Service)**: Core dependency - endpoint invokes agent service (already implemented)
- **Better Auth Integration**: JWT authentication middleware (already configured)

### External Dependencies
- **FastAPI**: Web framework (already installed)
- **SQLModel**: ORM for database operations (already installed)
- **Pydantic**: Request/response validation (already installed)

### Integration Points
- **Authentication Middleware** (`backend/middleware/auth.py`): Provides JWT validation
- **Database Session** (`backend/database/session.py`): Provides database sessions
- **Agent Service** (`backend/services/agent_service.py`): Provides `run_agent()` function
- **Conversation Model** (`backend/models/conversation.py`): Provides conversation operations
- **Message Model** (`backend/models/message.py`): Provides message operations

## Risk Analysis

### Risk 1: Agent Service Failures
**Likelihood**: Medium
**Impact**: High
**Mitigation**: Return 503 with user-friendly message; user message already persisted

### Risk 2: Database Transaction Failures
**Likelihood**: Low
**Impact**: Medium
**Mitigation**: Proper transaction management; rollback on errors

### Risk 3: Concurrent Requests to Same Conversation
**Likelihood**: Low
**Impact**: Low
**Mitigation**: Database transaction isolation handles this

## Success Metrics

- ✅ Endpoint successfully processes chat messages and returns AI responses
- ✅ Conversation history is persisted correctly (100% of successful requests)
- ✅ Authentication and authorization work correctly (100% of requests validated)
- ✅ Multi-turn conversations maintain context
- ✅ Error scenarios handled gracefully
- ✅ Response time <5 seconds for 95% of requests

## Next Steps

1. **Complete Phase 0**: Execute research tasks and create `research.md`
2. **Complete Phase 1**: Create `data-model.md`, `contracts/`, and `quickstart.md`
3. **Run `/sp.tasks`**: Generate detailed task breakdown in `tasks.md`
4. **Implementation**: Execute tasks via Claude Code
5. **Testing**: Validate with manual tests
6. **Integration**: Verify end-to-end flow with frontend

---

**Plan Status**: ✅ COMPLETE - Ready for Phase 0 research execution
**Next Command**: Begin Phase 0 research or proceed to `/sp.tasks` if research is complete
