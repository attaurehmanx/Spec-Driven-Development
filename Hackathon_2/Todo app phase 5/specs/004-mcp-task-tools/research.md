# Research: MCP Task Tools Implementation

**Feature**: 004-mcp-task-tools
**Date**: 2026-01-29
**Purpose**: Technical research and decision-making for MCP server implementation

## Overview

This document captures research findings and technical decisions for implementing an MCP (Model Context Protocol) server that exposes task management tools to AI agents.

## Technical Decisions

### 1. MCP Python SDK Selection

**Decision**: Use the Official Anthropic MCP Python SDK (`mcp` package version 1.10.1)

**Rationale**:
- Already installed in the environment
- Official support from Anthropic with active development
- Modern API using decorators (`@mcp.tool()`) for clean tool definitions
- Automatic schema generation from Python type hints
- Native async/await support throughout the API
- Built-in validation using Pydantic models

**Alternatives Considered**:
- Community MCP implementations: Less mature, inconsistent APIs
- Custom protocol implementation: Too much overhead, reinventing the wheel

**Implementation Notes**:
- Use `FastMCP` class for simplified server creation
- Leverage automatic tool schema generation from docstrings
- Use Pydantic models for input validation

---

### 2. MCP Server Architecture

**Decision**: Standalone MCP Server Module (Separate from FastAPI)

**Architecture**:
```
backend/
├── mcp_server/
│   ├── __init__.py
│   ├── server.py          # MCP server instance
│   ├── tools/
│   │   ├── __init__.py
│   │   └── task_tools.py  # Task CRUD tools
│   └── run_mcp.py         # Entry point
├── api/                   # Existing FastAPI routes
├── database/              # Shared database session
└── models/                # Shared SQLModel models
```

**Rationale**:
- **Separation of concerns**: MCP server handles AI agent interactions; FastAPI handles web API
- **Shared resources**: Both can use the same database session factory and models
- **Independent scaling**: Can run MCP server and FastAPI separately if needed
- **Clear boundaries**: MCP tools focus on AI-friendly operations; REST API focuses on web clients
- **Different protocols**: MCP uses stdio/SSE transport, not HTTP REST

**Alternatives Considered**:
- Integrate into FastAPI: Would mix concerns, different transport protocols, harder to maintain
- Completely separate service: Would duplicate database connection logic, more complex deployment

**Implementation Notes**:
- Share `database/session.py` for database connections
- Share `models/task_models.py` for data models
- Keep MCP server lifecycle independent from FastAPI

---

### 3. Async/Await Patterns

**Decision**: Use Async Tools with Async Database Sessions

**Pattern**:
- Convert database engine to async using `asyncpg` driver
- Use `AsyncSession` from SQLModel for database operations
- Define all MCP tools as async functions
- Use `@asynccontextmanager` for database connection lifecycle

**Rationale**:
- **Non-blocking**: Async operations prevent blocking the MCP server
- **Better performance**: Can handle multiple tool calls concurrently
- **SQLModel compatibility**: SQLModel supports async sessions via SQLAlchemy
- **MCP SDK native**: The SDK is designed for async operations

**Alternatives Considered**:
- Synchronous tools: Would block on database operations, poor performance
- Mixed sync/async: Inconsistent patterns, harder to maintain

**Implementation Notes**:
- Install `asyncpg` for async PostgreSQL driver
- Convert database URL: `postgresql://` → `postgresql+asyncpg://`
- Use `create_async_engine` instead of `create_engine`
- All tool functions must be `async def`

**Dependencies**:
```bash
pip install asyncpg
```

---

### 4. User Context Passing

**Decision**: Explicit `user_id` Parameter on Every Tool

**Pattern**:
- Every tool function includes `user_id: str` as the first parameter
- Tools validate user existence before performing operations
- Tools verify user ownership before modifying tasks
- No hidden context or global state

**Rationale**:
- **Explicit is better than implicit**: Clear that every operation requires user context
- **No hidden state**: User ID is visible in tool signatures
- **Validation at tool level**: Each tool validates user ownership
- **Matches existing pattern**: FastAPI routes already use this pattern
- **Security**: Forces explicit user validation on every operation

**Alternatives Considered**:
- Context objects: MCP doesn't have built-in authentication context
- Global state: Would be error-prone and hard to test
- Implicit user from connection: Not supported by MCP protocol

**Implementation Notes**:
- AI agent must provide user_id in every tool call
- Validate user exists in database before operations
- Verify task ownership: `task.user_id == user_id`
- Raise descriptive errors for unauthorized access

---

### 5. Tool Response Format

**Decision**: Structured JSON with Consistent Schema

**Standard Response Patterns**:

**Success Response**:
```json
{
  "task_id": 123,
  "status": "created|updated|completed|deleted",
  "title": "Task title",
  "description": "Optional description",
  "completed": false,
  "created_at": "2026-01-29T10:30:00Z",
  "updated_at": "2026-01-29T10:30:00Z"
}
```

**List Response**:
```json
{
  "tasks": [
    {"task_id": 1, "title": "...", "completed": false},
    {"task_id": 2, "title": "...", "completed": true}
  ],
  "count": 2,
  "filter": "all|pending|completed"
}
```

**Error Response** (via exceptions):
```python
raise ValueError("Task not found")
raise ValueError("User does not own this task")
raise ValueError("Title cannot be empty")
```

**Rationale**:
- **Consistent structure**: AI can reliably parse responses
- **Rich information**: Includes all relevant task data
- **ISO 8601 dates**: Standard format for timestamps
- **Clear status indicators**: Makes operation result obvious
- **Error handling via exceptions**: MCP SDK converts to proper error responses

**Alternatives Considered**:
- Plain text responses: Hard for AI to parse reliably
- Minimal responses: Would require additional queries for full data
- Custom error objects: MCP SDK handles exception conversion well

**Implementation Notes**:
- Use `.isoformat()` for datetime serialization
- Include operation status in every response
- Raise exceptions for errors (MCP SDK handles formatting)
- Return dictionaries (JSON-serializable)

---

### 6. Error Handling

**Decision**: Layered Error Handling with Descriptive Messages

**Error Hierarchy**:
```python
class TaskToolError(Exception):
    """Base exception for task tool errors."""
    pass

class TaskNotFoundError(TaskToolError):
    """Task does not exist."""
    pass

class UnauthorizedTaskAccessError(TaskToolError):
    """User does not own the task."""
    pass

class ValidationError(TaskToolError):
    """Input validation failed."""
    pass
```

**Error Handling Pattern**:
1. **Input validation**: Check parameters before database operations
2. **Existence checks**: Verify entities exist
3. **Authorization checks**: Verify user ownership
4. **Database errors**: Catch and wrap database exceptions
5. **Logging**: Log errors for debugging without exposing internals

**Rationale**:
- **Clear error messages**: AI can understand and communicate errors to users
- **Typed exceptions**: Different error types for different failure modes
- **Logging**: Capture errors for debugging
- **Graceful degradation**: Catch unexpected errors and provide safe fallback
- **Security**: Don't expose internal details in error messages

**Alternatives Considered**:
- Generic exceptions: Less informative for AI and users
- Error codes: Less readable than descriptive messages
- Silent failures: Would confuse AI and users

**Implementation Notes**:
- Validate inputs before database operations
- Use descriptive error messages that help AI understand the problem
- Log unexpected errors with full stack traces
- Re-raise custom exceptions, wrap unexpected ones

---

### 7. Testing Strategy

**Decision**: Multi-Layer Testing Approach

**Test Layers**:

1. **Unit Tests** (Mock Database):
   - Test business logic in isolation
   - Mock database sessions
   - Fast execution
   - Test error paths

2. **Integration Tests** (Real Database):
   - Test full tool lifecycle
   - Use test database
   - Verify database interactions
   - Test user isolation

3. **MCP Inspector** (Manual Testing):
   - Interactive tool testing
   - Verify AI-like interactions
   - Test tool discovery
   - Validate schemas

**Test Structure**:
```
backend/mcp_server/tests/
├── __init__.py
├── test_task_tools_unit.py      # Unit tests
├── test_task_tools_integration.py  # Integration tests
└── conftest.py                  # Pytest fixtures
```

**Rationale**:
- **Unit tests**: Fast feedback, test logic in isolation
- **Integration tests**: Verify database interactions work correctly
- **Manual testing**: Validate AI agent experience
- **Comprehensive coverage**: All error paths and edge cases tested

**Alternatives Considered**:
- Only integration tests: Too slow for rapid development
- Only unit tests: Wouldn't catch database integration issues
- No testing: Unacceptable for production code

**Implementation Notes**:
- Use `pytest` for test framework
- Use `pytest-asyncio` for async test support
- Create fixtures for test users and database sessions
- Aim for 100% coverage of tool functions
- Test all error paths and edge cases

**Dependencies**:
```bash
pip install pytest pytest-asyncio pytest-cov
```

---

## Integration Points

### Database Connection

**Shared Resource**: `backend/database/session.py`

**Modifications Needed**:
- Add async engine creation function
- Export both sync and async session factories
- Maintain existing sync engine for FastAPI compatibility

**Pattern**:
```python
# database/session.py
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession

# Existing sync engine (keep for FastAPI)
engine = create_engine(database_url, ...)

# New async engine (for MCP server)
async_engine = create_async_engine(
    database_url.replace("postgresql://", "postgresql+asyncpg://"),
    echo=False,
    pool_pre_ping=True
)

async def get_async_session():
    async with AsyncSession(async_engine) as session:
        yield session
```

---

### Data Models

**Shared Resource**: `backend/models/task_models.py`

**No Modifications Needed**: Existing models work with both sync and async sessions

**Models Used**:
- `User`: For user validation
- `Task`: For task CRUD operations
- `TaskCreate`, `TaskUpdate`: For input validation (can reuse or create Pydantic equivalents)

---

### Configuration

**Shared Resource**: `backend/config/settings.py`

**No Modifications Needed**: Existing settings work for MCP server

**Settings Used**:
- `DATABASE_URL`: PostgreSQL connection string
- Environment variables loaded via `python-dotenv`

---

## Deployment Considerations

### Running the MCP Server

**Development**:
```bash
# Run MCP server standalone
python backend/mcp_server/run_mcp.py

# Run with MCP Inspector for testing
npx -y @modelcontextprotocol/inspector
```

**Production**:
- MCP server runs as separate process from FastAPI
- Can use process manager (systemd, supervisor, PM2)
- Shares database connection pool with FastAPI
- Logs to separate file for debugging

---

### Environment Variables

**Required**:
- `DATABASE_URL`: PostgreSQL connection string (existing)

**Optional**:
- `MCP_LOG_LEVEL`: Logging level (default: INFO)
- `MCP_SERVER_NAME`: Server name for identification (default: task-tools)

---

## Security Considerations

### User Isolation

**Enforcement**:
- Every tool validates `user_id` parameter
- Database queries filter by `user_id`
- Ownership verification before modifications
- No cross-user data access

**Pattern**:
```python
# Verify user exists
user = await session.get(User, user_id)
if not user:
    raise ValueError(f"User {user_id} not found")

# Verify task ownership
task = await session.get(Task, task_id)
if task.user_id != user_id:
    raise ValueError(f"Task {task_id} does not belong to user {user_id}")
```

---

### Input Validation

**Validation Rules**:
- Title: Required, non-empty, max 255 characters
- Description: Optional, max 10,000 characters
- Status filter: Must be "all", "pending", or "completed"
- Task ID: Must be positive integer
- User ID: Must be valid UUID string

**Implementation**:
- Use Pydantic models for automatic validation
- Add custom validation in tool functions
- Raise `ValidationError` for invalid inputs

---

## Performance Considerations

### Database Connection Pooling

**Strategy**:
- Share connection pool between FastAPI and MCP server
- Use async connection pool for MCP server
- Configure pool size based on expected load

**Configuration**:
```python
async_engine = create_async_engine(
    database_url,
    pool_size=10,
    max_overflow=20,
    pool_timeout=30,
    pool_pre_ping=True
)
```

---

### Query Optimization

**Patterns**:
- Use indexes on `user_id` and `task.id` (already exist)
- Filter by user_id in all queries
- Use `select()` for efficient queries
- Avoid N+1 queries

---

## Monitoring and Logging

### Logging Strategy

**Log Levels**:
- INFO: Tool calls, successful operations
- WARNING: Validation failures, authorization failures
- ERROR: Database errors, unexpected exceptions

**Log Format**:
```python
logger.info(f"Tool called: add_task(user_id={user_id}, title={title})")
logger.warning(f"Unauthorized access attempt: user {user_id} tried to access task {task_id}")
logger.error(f"Database error in list_tasks: {str(e)}", exc_info=True)
```

---

### Metrics

**Track**:
- Tool call counts by tool name
- Success/failure rates
- Response times
- Database query times

---

## Next Steps

1. **Phase 1: Design & Contracts**
   - Create data-model.md with tool input/output schemas
   - Create contracts/ with OpenAPI-style tool specifications
   - Create quickstart.md with setup and testing instructions

2. **Phase 2: Implementation Planning**
   - Fill out plan.md with complete implementation strategy
   - Define file structure and module organization
   - Create task breakdown for implementation

3. **Phase 3: Implementation**
   - Create MCP server module structure
   - Implement async database session
   - Implement 5 task tools
   - Write comprehensive tests
   - Create documentation

---

## References

- [MCP Python SDK Documentation](https://github.com/modelcontextprotocol/python-sdk)
- [FastMCP Documentation](https://github.com/jlowin/fastmcp)
- [SQLModel Async Documentation](https://sqlmodel.tiangolo.com/advanced/async/)
- [Pydantic Validation](https://docs.pydantic.dev/latest/)
