# MCP Task Tools Server

A Model Context Protocol (MCP) server that exposes task management tools to AI agents, enabling conversational task management through natural language.

## Overview

This MCP server provides 5 task management tools that allow AI agents to perform full CRUD operations on tasks:

1. **add_task** - Create new tasks with title and optional description
2. **list_tasks** - Retrieve tasks with optional status filtering (all/pending/completed)
3. **complete_task** - Mark tasks as completed
4. **update_task** - Update task title and/or description
5. **delete_task** - Permanently delete tasks

All tools enforce strict user isolation - users can only access their own tasks.

## Features

- **Full CRUD Operations**: Create, read, update, delete tasks
- **User Isolation**: All operations scoped to authenticated user
- **Async/Await**: Non-blocking database operations using asyncpg
- **Input Validation**: Pydantic models with comprehensive validation
- **Error Handling**: Clear, AI-friendly error messages
- **Performance**: Optimized for 1000+ tasks per user
- **Security**: Ownership verification on all modifications

## Architecture

```
backend/
├── mcp_server/
│   ├── server.py              # FastMCP server instance
│   ├── run_mcp.py             # Entry point script
│   └── tools/
│       └── task_tools.py      # 5 task management tools
├── database/
│   └── session.py             # Async database session support
└── models/
    └── task_models.py         # Task and User models (existing)
```

## Installation

### Prerequisites

- Python 3.10+
- PostgreSQL database (Neon Serverless recommended)
- Existing Task and User models from Phase 2

### Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

Required packages:
- `mcp[cli]` - Official Anthropic MCP SDK
- `asyncpg` - Async PostgreSQL driver
- `sqlmodel` - ORM (existing)
- `pydantic` - Input validation (existing)

## Configuration

Set the database URL in your `.env` file:

```env
DATABASE_URL=postgresql://user:password@host:port/database?sslmode=require
```

The MCP server will automatically convert this to use asyncpg driver.

## Running the Server

### Development Mode

```bash
cd backend
python mcp_server/run_mcp.py
```

The server will start and listen for MCP tool calls.

### Production Deployment

#### Option 1: Systemd Service (Linux)

Create `/etc/systemd/system/mcp-task-tools.service`:

```ini
[Unit]
Description=MCP Task Tools Server
After=network.target postgresql.service

[Service]
Type=simple
User=www-data
WorkingDirectory=/var/www/backend
Environment="DATABASE_URL=postgresql://..."
ExecStart=/usr/bin/python3 mcp_server/run_mcp.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Start the service:

```bash
sudo systemctl enable mcp-task-tools
sudo systemctl start mcp-task-tools
sudo systemctl status mcp-task-tools
```

#### Option 2: Docker Container

```dockerfile
FROM python:3.10-slim
WORKDIR /app
COPY backend/ /app/
RUN pip install -r requirements.txt
CMD ["python", "mcp_server/run_mcp.py"]
```

Build and run:

```bash
docker build -t mcp-task-tools .
docker run -e DATABASE_URL="postgresql://..." mcp-task-tools
```

## Tool Usage

### add_task

Create a new task for the user.

**Parameters:**
- `user_id` (string, required): User's UUID
- `title` (string, required): Task title (1-255 characters)
- `description` (string, optional): Task description (max 10,000 characters)

**Example:**
```python
result = await add_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    title="Buy groceries",
    description="Milk, eggs, bread"
)
```

**Response:**
```json
{
  "task_id": 123,
  "status": "created",
  "title": "Buy groceries",
  "description": "Milk, eggs, bread",
  "completed": false,
  "created_at": "2026-01-29T10:30:00Z",
  "updated_at": "2026-01-29T10:30:00Z"
}
```

### list_tasks

Retrieve user's tasks with optional status filtering.

**Parameters:**
- `user_id` (string, required): User's UUID
- `status` (string, optional): Filter by "all", "pending", or "completed" (default: "all")

**Example:**
```python
result = await list_tasks(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    status="pending"
)
```

**Response:**
```json
{
  "tasks": [
    {
      "task_id": 123,
      "title": "Buy groceries",
      "description": "Milk, eggs, bread",
      "completed": false,
      "created_at": "2026-01-29T10:30:00Z",
      "updated_at": "2026-01-29T10:30:00Z"
    }
  ],
  "count": 1,
  "filter": "pending"
}
```

### complete_task

Mark a task as completed.

**Parameters:**
- `user_id` (string, required): User's UUID
- `task_id` (integer, required): Task ID to complete

**Example:**
```python
result = await complete_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=123
)
```

### update_task

Update task title and/or description.

**Parameters:**
- `user_id` (string, required): User's UUID
- `task_id` (integer, required): Task ID to update
- `title` (string, optional): New title (1-255 characters)
- `description` (string, optional): New description (max 10,000 characters)

**Note:** At least one of title or description must be provided.

**Example:**
```python
result = await update_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=123,
    title="Buy organic groceries"
)
```

### delete_task

Permanently delete a task.

**Parameters:**
- `user_id` (string, required): User's UUID
- `task_id` (integer, required): Task ID to delete

**Example:**
```python
result = await delete_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=123
)
```

## Testing

### Manual Testing with MCP Inspector

```bash
# Terminal 1: Start MCP server
cd backend
python mcp_server/run_mcp.py

# Terminal 2: Run MCP Inspector
npx @modelcontextprotocol/inspector
```

The Inspector will discover all 5 tools and allow interactive testing.

### Automated Tests

Run the test scripts:

```bash
cd backend

# Test individual tools
python test_add_task.py
python test_list_tasks.py
python test_complete_task.py
python test_update_task.py
python test_delete_task.py

# Test full lifecycle
python test_full_lifecycle.py

# Test async database connection
python test_async_db.py
```

## Security

### User Isolation

Every tool operation:
1. Validates user_id parameter
2. Verifies user exists in database
3. Filters all queries by user_id
4. Verifies task ownership before modifications

**No cross-user data access is possible.**

### Input Validation

- Pydantic models validate all inputs
- Title: 1-255 characters, non-empty
- Description: max 10,000 characters
- Task IDs: positive integers only
- Status filters: "all", "pending", or "completed" only

### SQL Injection Prevention

- SQLModel ORM uses parameterized queries
- No raw SQL execution
- All inputs sanitized automatically

## Performance

### Benchmarks

- add_task: < 100ms
- list_tasks: < 500ms for 100 tasks, < 2s for 1000 tasks
- complete_task: < 100ms
- update_task: < 100ms
- delete_task: < 100ms

### Optimization

- Async operations prevent blocking
- Database connection pooling (10 connections, 20 overflow)
- Indexed queries on user_id and task_id
- Pre-ping enabled for connection health checks

## Error Handling

All tools return clear, AI-friendly error messages:

- **ValueError**: User not found, validation errors
- **TaskNotFoundError**: Task does not exist
- **UnauthorizedTaskAccessError**: User doesn't own the task
- **ValidationError**: Input validation failed

Example error response:
```json
{
  "error": {
    "code": "TaskNotFoundError",
    "message": "Task 123 not found"
  }
}
```

## Logging

Logs are written to console and can be configured in `server.py`:

```python
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
```

Log levels:
- **INFO**: Tool calls, successful operations
- **WARNING**: User/task not found, validation failures
- **ERROR**: Exceptions, unexpected errors

## Monitoring

Track these metrics in production:

- Tool call counts by tool name
- Success/failure rates
- Response times (p50, p95, p99)
- Database query times
- Error rates by error type
- Active database connections

## Troubleshooting

### Server won't start

Check:
1. Database URL is correct in `.env`
2. Database is accessible
3. All dependencies installed: `pip install -r requirements.txt`
4. Python version is 3.10+

### Tools not discoverable

Verify:
1. Server is running: `python mcp_server/run_mcp.py`
2. No import errors in logs
3. All 5 tools are decorated with `@mcp.tool()`

### Database connection errors

Check:
1. PostgreSQL is running
2. Database URL format: `postgresql://user:pass@host:port/db`
3. SSL mode is correct for your database
4. Connection pool not exhausted (check pool_size in session.py)

### Slow performance

Optimize:
1. Increase connection pool size in `database/session.py`
2. Add database indexes on frequently queried columns
3. Check database query performance with `echo=True` in engine config
4. Monitor database connection count

## Contributing

When adding new tools:

1. Define Pydantic input model in `task_tools.py`
2. Implement tool function with `@mcp.tool()` decorator
3. Add user_id validation
4. Implement business logic with async database operations
5. Return standardized JSON response
6. Add comprehensive error handling
7. Write tests
8. Update this README

## License

Part of the Multi-User Task Management Web Application project.

## Support

For issues or questions, refer to:
- Specification: `specs/004-mcp-task-tools/spec.md`
- Implementation Plan: `specs/004-mcp-task-tools/plan.md`
- Tool Contracts: `specs/004-mcp-task-tools/contracts/`
