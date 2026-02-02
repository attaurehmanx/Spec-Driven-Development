# Quickstart: MCP Task Tools

**Feature**: 004-mcp-task-tools
**Date**: 2026-01-29
**Purpose**: Setup and testing instructions for MCP Task Tools implementation

## Prerequisites

- Python 3.10+ installed
- PostgreSQL database running (Neon Serverless or local)
- Backend dependencies installed
- Database connection configured in `backend/.env`
- Existing Task and User models from Phase 2

## Environment Setup

Ensure your `backend/.env` file contains:

```env
DATABASE_URL=postgresql://username:password@host:port/database
JWT_SECRET_KEY=your-secret-key
JWT_ALGORITHM=HS256
```

## Step 1: Install MCP Dependencies

Install the required packages for MCP server:

```bash
# Navigate to backend directory
cd backend

# Install MCP SDK and async PostgreSQL driver
pip install "mcp[cli]" asyncpg pytest pytest-asyncio pytest-cov
```

**Expected Output**: Packages installed successfully

## Step 2: Verify Existing Backend

Before adding MCP server, verify the existing backend works:

```bash
# Test database connection
python -c "from database.session import engine; print('Database connected:', engine.url)"

# Verify existing models
python -c "from models.task_models import User, Task; print('Models imported successfully')"
```

**Expected Output**: No errors, confirmation messages printed

## Step 3: Create MCP Server Directory Structure

Create the MCP server module structure:

```bash
# From backend directory
mkdir -p mcp_server/tools
mkdir -p mcp_server/tests

# Create __init__.py files
touch mcp_server/__init__.py
touch mcp_server/tools/__init__.py
touch mcp_server/tests/__init__.py
```

**Expected Structure**:
```
backend/
├── mcp_server/
│   ├── __init__.py
│   ├── server.py          # To be created
│   ├── tools/
│   │   ├── __init__.py
│   │   └── task_tools.py  # To be created
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── conftest.py    # To be created
│   │   └── test_task_tools.py  # To be created
│   └── run_mcp.py         # To be created
```

## Step 4: Update Database Session for Async Support

Modify `backend/database/session.py` to add async engine:

```python
# Add these imports at the top
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

# After the existing sync engine, add:
# Convert database URL for async driver
async_database_url = database_url.replace("postgresql://", "postgresql+asyncpg://")

# Create async engine
async_engine = create_async_engine(
    async_database_url,
    echo=False,
    pool_pre_ping=True,
    pool_recycle=300,
    pool_size=10,
    max_overflow=20,
    pool_timeout=30
)

# Async session factory
async_session_maker = sessionmaker(
    async_engine,
    class_=AsyncSession,
    expire_on_commit=False
)

async def get_async_session():
    """Dependency to get an async database session."""
    async with async_session_maker() as session:
        try:
            yield session
        except Exception as e:
            logging.error(f"Async database session error: {str(e)}")
            await session.rollback()
            raise
```

## Step 5: Implement MCP Server

Create `backend/mcp_server/server.py`:

```python
from mcp.server.fastmcp import FastMCP
from contextlib import asynccontextmanager
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from database.session import async_engine

@asynccontextmanager
async def app_lifespan(server: FastMCP):
    """Manage MCP server lifecycle."""
    # Startup
    print("MCP Task Tools server starting...")
    yield
    # Shutdown
    print("MCP Task Tools server shutting down...")
    await async_engine.dispose()

# Create MCP server instance
mcp = FastMCP(
    "task-tools",
    lifespan=app_lifespan
)

# Import tools (will be registered via decorators)
from tools import task_tools
```

## Step 6: Implement Task Tools

Create `backend/mcp_server/tools/task_tools.py` with all 5 tools:

```python
from mcp_server.server import mcp
from sqlmodel import select
from sqlalchemy.ext.asyncio import AsyncSession
from database.session import async_engine
from models.task_models import User, Task
from datetime import datetime
from typing import Optional

# Custom exceptions
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

@mcp.tool()
async def add_task(
    user_id: str,
    title: str,
    description: Optional[str] = None
) -> dict:
    """Create a new task for the user.

    Args:
        user_id: The authenticated user's UUID
        title: Task title (1-255 characters)
        description: Optional task description (max 10,000 characters)

    Returns:
        dict: Task creation result with task_id, status, and task details
    """
    # Validation
    if not title or not title.strip():
        raise ValidationError("Title cannot be empty or whitespace")
    if len(title) > 255:
        raise ValidationError("Title exceeds 255 characters")
    if description and len(description) > 10000:
        raise ValidationError("Description exceeds 10,000 characters")

    async with AsyncSession(async_engine) as session:
        # Verify user exists
        user = await session.get(User, user_id)
        if not user:
            raise ValueError(f"User {user_id} not found")

        # Create task
        task = Task(
            user_id=user_id,
            title=title.strip(),
            description=description,
            completed=False
        )
        session.add(task)
        await session.commit()
        await session.refresh(task)

        return {
            "task_id": task.id,
            "status": "created",
            "title": task.title,
            "description": task.description,
            "completed": task.completed,
            "created_at": task.created_at.isoformat(),
            "updated_at": task.updated_at.isoformat()
        }

# Implement remaining tools: list_tasks, complete_task, update_task, delete_task
# (See data-model.md and contracts/ for full specifications)
```

## Step 7: Create Run Script

Create `backend/mcp_server/run_mcp.py`:

```python
#!/usr/bin/env python3
import sys
import os

# Add parent directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.server import mcp

if __name__ == "__main__":
    # Run the MCP server
    mcp.run()
```

Make it executable:

```bash
chmod +x backend/mcp_server/run_mcp.py
```

## Step 8: Test MCP Server with Inspector

Run the MCP server and test with MCP Inspector:

```bash
# Terminal 1: Run MCP server
cd backend
python mcp_server/run_mcp.py

# Terminal 2: Run MCP Inspector
npx -y @modelcontextprotocol/inspector
```

**In MCP Inspector**:
1. Connect to the server (stdio transport)
2. View available tools (should see 5 tools)
3. Test each tool with sample inputs

## Step 9: Write Unit Tests

Create `backend/mcp_server/tests/conftest.py`:

```python
import pytest
from unittest.mock import AsyncMock, MagicMock
from sqlmodel import Session
from database.session import engine

@pytest.fixture
def mock_session():
    """Create a mock async session for unit tests."""
    session = MagicMock()
    session.get = AsyncMock()
    session.add = MagicMock()
    session.commit = AsyncMock()
    session.refresh = AsyncMock()
    session.execute = AsyncMock()
    session.delete = MagicMock()
    return session

@pytest.fixture(scope="function")
def db_session():
    """Create a real database session for integration tests."""
    connection = engine.connect()
    transaction = connection.begin()
    session = Session(bind=connection)

    yield session

    session.close()
    transaction.rollback()
    connection.close()
```

Create `backend/mcp_server/tests/test_task_tools.py`:

```python
import pytest
from mcp_server.tools.task_tools import add_task, list_tasks, complete_task
from models.task_models import User, Task

@pytest.mark.asyncio
async def test_add_task_success(mock_session):
    """Test successful task creation."""
    # Arrange
    user_id = "test-user-123"
    title = "Test Task"

    mock_user = User(id=user_id, email="test@example.com", name="Test User")
    mock_session.get = AsyncMock(return_value=mock_user)

    # Act
    result = await add_task(user_id, title)

    # Assert
    assert result["status"] == "created"
    assert result["title"] == title
    mock_session.add.assert_called_once()
    mock_session.commit.assert_called_once()

@pytest.mark.asyncio
async def test_add_task_user_not_found(mock_session):
    """Test task creation with invalid user."""
    mock_session.get = AsyncMock(return_value=None)

    with pytest.raises(ValueError, match="User .* not found"):
        await add_task("invalid-user", "Test Task")

# Add more tests for all tools and error cases
```

## Step 10: Run Tests

Run the test suite:

```bash
# From backend directory
pytest mcp_server/tests/ -v --cov=mcp_server/tools
```

**Expected Output**:
```
test_task_tools.py::test_add_task_success PASSED
test_task_tools.py::test_add_task_user_not_found PASSED
...
Coverage: 100%
```

## Step 11: Integration Testing

Create a test script to verify end-to-end functionality:

```python
# test_mcp_integration.py
import asyncio
from mcp_server.tools.task_tools import add_task, list_tasks, complete_task, delete_task

async def test_full_lifecycle():
    """Test complete task lifecycle."""
    user_id = "test-user-uuid"  # Use a real test user ID

    # Create task
    print("Creating task...")
    create_result = await add_task(user_id, "Integration Test Task", "Test description")
    task_id = create_result["task_id"]
    print(f"✓ Created task #{task_id}")

    # List tasks
    print("Listing tasks...")
    list_result = await list_tasks(user_id, "all")
    print(f"✓ Found {list_result['count']} tasks")

    # Complete task
    print("Completing task...")
    complete_result = await complete_task(user_id, task_id)
    print(f"✓ Completed task #{task_id}")

    # Delete task
    print("Deleting task...")
    delete_result = await delete_task(user_id, task_id)
    print(f"✓ Deleted task #{task_id}")

    print("\n✅ All integration tests passed!")

if __name__ == "__main__":
    asyncio.run(test_full_lifecycle())
```

Run integration test:

```bash
python test_mcp_integration.py
```

## Troubleshooting

### Issue: Import errors

**Solution**: Ensure Python path includes backend directory:
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
```

### Issue: Database connection errors

**Solution**: Verify DATABASE_URL in `.env` and database is running

### Issue: Async driver not found

**Solution**: Install asyncpg:
```bash
pip install asyncpg
```

### Issue: MCP server won't start

**Solution**: Check for port conflicts, verify all imports are correct

### Issue: Tools not appearing in Inspector

**Solution**: Ensure tools are imported in server.py and decorated with @mcp.tool()

## Verification Checklist

- [ ] MCP SDK installed (`mcp[cli]`, `asyncpg`)
- [ ] Directory structure created
- [ ] Async database session added to session.py
- [ ] MCP server.py created
- [ ] All 5 tools implemented in task_tools.py
- [ ] Run script created and executable
- [ ] MCP Inspector can connect and see tools
- [ ] Unit tests written and passing
- [ ] Integration tests passing
- [ ] All tools tested manually via Inspector

## Next Steps

After verifying the MCP server works:

1. **Integrate with AI Agent**: Connect the MCP server to an AI agent service
2. **Add Logging**: Implement comprehensive logging for debugging
3. **Add Monitoring**: Track tool usage metrics
4. **Production Deployment**: Set up process manager (systemd, supervisor)
5. **Documentation**: Create user guide for AI agent integration

## Success Criteria Verification

- ✅ All 5 tools implemented and functional
- ✅ User isolation enforced (users can only access their own tasks)
- ✅ Input validation working (empty titles rejected, length limits enforced)
- ✅ Error handling comprehensive (clear error messages)
- ✅ Tests passing (unit and integration)
- ✅ MCP Inspector can discover and call all tools
- ✅ Performance acceptable (<2s for list operations)

## Common Test Scenarios

### Scenario 1: Create and List Tasks
```python
# Create 3 tasks
await add_task(user_id, "Task 1")
await add_task(user_id, "Task 2")
await add_task(user_id, "Task 3")

# List all tasks
result = await list_tasks(user_id, "all")
assert result["count"] == 3
```

### Scenario 2: Filter by Status
```python
# Create and complete some tasks
task1 = await add_task(user_id, "Task 1")
task2 = await add_task(user_id, "Task 2")
await complete_task(user_id, task1["task_id"])

# List pending tasks
pending = await list_tasks(user_id, "pending")
assert pending["count"] == 1

# List completed tasks
completed = await list_tasks(user_id, "completed")
assert completed["count"] == 1
```

### Scenario 3: Update Task
```python
# Create task
task = await add_task(user_id, "Original Title")

# Update title
updated = await update_task(user_id, task["task_id"], title="New Title")
assert updated["title"] == "New Title"
```

### Scenario 4: User Isolation
```python
# User A creates task
task = await add_task("user-a", "Task A")

# User B tries to access it (should fail)
with pytest.raises(UnauthorizedTaskAccessError):
    await complete_task("user-b", task["task_id"])
```

## Performance Benchmarks

Expected performance targets:

- **add_task**: < 100ms
- **list_tasks**: < 500ms for 100 tasks, < 2s for 1000 tasks
- **complete_task**: < 100ms
- **update_task**: < 100ms
- **delete_task**: < 100ms

Run performance tests:

```python
import time
import asyncio

async def benchmark_list_tasks(user_id, num_tasks):
    start = time.time()
    result = await list_tasks(user_id, "all")
    elapsed = time.time() - start
    print(f"Listed {result['count']} tasks in {elapsed:.3f}s")
    assert elapsed < 2.0, "Performance target not met"

# Run benchmark
asyncio.run(benchmark_list_tasks("test-user", 1000))
```

## Security Testing

Verify security measures:

```python
# Test 1: Cross-user access prevention
user_a_task = await add_task("user-a", "Task A")
with pytest.raises(UnauthorizedTaskAccessError):
    await delete_task("user-b", user_a_task["task_id"])

# Test 2: SQL injection prevention
malicious_title = "'; DROP TABLE task; --"
result = await add_task(user_id, malicious_title)
# Should create task with literal title, not execute SQL

# Test 3: Input validation
with pytest.raises(ValidationError):
    await add_task(user_id, "")  # Empty title

with pytest.raises(ValidationError):
    await add_task(user_id, "A" * 256)  # Title too long
```
