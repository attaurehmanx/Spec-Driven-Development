# Data Model: MCP Task Tools

**Feature**: 004-mcp-task-tools
**Date**: 2026-01-29
**Based on**: Existing Task and User models from `backend/models/task_models.py`

## Overview

This data model defines the input and output schemas for MCP task management tools. It leverages existing database entities (User and Task) and defines Pydantic models for tool parameters and responses.

## Existing Entities (Reused)

### User Entity

**Source**: `backend/models/task_models.py`

**Fields**:
- **id**: Unique identifier (UUID string, primary key)
- **email**: User email address (unique, indexed)
- **name**: User's full name
- **created_at**: Account creation timestamp
- **is_active**: Account status flag

**Usage in MCP Tools**: Used for user validation - every tool verifies the user exists before performing operations.

---

### Task Entity

**Source**: `backend/models/task_models.py`

**Fields**:
- **id**: Unique identifier (integer, auto-increment, primary key)
- **user_id**: Foreign key to User (UUID string, indexed)
- **title**: Task title (string, required, max 255 characters)
- **description**: Task description (string, optional)
- **completed**: Completion status (boolean, default false)
- **created_at**: Creation timestamp (datetime, auto-generated)
- **updated_at**: Last update timestamp (datetime, auto-updated)

**Relationships**:
- Many-to-one with User: `owner: User` (back_populates="tasks")

**Usage in MCP Tools**: Primary entity for all CRUD operations.

---

## Tool Input Schemas

### AddTaskInput

**Purpose**: Parameters for creating a new task

**Fields**:
- **user_id**: str (required) - Authenticated user's UUID
- **title**: str (required) - Task title, 1-255 characters
- **description**: str | None (optional) - Task description, max 10,000 characters

**Validation Rules**:
- `user_id` must be a valid UUID string
- `title` cannot be empty or whitespace-only
- `title` length: 1-255 characters
- `description` length: 0-10,000 characters if provided
- User with `user_id` must exist in database

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator

class AddTaskInput(BaseModel):
    user_id: str = Field(..., description="Authenticated user's UUID")
    title: str = Field(..., min_length=1, max_length=255, description="Task title")
    description: str | None = Field(None, max_length=10000, description="Optional task description")

    @validator('title')
    def title_not_empty(cls, v):
        if not v or not v.strip():
            raise ValueError("Title cannot be empty or whitespace")
        return v.strip()
```

---

### ListTasksInput

**Purpose**: Parameters for retrieving user's task list

**Fields**:
- **user_id**: str (required) - Authenticated user's UUID
- **status**: str (optional, default="all") - Filter by completion status

**Validation Rules**:
- `user_id` must be a valid UUID string
- `status` must be one of: "all", "pending", "completed"
- User with `user_id` must exist in database

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator
from typing import Literal

class ListTasksInput(BaseModel):
    user_id: str = Field(..., description="Authenticated user's UUID")
    status: Literal["all", "pending", "completed"] = Field(
        default="all",
        description="Filter tasks by completion status"
    )
```

---

### CompleteTaskInput

**Purpose**: Parameters for marking a task as completed

**Fields**:
- **user_id**: str (required) - Authenticated user's UUID
- **task_id**: int (required) - Task identifier

**Validation Rules**:
- `user_id` must be a valid UUID string
- `task_id` must be a positive integer
- User with `user_id` must exist in database
- Task with `task_id` must exist
- Task must belong to the user (`task.user_id == user_id`)

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator

class CompleteTaskInput(BaseModel):
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to mark as completed")
```

---

### UpdateTaskInput

**Purpose**: Parameters for updating task details

**Fields**:
- **user_id**: str (required) - Authenticated user's UUID
- **task_id**: int (required) - Task identifier
- **title**: str | None (optional) - New task title
- **description**: str | None (optional) - New task description

**Validation Rules**:
- `user_id` must be a valid UUID string
- `task_id` must be a positive integer
- At least one of `title` or `description` must be provided
- If `title` provided: 1-255 characters, not empty/whitespace
- If `description` provided: max 10,000 characters
- User with `user_id` must exist in database
- Task with `task_id` must exist
- Task must belong to the user (`task.user_id == user_id`)

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, validator, root_validator

class UpdateTaskInput(BaseModel):
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to update")
    title: str | None = Field(None, min_length=1, max_length=255, description="New task title")
    description: str | None = Field(None, max_length=10000, description="New task description")

    @validator('title')
    def title_not_empty(cls, v):
        if v is not None and (not v or not v.strip()):
            raise ValueError("Title cannot be empty or whitespace")
        return v.strip() if v else None

    @root_validator
    def at_least_one_field(cls, values):
        if values.get('title') is None and values.get('description') is None:
            raise ValueError("At least one of title or description must be provided")
        return values
```

---

### DeleteTaskInput

**Purpose**: Parameters for deleting a task

**Fields**:
- **user_id**: str (required) - Authenticated user's UUID
- **task_id**: int (required) - Task identifier

**Validation Rules**:
- `user_id` must be a valid UUID string
- `task_id` must be a positive integer
- User with `user_id` must exist in database
- Task with `task_id` must exist
- Task must belong to the user (`task.user_id == user_id`)

**Pydantic Model**:
```python
from pydantic import BaseModel, Field

class DeleteTaskInput(BaseModel):
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to delete")
```

---

## Tool Output Schemas

### TaskOperationResponse

**Purpose**: Standard response for single-task operations (add, complete, update, delete)

**Fields**:
- **task_id**: int - Task identifier
- **status**: str - Operation status ("created", "completed", "updated", "deleted")
- **title**: str - Task title
- **description**: str | None - Task description (optional)
- **completed**: bool - Completion status
- **created_at**: str - Creation timestamp (ISO 8601 format)
- **updated_at**: str - Last update timestamp (ISO 8601 format)

**JSON Schema**:
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

**Pydantic Model**:
```python
from pydantic import BaseModel
from datetime import datetime

class TaskOperationResponse(BaseModel):
    task_id: int
    status: str  # "created" | "completed" | "updated" | "deleted"
    title: str
    description: str | None
    completed: bool
    created_at: str  # ISO 8601 format
    updated_at: str  # ISO 8601 format
```

---

### TaskListResponse

**Purpose**: Response for list_tasks operation

**Fields**:
- **tasks**: list[TaskSummary] - Array of task objects
- **count**: int - Number of tasks returned
- **filter**: str - Applied filter ("all", "pending", "completed")

**TaskSummary Fields**:
- **task_id**: int - Task identifier
- **title**: str - Task title
- **description**: str | None - Task description (optional)
- **completed**: bool - Completion status
- **created_at**: str - Creation timestamp (ISO 8601)
- **updated_at**: str - Last update timestamp (ISO 8601)

**JSON Schema**:
```json
{
  "tasks": [
    {
      "task_id": 1,
      "title": "Buy groceries",
      "description": "Milk, eggs, bread",
      "completed": false,
      "created_at": "2026-01-29T10:30:00Z",
      "updated_at": "2026-01-29T10:30:00Z"
    },
    {
      "task_id": 2,
      "title": "Call dentist",
      "description": null,
      "completed": true,
      "created_at": "2026-01-28T14:20:00Z",
      "updated_at": "2026-01-29T09:15:00Z"
    }
  ],
  "count": 2,
  "filter": "all"
}
```

**Pydantic Model**:
```python
from pydantic import BaseModel
from typing import List

class TaskSummary(BaseModel):
    task_id: int
    title: str
    description: str | None
    completed: bool
    created_at: str  # ISO 8601 format
    updated_at: str  # ISO 8601 format

class TaskListResponse(BaseModel):
    tasks: List[TaskSummary]
    count: int
    filter: str  # "all" | "pending" | "completed"
```

---

## Error Response Schema

**Purpose**: Standard error response for all tools

**Format**: MCP SDK automatically converts Python exceptions to error responses

**Error Types**:
- **ValueError**: Validation errors, not found errors, authorization errors
- **TaskToolError**: Base exception for custom tool errors
- **TaskNotFoundError**: Task does not exist
- **UnauthorizedTaskAccessError**: User does not own the task
- **ValidationError**: Input validation failed

**Error Message Format**:
```json
{
  "error": {
    "code": "ValueError",
    "message": "Task 123 not found"
  }
}
```

**Exception Hierarchy**:
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

---

## Validation Rules Summary

### User Validation

**Rule**: User must exist before any operation
**Check**: `user = await session.get(User, user_id)`
**Error**: `ValueError("User {user_id} not found")`

---

### Task Ownership Validation

**Rule**: User must own the task for modify/delete operations
**Check**: `task.user_id == user_id`
**Error**: `UnauthorizedTaskAccessError("Task {task_id} does not belong to user {user_id}")`

---

### Title Validation

**Rules**:
- Required for add_task
- Optional for update_task
- Cannot be empty or whitespace-only
- Length: 1-255 characters

**Error**: `ValidationError("Title cannot be empty")` or `ValidationError("Title exceeds 255 characters")`

---

### Description Validation

**Rules**:
- Optional for all operations
- Max length: 10,000 characters

**Error**: `ValidationError("Description exceeds 10,000 characters")`

---

### Status Filter Validation

**Rules**:
- Must be one of: "all", "pending", "completed"
- Case-sensitive

**Error**: `ValidationError("Invalid status filter: {status}")`

---

### Task ID Validation

**Rules**:
- Must be a positive integer
- Task must exist in database

**Error**: `TaskNotFoundError("Task {task_id} not found")`

---

## Data Flow Patterns

### Create Task Flow

1. **Input**: `AddTaskInput(user_id, title, description)`
2. **Validation**: Validate user exists
3. **Create**: `Task(user_id=user_id, title=title, description=description, completed=False)`
4. **Persist**: `session.add(task)`, `session.commit()`, `session.refresh(task)`
5. **Output**: `TaskOperationResponse(task_id, status="created", ...)`

---

### List Tasks Flow

1. **Input**: `ListTasksInput(user_id, status)`
2. **Validation**: Validate user exists, validate status filter
3. **Query**: `select(Task).where(Task.user_id == user_id)`
4. **Filter**: Apply status filter if not "all"
5. **Output**: `TaskListResponse(tasks=[...], count=len(tasks), filter=status)`

---

### Complete Task Flow

1. **Input**: `CompleteTaskInput(user_id, task_id)`
2. **Validation**: Validate user exists, validate task exists, validate ownership
3. **Update**: `task.completed = True`, `task.updated_at = datetime.utcnow()`
4. **Persist**: `session.commit()`, `session.refresh(task)`
5. **Output**: `TaskOperationResponse(task_id, status="completed", ...)`

---

### Update Task Flow

1. **Input**: `UpdateTaskInput(user_id, task_id, title?, description?)`
2. **Validation**: Validate user exists, validate task exists, validate ownership, validate at least one field
3. **Update**: Set provided fields, update `updated_at`
4. **Persist**: `session.commit()`, `session.refresh(task)`
5. **Output**: `TaskOperationResponse(task_id, status="updated", ...)`

---

### Delete Task Flow

1. **Input**: `DeleteTaskInput(user_id, task_id)`
2. **Validation**: Validate user exists, validate task exists, validate ownership
3. **Delete**: `session.delete(task)`, `session.commit()`
4. **Output**: `TaskOperationResponse(task_id, status="deleted", ...)`

---

## Database Queries

### User Existence Check

```python
user = await session.get(User, user_id)
if not user:
    raise ValueError(f"User {user_id} not found")
```

---

### Task Retrieval with Ownership Check

```python
task = await session.get(Task, task_id)
if not task:
    raise TaskNotFoundError(f"Task {task_id} not found")
if task.user_id != user_id:
    raise UnauthorizedTaskAccessError(f"Task {task_id} does not belong to user {user_id}")
```

---

### List All Tasks

```python
from sqlmodel import select

stmt = select(Task).where(Task.user_id == user_id)
result = await session.execute(stmt)
tasks = result.scalars().all()
```

---

### List Pending Tasks

```python
stmt = select(Task).where(Task.user_id == user_id, Task.completed == False)
result = await session.execute(stmt)
tasks = result.scalars().all()
```

---

### List Completed Tasks

```python
stmt = select(Task).where(Task.user_id == user_id, Task.completed == True)
result = await session.execute(stmt)
tasks = result.scalars().all()
```

---

## Performance Considerations

### Indexes

**Existing Indexes** (from Phase 2):
- `user.id` (primary key, automatic)
- `user.email` (unique index)
- `task.id` (primary key, automatic)
- `task.user_id` (foreign key, indexed)

**Query Performance**:
- User lookup by ID: O(1) via primary key
- Task lookup by ID: O(1) via primary key
- List tasks by user: O(n) with index on user_id
- Filter by completion status: O(n) with sequential scan (acceptable for user-scoped queries)

---

### Connection Pooling

**Strategy**: Use async connection pool shared with FastAPI
**Configuration**:
- Pool size: 10 connections
- Max overflow: 20 connections
- Pool timeout: 30 seconds
- Pre-ping: Enabled (verify connections before use)

---

## Security Considerations

### User Isolation

**Enforcement**:
- Every query filters by `user_id`
- Ownership verified before modifications
- No cross-user data access possible

**Pattern**:
```python
# Always filter by user_id
stmt = select(Task).where(Task.user_id == user_id)

# Always verify ownership
if task.user_id != user_id:
    raise UnauthorizedTaskAccessError(...)
```

---

### Input Sanitization

**Validation**:
- Pydantic models validate types and constraints
- Custom validators check business rules
- Database constraints enforce data integrity

**SQL Injection Prevention**:
- SQLModel uses parameterized queries
- No raw SQL execution
- ORM handles escaping automatically

---

## Assumptions

- User IDs are UUID strings (existing schema)
- Task IDs are auto-incrementing integers (existing schema)
- All timestamps are stored in UTC
- Task titles are plain text (no rich formatting)
- Task descriptions are plain text (no rich formatting)
- Completion status is binary (true/false, no partial completion)
- No soft deletes (tasks are permanently removed)
- No task history or audit trail
- No concurrent modification handling (last write wins)

---

## Constraints

- Title max length: 255 characters (database constraint)
- Description max length: 10,000 characters (application constraint)
- User ID format: UUID string (database constraint)
- Task ID format: Positive integer (database constraint)
- Status filter values: "all", "pending", "completed" (application constraint)
- User isolation: All operations scoped to authenticated user (security constraint)
