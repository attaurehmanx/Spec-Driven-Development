# MCP Tool Contract: add_task

**Tool Name**: `add_task`
**Purpose**: Create a new task for the authenticated user
**Priority**: P1 (MVP)

## Tool Signature

```python
@mcp.tool()
async def add_task(
    user_id: str,
    title: str,
    description: str = None
) -> dict:
    """Create a new task for the user.

    Args:
        user_id: The authenticated user's UUID
        title: Task title (1-255 characters)
        description: Optional task description (max 10,000 characters)

    Returns:
        dict: Task creation result with task_id, status, and task details

    Raises:
        ValueError: If user not found or validation fails
    """
```

## Input Parameters

| Parameter | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| user_id | string | Yes | Valid UUID | Authenticated user's unique identifier |
| title | string | Yes | 1-255 chars, non-empty | Task title |
| description | string | No | Max 10,000 chars | Optional task description |

## Input Validation

- **user_id**: Must be a valid UUID string, user must exist in database
- **title**: Cannot be empty or whitespace-only, length 1-255 characters
- **description**: If provided, max 10,000 characters

## Success Response

**HTTP Status**: 200 OK

**Response Schema**:
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

**Response Fields**:
- `task_id` (integer): Unique identifier for the created task
- `status` (string): Always "created" for this operation
- `title` (string): Task title as provided
- `description` (string|null): Task description or null if not provided
- `completed` (boolean): Always false for new tasks
- `created_at` (string): ISO 8601 timestamp of creation
- `updated_at` (string): ISO 8601 timestamp of last update (same as created_at for new tasks)

## Error Responses

### User Not Found
```json
{
  "error": {
    "code": "ValueError",
    "message": "User abc-123-def not found"
  }
}
```

### Empty Title
```json
{
  "error": {
    "code": "ValidationError",
    "message": "Title cannot be empty or whitespace"
  }
}
```

### Title Too Long
```json
{
  "error": {
    "code": "ValidationError",
    "message": "Title exceeds 255 characters"
  }
}
```

### Description Too Long
```json
{
  "error": {
    "code": "ValidationError",
    "message": "Description exceeds 10,000 characters"
  }
}
```

## Business Logic

1. Validate user_id parameter (UUID format)
2. Query database to verify user exists
3. Validate title (non-empty, length constraints)
4. Validate description if provided (length constraints)
5. Create new Task entity with:
   - user_id from parameter
   - title from parameter
   - description from parameter (or null)
   - completed = false
   - created_at = current UTC timestamp
   - updated_at = current UTC timestamp
6. Persist task to database
7. Return task details with status="created"

## Database Operations

```sql
-- Verify user exists
SELECT * FROM user WHERE id = :user_id;

-- Insert new task
INSERT INTO task (user_id, title, description, completed, created_at, updated_at)
VALUES (:user_id, :title, :description, false, NOW(), NOW())
RETURNING *;
```

## Security Considerations

- User must exist in database before task creation
- Task is automatically associated with the authenticated user
- No cross-user task creation possible
- Input validation prevents SQL injection
- Title and description are sanitized

## Performance Considerations

- Single database query for user validation
- Single database insert for task creation
- Uses database connection pool
- Async operation prevents blocking

## Test Cases

### Test 1: Create task with title only
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "title": "Buy groceries"
}
```
**Expected Output**: Task created with null description

### Test 2: Create task with title and description
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "title": "Buy groceries",
  "description": "Milk, eggs, bread"
}
```
**Expected Output**: Task created with description

### Test 3: Invalid user
**Input**:
```json
{
  "user_id": "invalid-uuid",
  "title": "Test task"
}
```
**Expected Output**: ValueError "User invalid-uuid not found"

### Test 4: Empty title
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "title": "   "
}
```
**Expected Output**: ValidationError "Title cannot be empty"

### Test 5: Title too long
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "title": "A" * 256
}
```
**Expected Output**: ValidationError "Title exceeds 255 characters"

## Usage Example

```python
# AI agent calls the tool
result = await add_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    title="Buy groceries",
    description="Milk, eggs, bread"
)

# AI communicates result to user
print(f"Created task #{result['task_id']}: {result['title']}")
```

## Related Tools

- `list_tasks`: Retrieve created tasks
- `update_task`: Modify task details
- `complete_task`: Mark task as done
- `delete_task`: Remove task
