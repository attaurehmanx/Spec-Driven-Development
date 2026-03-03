# MCP Tool Contract: update_task

**Tool Name**: `update_task`
**Purpose**: Update a task's title and/or description for the authenticated user
**Priority**: P4

## Tool Signature

```python
@mcp.tool()
async def update_task(
    user_id: str,
    task_id: int,
    title: str = None,
    description: str = None
) -> dict:
    """Update a task's title and/or description.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to update
        title: New task title (optional, 1-255 characters)
        description: New task description (optional, max 10,000 characters)

    Returns:
        dict: Updated task details with status="updated"

    Raises:
        ValueError: If user not found or no fields provided
        TaskNotFoundError: If task not found
        UnauthorizedTaskAccessError: If user doesn't own the task
        ValidationError: If validation fails
    """
```

## Input Parameters

| Parameter | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| user_id | string | Yes | Valid UUID | Authenticated user's unique identifier |
| task_id | integer | Yes | Positive integer | Task identifier to update |
| title | string | No | 1-255 chars, non-empty | New task title |
| description | string | No | Max 10,000 chars | New task description |

## Input Validation

- **user_id**: Must be a valid UUID string, user must exist in database
- **task_id**: Must be a positive integer, task must exist in database
- **title**: If provided, cannot be empty or whitespace-only, length 1-255 characters
- **description**: If provided, max 10,000 characters
- **At least one field**: Either title or description must be provided
- **Ownership**: Task must belong to the user (task.user_id == user_id)

## Success Response

**HTTP Status**: 200 OK

**Response Schema**:
```json
{
  "task_id": 123,
  "status": "updated",
  "title": "Buy organic groceries",
  "description": "Milk, eggs, bread, vegetables",
  "completed": false,
  "created_at": "2026-01-29T10:30:00Z",
  "updated_at": "2026-01-29T16:20:00Z"
}
```

**Response Fields**:
- `task_id` (integer): Task identifier
- `status` (string): Always "updated" for this operation
- `title` (string): Updated task title
- `description` (string|null): Updated task description or null
- `completed` (boolean): Current completion status (unchanged)
- `created_at` (string): ISO 8601 creation timestamp (unchanged)
- `updated_at` (string): ISO 8601 timestamp of update

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

### Task Not Found
```json
{
  "error": {
    "code": "TaskNotFoundError",
    "message": "Task 123 not found"
  }
}
```

### Unauthorized Access
```json
{
  "error": {
    "code": "UnauthorizedTaskAccessError",
    "message": "Task 123 does not belong to user abc-123-def"
  }
}
```

### No Fields Provided
```json
{
  "error": {
    "code": "ValidationError",
    "message": "At least one of title or description must be provided"
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
3. Validate task_id parameter (positive integer)
4. Validate at least one of title or description is provided
5. Validate title if provided (non-empty, length constraints)
6. Validate description if provided (length constraints)
7. Query database to retrieve task
8. Verify task exists
9. Verify task belongs to user (task.user_id == user_id)
10. Update task fields:
    - If title provided: update title
    - If description provided: update description
    - Set updated_at = current UTC timestamp
11. Persist changes to database
12. Return updated task details with status="updated"

## Database Operations

```sql
-- Verify user exists
SELECT * FROM user WHERE id = :user_id;

-- Retrieve and verify task ownership
SELECT * FROM task WHERE id = :task_id;

-- Update task (title only)
UPDATE task
SET title = :title, updated_at = NOW()
WHERE id = :task_id AND user_id = :user_id
RETURNING *;

-- Update task (description only)
UPDATE task
SET description = :description, updated_at = NOW()
WHERE id = :task_id AND user_id = :user_id
RETURNING *;

-- Update task (both fields)
UPDATE task
SET title = :title, description = :description, updated_at = NOW()
WHERE id = :task_id AND user_id = :user_id
RETURNING *;
```

## Security Considerations

- User must exist in database
- Task must exist in database
- User must own the task (ownership verification)
- No cross-user task modification possible
- Input validation prevents SQL injection
- Partial updates supported (only provided fields are updated)

## Performance Considerations

- Uses primary key index for task lookup
- Single database update operation
- Async operation prevents blocking
- Ownership check prevents unauthorized access

## Test Cases

### Test 1: Update title only
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5,
  "title": "Buy organic groceries"
}
```
**Expected Output**: Task title updated, description unchanged

### Test 2: Update description only
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5,
  "description": "Updated description"
}
```
**Expected Output**: Task description updated, title unchanged

### Test 3: Update both fields
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5,
  "title": "New title",
  "description": "New description"
}
```
**Expected Output**: Both fields updated

### Test 4: No fields provided
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5
}
```
**Expected Output**: ValidationError

### Test 5: Update another user's task
**Input**:
```json
{
  "user_id": "user-a",
  "task_id": 10,
  "title": "Hacked"
}
```
**Expected Output**: UnauthorizedTaskAccessError

### Test 6: Empty title
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5,
  "title": "   "
}
```
**Expected Output**: ValidationError

## Usage Example

```python
# AI agent calls the tool
result = await update_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=5,
    title="Buy organic groceries"
)

# AI communicates result to user
print(f"Updated task #{result['task_id']}: {result['title']}")
```

## Related Tools

- `add_task`: Create new tasks
- `list_tasks`: View task list
- `complete_task`: Mark task as done
- `delete_task`: Remove tasks
