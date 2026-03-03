# MCP Tool Contract: complete_task

**Tool Name**: `complete_task`
**Purpose**: Mark a task as completed for the authenticated user
**Priority**: P3

## Tool Signature

```python
@mcp.tool()
async def complete_task(
    user_id: str,
    task_id: int
) -> dict:
    """Mark a task as completed.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to mark as completed

    Returns:
        dict: Updated task details with status="completed"

    Raises:
        ValueError: If user not found
        TaskNotFoundError: If task not found
        UnauthorizedTaskAccessError: If user doesn't own the task
    """
```

## Input Parameters

| Parameter | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| user_id | string | Yes | Valid UUID | Authenticated user's unique identifier |
| task_id | integer | Yes | Positive integer | Task identifier to mark as completed |

## Input Validation

- **user_id**: Must be a valid UUID string, user must exist in database
- **task_id**: Must be a positive integer, task must exist in database
- **Ownership**: Task must belong to the user (task.user_id == user_id)

## Success Response

**HTTP Status**: 200 OK

**Response Schema**:
```json
{
  "task_id": 123,
  "status": "completed",
  "title": "Buy groceries",
  "description": "Milk, eggs, bread",
  "completed": true,
  "created_at": "2026-01-29T10:30:00Z",
  "updated_at": "2026-01-29T15:45:00Z"
}
```

**Response Fields**:
- `task_id` (integer): Task identifier
- `status` (string): Always "completed" for this operation
- `title` (string): Task title
- `description` (string|null): Task description or null
- `completed` (boolean): Always true after this operation
- `created_at` (string): ISO 8601 creation timestamp
- `updated_at` (string): ISO 8601 timestamp of completion

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

## Business Logic

1. Validate user_id parameter (UUID format)
2. Query database to verify user exists
3. Validate task_id parameter (positive integer)
4. Query database to retrieve task
5. Verify task exists
6. Verify task belongs to user (task.user_id == user_id)
7. Update task:
   - Set completed = true
   - Set updated_at = current UTC timestamp
8. Persist changes to database
9. Return updated task details with status="completed"

## Database Operations

```sql
-- Verify user exists
SELECT * FROM user WHERE id = :user_id;

-- Retrieve and verify task ownership
SELECT * FROM task WHERE id = :task_id;

-- Update task completion status
UPDATE task
SET completed = true, updated_at = NOW()
WHERE id = :task_id AND user_id = :user_id
RETURNING *;
```

## Security Considerations

- User must exist in database
- Task must exist in database
- User must own the task (ownership verification)
- No cross-user task modification possible
- Atomic update operation

## Performance Considerations

- Uses primary key index for task lookup
- Single database update operation
- Async operation prevents blocking
- Ownership check prevents unauthorized access

## Test Cases

### Test 1: Complete own task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5
}
```
**Expected Output**: Task marked as completed

### Test 2: Complete non-existent task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 99999
}
```
**Expected Output**: TaskNotFoundError

### Test 3: Complete another user's task
**Input**:
```json
{
  "user_id": "user-a",
  "task_id": 10
}
```
**Expected Output**: UnauthorizedTaskAccessError (if task belongs to user-b)

### Test 4: Complete already completed task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5
}
```
**Expected Output**: Task remains completed (idempotent operation)

## Usage Example

```python
# AI agent calls the tool
result = await complete_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=5
)

# AI communicates result to user
print(f"✓ Completed: {result['title']}")
```

## Related Tools

- `add_task`: Create new tasks
- `list_tasks`: View task list
- `update_task`: Modify task details
- `delete_task`: Remove tasks
