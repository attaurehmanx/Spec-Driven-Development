# MCP Tool Contract: delete_task

**Tool Name**: `delete_task`
**Purpose**: Permanently delete a task for the authenticated user
**Priority**: P5

## Tool Signature

```python
@mcp.tool()
async def delete_task(
    user_id: str,
    task_id: int
) -> dict:
    """Permanently delete a task.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to delete

    Returns:
        dict: Deleted task details with status="deleted"

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
| task_id | integer | Yes | Positive integer | Task identifier to delete |

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
  "status": "deleted",
  "title": "Buy groceries",
  "description": "Milk, eggs, bread",
  "completed": false,
  "created_at": "2026-01-29T10:30:00Z",
  "updated_at": "2026-01-29T15:45:00Z"
}
```

**Response Fields**:
- `task_id` (integer): Deleted task identifier
- `status` (string): Always "deleted" for this operation
- `title` (string): Task title (before deletion)
- `description` (string|null): Task description (before deletion) or null
- `completed` (boolean): Completion status (before deletion)
- `created_at` (string): ISO 8601 creation timestamp
- `updated_at` (string): ISO 8601 last update timestamp

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
7. Capture task details for response
8. Delete task from database (permanent deletion)
9. Return deleted task details with status="deleted"

## Database Operations

```sql
-- Verify user exists
SELECT * FROM user WHERE id = :user_id;

-- Retrieve and verify task ownership
SELECT * FROM task WHERE id = :task_id;

-- Delete task
DELETE FROM task
WHERE id = :task_id AND user_id = :user_id
RETURNING *;
```

## Security Considerations

- User must exist in database
- Task must exist in database
- User must own the task (ownership verification)
- No cross-user task deletion possible
- Permanent deletion (no soft delete)
- No undo mechanism

## Performance Considerations

- Uses primary key index for task lookup
- Single database delete operation
- Async operation prevents blocking
- Ownership check prevents unauthorized deletion

## Test Cases

### Test 1: Delete own task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5
}
```
**Expected Output**: Task permanently deleted

### Test 2: Delete non-existent task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 99999
}
```
**Expected Output**: TaskNotFoundError

### Test 3: Delete another user's task
**Input**:
```json
{
  "user_id": "user-a",
  "task_id": 10
}
```
**Expected Output**: UnauthorizedTaskAccessError (if task belongs to user-b)

### Test 4: Delete already deleted task
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "task_id": 5
}
```
**Expected Output**: TaskNotFoundError (task no longer exists)

### Test 5: Verify task is gone
**Input**: Call list_tasks after deletion
**Expected Output**: Deleted task not in list

## Usage Example

```python
# AI agent calls the tool
result = await delete_task(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    task_id=5
)

# AI communicates result to user
print(f"✓ Deleted: {result['title']}")
```

## Important Notes

- **Permanent Deletion**: This operation cannot be undone
- **No Soft Delete**: Task is completely removed from database
- **No History**: No audit trail of deleted tasks
- **Cascade Effects**: If future features depend on tasks, consider cascade behavior

## Related Tools

- `add_task`: Create new tasks
- `list_tasks`: View task list
- `complete_task`: Mark task as done
- `update_task`: Modify task details
