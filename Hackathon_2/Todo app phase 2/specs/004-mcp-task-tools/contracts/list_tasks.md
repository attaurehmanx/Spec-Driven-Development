# MCP Tool Contract: list_tasks

**Tool Name**: `list_tasks`
**Purpose**: Retrieve all tasks for the authenticated user with optional status filtering
**Priority**: P2

## Tool Signature

```python
@mcp.tool()
async def list_tasks(
    user_id: str,
    status: str = "all"
) -> dict:
    """List tasks for a user with optional status filter.

    Args:
        user_id: The authenticated user's UUID
        status: Filter by status - "all", "pending", or "completed"

    Returns:
        dict: List of tasks with count and filter information

    Raises:
        ValueError: If user not found or invalid status filter
    """
```

## Input Parameters

| Parameter | Type | Required | Constraints | Description |
|-----------|------|----------|-------------|-------------|
| user_id | string | Yes | Valid UUID | Authenticated user's unique identifier |
| status | string | No | "all", "pending", "completed" | Filter by completion status (default: "all") |

## Input Validation

- **user_id**: Must be a valid UUID string, user must exist in database
- **status**: Must be one of: "all", "pending", "completed" (case-sensitive)

## Success Response

**HTTP Status**: 200 OK

**Response Schema**:
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

**Response Fields**:
- `tasks` (array): List of task objects
  - `task_id` (integer): Unique task identifier
  - `title` (string): Task title
  - `description` (string|null): Task description or null
  - `completed` (boolean): Completion status
  - `created_at` (string): ISO 8601 creation timestamp
  - `updated_at` (string): ISO 8601 last update timestamp
- `count` (integer): Number of tasks returned
- `filter` (string): Applied filter ("all", "pending", "completed")

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

### Invalid Status Filter
```json
{
  "error": {
    "code": "ValidationError",
    "message": "Invalid status filter: invalid_status. Must be 'all', 'pending', or 'completed'"
  }
}
```

## Business Logic

1. Validate user_id parameter (UUID format)
2. Query database to verify user exists
3. Validate status parameter (must be "all", "pending", or "completed")
4. Query tasks filtered by user_id
5. Apply status filter:
   - "all": Return all tasks
   - "pending": Return tasks where completed = false
   - "completed": Return tasks where completed = true
6. Order tasks by created_at descending (newest first)
7. Return task list with count and filter information

## Database Operations

```sql
-- Verify user exists
SELECT * FROM user WHERE id = :user_id;

-- List all tasks
SELECT * FROM task
WHERE user_id = :user_id
ORDER BY created_at DESC;

-- List pending tasks
SELECT * FROM task
WHERE user_id = :user_id AND completed = false
ORDER BY created_at DESC;

-- List completed tasks
SELECT * FROM task
WHERE user_id = :user_id AND completed = true
ORDER BY created_at DESC;
```

## Security Considerations

- User must exist in database
- Only returns tasks belonging to the authenticated user
- No cross-user data access possible
- User isolation enforced at query level

## Performance Considerations

- Uses index on user_id for efficient filtering
- Single database query for task retrieval
- Async operation prevents blocking
- Supports large task lists (1000+ tasks)

## Test Cases

### Test 1: List all tasks
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "status": "all"
}
```
**Expected Output**: All tasks for user

### Test 2: List pending tasks
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "status": "pending"
}
```
**Expected Output**: Only incomplete tasks

### Test 3: List completed tasks
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "status": "completed"
}
```
**Expected Output**: Only completed tasks

### Test 4: Empty task list
**Input**:
```json
{
  "user_id": "user-with-no-tasks",
  "status": "all"
}
```
**Expected Output**: Empty tasks array, count=0

### Test 5: Invalid status filter
**Input**:
```json
{
  "user_id": "valid-uuid-123",
  "status": "invalid"
}
```
**Expected Output**: ValidationError

## Usage Example

```python
# AI agent calls the tool
result = await list_tasks(
    user_id="550e8400-e29b-41d4-a716-446655440000",
    status="pending"
)

# AI communicates result to user
print(f"You have {result['count']} pending tasks:")
for task in result['tasks']:
    print(f"- {task['title']}")
```

## Related Tools

- `add_task`: Create new tasks
- `complete_task`: Mark tasks as done
- `update_task`: Modify task details
- `delete_task`: Remove tasks
