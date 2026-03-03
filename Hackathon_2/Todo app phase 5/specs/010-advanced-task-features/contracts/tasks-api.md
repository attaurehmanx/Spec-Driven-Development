# API Contract: Task Management Endpoints

**Feature**: 010-advanced-task-features
**Date**: 2026-02-15
**Purpose**: REST API contract specifications for advanced task features

## Base URL

- **Local Development**: `http://localhost:8000`
- **Production**: `https://api.taskapp.example.com`

## Authentication

All endpoints require JWT authentication via Bearer token:

```
Authorization: Bearer <jwt_token>
```

The `user_id` in the URL path must match the authenticated user's ID from the JWT token. Requests with mismatched user IDs return `403 Forbidden`.

---

## Endpoints

### 1. List Tasks (Extended)

**Endpoint**: `GET /api/{user_id}/tasks`

**Description**: Retrieve all tasks for a user with optional filtering, searching, and sorting.

**Path Parameters**:
- `user_id` (UUID, required): User identifier

**Query Parameters**:

| Parameter | Type | Required | Description | Example |
|-----------|------|----------|-------------|---------|
| priority | string | No | Filter by priority level | `high`, `medium`, `low` |
| tags | string[] | No | Filter by tags (comma-separated) | `work,urgent` |
| status | string | No | Filter by completion status | `done`, `not_done` |
| search | string | No | Search in title and description | `meeting` |
| sort | string | No | Sort order | `priority`, `due_date`, `created_at`, `title` |
| limit | integer | No | Maximum results (default: 1000) | `100` |
| offset | integer | No | Pagination offset (default: 0) | `0` |

**Request Example**:
```http
GET /api/550e8400-e29b-41d4-a716-446655440000/tasks?priority=high&tags=work&sort=due_date
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response**: `200 OK`

```json
{
  "tasks": [
    {
      "id": "123e4567-e89b-12d3-a456-426614174000",
      "user_id": "550e8400-e29b-41d4-a716-446655440000",
      "title": "Complete project proposal",
      "description": "Finalize Q1 project proposal for client review",
      "completed": false,
      "priority": "high",
      "tags": ["work", "urgent"],
      "due_date": "2026-02-20T17:00:00Z",
      "recurring": "none",
      "parent_task_id": null,
      "created_at": "2026-02-15T10:00:00Z",
      "updated_at": "2026-02-15T10:00:00Z",
      "is_overdue": false
    }
  ],
  "total": 1,
  "limit": 1000,
  "offset": 0
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `400 Bad Request`: Invalid query parameters

---

### 2. Create Task (Extended)

**Endpoint**: `POST /api/{user_id}/tasks`

**Description**: Create a new task with advanced features.

**Path Parameters**:
- `user_id` (UUID, required): User identifier

**Request Body**:

```json
{
  "title": "Complete project proposal",
  "description": "Finalize Q1 project proposal for client review",
  "priority": "high",
  "tags": ["work", "urgent"],
  "due_date": "2026-02-20T17:00:00Z",
  "recurring": "none"
}
```

**Field Specifications**:

| Field | Type | Required | Constraints | Default |
|-------|------|----------|-------------|---------|
| title | string | Yes | 1-255 characters | - |
| description | string | No | Max 10,000 characters | null |
| priority | enum | No | `high`, `medium`, `low` | `medium` |
| tags | string[] | No | Max 20 tags, each 1-50 chars, alphanumeric + hyphens | `[]` |
| due_date | datetime | No | ISO 8601 format, not >1 year past or >10 years future | null |
| recurring | enum | No | `none`, `daily`, `weekly`, `monthly` | `none` |

**Response**: `201 Created`

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Complete project proposal",
  "description": "Finalize Q1 project proposal for client review",
  "completed": false,
  "priority": "high",
  "tags": ["work", "urgent"],
  "due_date": "2026-02-20T17:00:00Z",
  "recurring": "none",
  "parent_task_id": null,
  "created_at": "2026-02-15T10:00:00Z",
  "updated_at": "2026-02-15T10:00:00Z",
  "is_overdue": false
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `400 Bad Request`: Invalid request body
- `422 Unprocessable Entity`: Validation errors

**Validation Error Example**:
```json
{
  "detail": [
    {
      "loc": ["body", "tags", 0],
      "msg": "Invalid tag format: urgent!",
      "type": "value_error"
    }
  ]
}
```

---

### 3. Get Task by ID

**Endpoint**: `GET /api/{user_id}/tasks/{task_id}`

**Description**: Retrieve a specific task by ID.

**Path Parameters**:
- `user_id` (UUID, required): User identifier
- `task_id` (UUID, required): Task identifier

**Response**: `200 OK`

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Complete project proposal",
  "description": "Finalize Q1 project proposal for client review",
  "completed": false,
  "priority": "high",
  "tags": ["work", "urgent"],
  "due_date": "2026-02-20T17:00:00Z",
  "recurring": "none",
  "parent_task_id": null,
  "created_at": "2026-02-15T10:00:00Z",
  "updated_at": "2026-02-15T10:00:00Z",
  "is_overdue": false
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `404 Not Found`: Task not found

---

### 4. Update Task (Extended)

**Endpoint**: `PUT /api/{user_id}/tasks/{task_id}`

**Description**: Update an existing task with advanced features.

**Path Parameters**:
- `user_id` (UUID, required): User identifier
- `task_id` (UUID, required): Task identifier

**Request Body** (all fields optional):

```json
{
  "title": "Complete project proposal - UPDATED",
  "description": "Finalize Q1 project proposal for client review - added details",
  "completed": false,
  "priority": "medium",
  "tags": ["work"],
  "due_date": "2026-02-21T17:00:00Z",
  "recurring": "weekly"
}
```

**Response**: `200 OK`

```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Complete project proposal - UPDATED",
  "description": "Finalize Q1 project proposal for client review - added details",
  "completed": false,
  "priority": "medium",
  "tags": ["work"],
  "due_date": "2026-02-21T17:00:00Z",
  "recurring": "weekly",
  "parent_task_id": null,
  "created_at": "2026-02-15T10:00:00Z",
  "updated_at": "2026-02-15T11:30:00Z",
  "is_overdue": false
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `404 Not Found`: Task not found
- `400 Bad Request`: Invalid request body
- `422 Unprocessable Entity`: Validation errors

---

### 5. Complete Task (Extended)

**Endpoint**: `PATCH /api/{user_id}/tasks/{task_id}/complete`

**Description**: Toggle task completion status. If task is recurring and being marked complete, automatically creates next instance.

**Path Parameters**:
- `user_id` (UUID, required): User identifier
- `task_id` (UUID, required): Task identifier

**Request Body**:

```json
{
  "completed": true
}
```

**Response**: `200 OK`

```json
{
  "task": {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "title": "Weekly team meeting",
    "description": "Discuss project progress",
    "completed": true,
    "priority": "medium",
    "tags": ["work", "meeting"],
    "due_date": "2026-02-15T14:00:00Z",
    "recurring": "weekly",
    "parent_task_id": null,
    "created_at": "2026-02-08T10:00:00Z",
    "updated_at": "2026-02-15T14:05:00Z",
    "is_overdue": false
  },
  "next_instance": {
    "id": "789e4567-e89b-12d3-a456-426614174999",
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "title": "Weekly team meeting",
    "description": "Discuss project progress",
    "completed": false,
    "priority": "medium",
    "tags": ["work", "meeting"],
    "due_date": "2026-02-22T14:00:00Z",
    "recurring": "weekly",
    "parent_task_id": "123e4567-e89b-12d3-a456-426614174000",
    "created_at": "2026-02-15T14:05:00Z",
    "updated_at": "2026-02-15T14:05:00Z",
    "is_overdue": false
  }
}
```

**Note**: If task is not recurring, `next_instance` will be `null`.

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `404 Not Found`: Task not found

---

### 6. Delete Task

**Endpoint**: `DELETE /api/{user_id}/tasks/{task_id}`

**Description**: Delete a task. For recurring tasks, only deletes the current instance.

**Path Parameters**:
- `user_id` (UUID, required): User identifier
- `task_id` (UUID, required): Task identifier

**Response**: `204 No Content`

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch
- `404 Not Found`: Task not found

---

### 7. Get User Tags (New)

**Endpoint**: `GET /api/{user_id}/tasks/tags`

**Description**: Retrieve all unique tags used by the user across all tasks.

**Path Parameters**:
- `user_id` (UUID, required): User identifier

**Response**: `200 OK`

```json
{
  "tags": [
    "work",
    "urgent",
    "home",
    "personal",
    "shopping",
    "meeting"
  ]
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch

---

### 8. Get Overdue Tasks (New)

**Endpoint**: `GET /api/{user_id}/tasks/overdue`

**Description**: Retrieve all overdue tasks (due_date < NOW() AND completed = false).

**Path Parameters**:
- `user_id` (UUID, required): User identifier

**Response**: `200 OK`

```json
{
  "tasks": [
    {
      "id": "123e4567-e89b-12d3-a456-426614174000",
      "user_id": "550e8400-e29b-41d4-a716-446655440000",
      "title": "Submit tax documents",
      "description": "File Q4 tax documents",
      "completed": false,
      "priority": "high",
      "tags": ["personal", "urgent"],
      "due_date": "2026-02-10T23:59:59Z",
      "recurring": "none",
      "parent_task_id": null,
      "created_at": "2026-01-15T10:00:00Z",
      "updated_at": "2026-01-15T10:00:00Z",
      "is_overdue": true
    }
  ],
  "total": 1
}
```

**Error Responses**:
- `401 Unauthorized`: Missing or invalid JWT token
- `403 Forbidden`: User ID mismatch

---

## Common Response Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created successfully |
| 204 | No Content | Resource deleted successfully |
| 400 | Bad Request | Invalid request format or parameters |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | User not authorized for this resource |
| 404 | Not Found | Resource not found |
| 422 | Unprocessable Entity | Validation errors in request body |
| 500 | Internal Server Error | Server error |

---

## Rate Limiting

- **Limit**: 1000 requests per hour per user
- **Headers**:
  - `X-RateLimit-Limit`: Maximum requests per hour
  - `X-RateLimit-Remaining`: Remaining requests in current window
  - `X-RateLimit-Reset`: Unix timestamp when limit resets

**Rate Limit Exceeded Response**: `429 Too Many Requests`

```json
{
  "detail": "Rate limit exceeded. Try again in 1800 seconds."
}
```

---

## Pagination

For endpoints returning lists (e.g., `GET /api/{user_id}/tasks`):

- Default limit: 1000 tasks
- Maximum limit: 1000 tasks
- Use `offset` parameter for pagination

**Example**:
```http
GET /api/{user_id}/tasks?limit=100&offset=100
```

Returns tasks 101-200.

---

## Filtering Logic

### Priority Filter
- Single value: `?priority=high`
- Returns only tasks with specified priority

### Tags Filter
- Comma-separated: `?tags=work,urgent`
- Returns tasks with ANY of the specified tags (OR logic)

### Status Filter
- `?status=done`: Returns completed tasks
- `?status=not_done`: Returns incomplete tasks

### Search
- `?search=meeting`: Case-insensitive partial match on title OR description
- Uses SQL `ILIKE` operator

### Combined Filters
All filters can be combined with AND logic:
```http
GET /api/{user_id}/tasks?priority=high&tags=work&status=not_done&search=proposal
```

Returns incomplete, high-priority tasks tagged with "work" that contain "proposal" in title or description.

---

## Sorting Logic

**Sort Parameter Values**:
- `priority`: High → Medium → Low
- `due_date`: Overdue first, then nearest due date first, then no due date
- `created_at`: Newest first
- `title`: Alphabetical A-Z

**Default**: No explicit sort (database order, typically by `created_at DESC`)

**Example**:
```http
GET /api/{user_id}/tasks?sort=due_date
```

---

## Validation Rules Summary

### Title
- Required
- 1-255 characters
- Cannot be empty string

### Description
- Optional
- Max 10,000 characters

### Priority
- Must be one of: `high`, `medium`, `low`
- Defaults to `medium`

### Tags
- Array of strings
- Max 20 tags per task
- Each tag: 1-50 characters, alphanumeric + hyphens only
- Pattern: `^[a-zA-Z0-9-]{1,50}$`

### Due Date
- Optional
- ISO 8601 format with timezone
- Cannot be >1 year in the past
- Cannot be >10 years in the future

### Recurring
- Must be one of: `none`, `daily`, `weekly`, `monthly`
- Defaults to `none`

---

## Backward Compatibility

All new fields (priority, tags, due_date, recurring) are optional in request bodies. Existing API clients that don't send these fields will continue to work with default values:

- `priority`: `medium`
- `tags`: `[]`
- `due_date`: `null`
- `recurring`: `none`

Response bodies include all fields. Clients should ignore unknown fields for forward compatibility.
