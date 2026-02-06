# API Contract: Task API & Persistence Layer

## Authentication Requirements

All endpoints require:
- Valid JWT token in Authorization header: `Authorization: Bearer {token}`
- User ID in URL path must match user ID in JWT claims
- 401 Unauthorized returned for invalid/missing tokens
- 403 Forbidden returned for cross-user access attempts

## Task Operations

### List Tasks
- **GET** `/api/{user_id}/tasks`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject
- **Response**: Array of Task objects
- **Success**: 200 OK
- **Errors**: 401, 403, 500

### Create Task
- **POST** `/api/{user_id}/tasks`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject
- **Request Body**: `{ "title": "string", "description": "string", "completed": false }`
- **Response**: Created Task object with ID and timestamps
- **Success**: 201 Created
- **Errors**: 400, 401, 403, 422, 500

### Get Task
- **GET** `/api/{user_id}/tasks/{id}`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject AND task ID must belong to user
- **Response**: Task object
- **Success**: 200 OK
- **Errors**: 401, 403, 404, 500

### Update Task
- **PUT** `/api/{user_id}/tasks/{id}`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject AND task ID must belong to user
- **Request Body**: `{ "title": "string", "description": "string", "completed": false }`
- **Response**: Updated Task object
- **Success**: 200 OK
- **Errors**: 400, 401, 403, 404, 422, 500

### Delete Task
- **DELETE** `/api/{user_id}/tasks/{id}`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject AND task ID must belong to user
- **Response**: Empty body
- **Success**: 204 No Content
- **Errors**: 401, 403, 404, 500

### Toggle Task Completion
- **PATCH** `/api/{user_id}/tasks/{id}/complete`
- **Authentication**: JWT required
- **Authorization**: user_id in URL must match JWT subject AND task ID must belong to user
- **Response**: Updated Task object with toggled completion status
- **Success**: 200 OK
- **Errors**: 401, 403, 404, 500

## Task Object Structure

```json
{
  "id": "string",
  "title": "string",
  "description": "string",
  "completed": true,
  "created_at": "2026-01-11T10:00:00Z",
  "updated_at": "2026-01-11T10:00:00Z",
  "user_id": "string"
}
```

## Error Response Structure

```json
{
  "detail": "Error message explaining the issue"
}
```

## Validation Rules

- Title: Required, 1-255 characters
- Description: Optional, maximum 1000 characters
- User ID: Must match authenticated user
- Task ID: Must exist and belong to authenticated user