# API Contract: Frontend Application & User Experience

## Authentication API Endpoints (via Better Auth)

### Registration
- **POST** `/api/auth/register`
- **Request Body**: `{ "email": "string", "password": "string", "name": "string" }`
- **Response**: JWT token upon successful registration
- **Success**: 200 OK with user object and token
- **Errors**: 400 (validation), 409 (duplicate email)

### Login
- **POST** `/api/auth/login`
- **Request Body**: `{ "email": "string", "password": "string" }`
- **Response**: JWT token upon successful authentication
- **Success**: 200 OK with user object and token
- **Errors**: 400 (validation), 401 (invalid credentials)

## Protected Task API Endpoints

### Authentication Requirements
All task endpoints require:
- Valid JWT token in Authorization header: `Authorization: Bearer {token}`
- User ID in URL path must match user ID in JWT claims
- 401 Unauthorized returned for invalid/missing tokens
- 403 Forbidden returned for cross-user access attempts

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

## Frontend API Client Responsibilities

### Request Handling
- Attach JWT token to all authenticated requests in Authorization header
- Handle token expiration and refresh if applicable
- Retry failed requests with exponential backoff
- Provide loading states during API operations

### Error Handling
- Catch and handle 401 Unauthorized responses by redirecting to login
- Display user-friendly error messages for validation failures
- Handle network connectivity issues gracefully
- Log errors for debugging purposes

### Response Processing
- Parse JSON responses appropriately
- Validate response structure before processing
- Cache responses where appropriate for performance
- Update UI state based on response data

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

## Frontend Routing Contract

### Public Routes
- `/` - Landing page (accessible to all users)
- `/(auth)/sign-up` - User registration page
- `/(auth)/sign-in` - User login page

### Protected Routes
- `/dashboard` - Task management dashboard (requires authentication)
- `/tasks/[id]` - Individual task view (requires authentication)

### Redirect Behavior
- Unauthenticated users accessing protected routes → redirect to `/sign-in`
- Authenticated users accessing auth pages → redirect to `/dashboard`
- Successful authentication → redirect to `/dashboard`
- Successful logout → redirect to `/`

## Validation Rules
- Title: Required, 1-255 characters
- Description: Optional, maximum 1000 characters
- User ID: Must match authenticated user
- Task ID: Must exist and belong to authenticated user