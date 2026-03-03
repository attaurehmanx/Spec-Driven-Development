# Task Management API

A secure, multi-user task management API built with FastAPI, SQLModel, and PostgreSQL.

## Features

- **Multi-user support**: Each user has isolated tasks
- **Authentication**: JWT-based authentication
- **Authorization**: User access control to prevent cross-user data access
- **RESTful API**: Standard HTTP methods and status codes
- **Persistent storage**: PostgreSQL database with connection pooling
- **Health checks**: Built-in health check endpoints

## API Endpoints

### Authentication Required

All endpoints require a valid JWT token in the Authorization header:
```
Authorization: Bearer <your-jwt-token>
```

### Base URL
```
/api/{user_id}/
```

### Task Endpoints

#### List User Tasks
```
GET /api/{user_id}/tasks
```
- **Description**: Retrieve all tasks for the specified user
- **Response**: Array of Task objects
- **Authentication**: Required, user_id must match authenticated user

#### Create Task
```
POST /api/{user_id}/tasks
```
- **Description**: Create a new task for the specified user
- **Request Body**:
```json
{
  "title": "Task title",
  "description": "Optional task description",
  "completed": false
}
```
- **Response**: Created Task object
- **Authentication**: Required, user_id must match authenticated user

#### Get Task
```
GET /api/{user_id}/tasks/{task_id}
```
- **Description**: Retrieve a specific task
- **Response**: Task object
- **Authentication**: Required, user_id must match authenticated user

#### Update Task
```
PUT /api/{user_id}/tasks/{task_id}
```
- **Description**: Update a specific task
- **Request Body**:
```json
{
  "title": "Updated title",
  "description": "Updated description",
  "completed": true
}
```
- **Response**: Updated Task object
- **Authentication**: Required, user_id must match authenticated user

#### Delete Task
```
DELETE /api/{user_id}/tasks/{task_id}
```
- **Description**: Delete a specific task
- **Response**: No content (204)
- **Authentication**: Required, user_id must match authenticated user

#### Toggle Task Completion
```
PATCH /api/{user_id}/tasks/{task_id}/complete
```
- **Description**: Toggle the completion status of a task
- **Response**: Updated Task object
- **Authentication**: Required, user_id must match authenticated user

### Health Check Endpoints

#### General Health Check
```
GET /health
```
- **Description**: Check overall service health

#### Database Health Check
```
GET /health/db
```
- **Description**: Check database connectivity

#### Readiness Check
```
GET /health/ready
```
- **Description**: Check if service is ready to accept traffic

## Task Object Structure

```json
{
  "id": 1,
  "title": "Sample Task",
  "description": "A sample task description",
  "completed": false,
  "user_id": 123,
  "created_at": "2023-01-01T00:00:00",
  "updated_at": "2023-01-01T00:00:00"
}
```

## Security

- All endpoints require JWT authentication
- Users can only access their own tasks
- Unauthorized access attempts return 403 Forbidden
- JWT tokens are validated on each request

## Installation

1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Set up environment variables in `.env`:
   ```
   DATABASE_URL=postgresql://username:password@localhost:5432/task_management
   JWT_SECRET_KEY=your-super-secret-jwt-key-change-in-production
   JWT_ALGORITHM=HS256
   JWT_ACCESS_TOKEN_EXPIRE_MINUTES=30
   ```
4. Start the application: `python -m backend.main`

## Environment Variables

- `DATABASE_URL`: PostgreSQL connection string
- `JWT_SECRET_KEY`: Secret key for JWT signing (change in production!)
- `JWT_ALGORITHM`: Algorithm for JWT (default: HS256)
- `JWT_ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 30)

## Error Handling

- `401 Unauthorized`: Invalid or missing JWT token
- `403 Forbidden`: User trying to access resources they don't own
- `404 Not Found`: Resource not found
- `422 Unprocessable Entity`: Invalid request data
- `500 Internal Server Error`: Unexpected server error

## Database Schema

The application uses SQLModel to define the Task entity:

- `id` (Primary Key, Auto-increment)
- `title` (String, Required, max 255 chars)
- `description` (String, Optional, max 1000 chars)
- `completed` (Boolean, Default: false)
- `user_id` (Integer, Required, Foreign Key with index)
- `created_at` (DateTime, Required)
- `updated_at` (DateTime, Required)

## Development

To run tests:
```
pytest
```

To run the application in development mode:
```
uvicorn backend.main:app --reload
```

## Deployment

For production deployment, ensure you:
1. Use a strong JWT secret key
2. Configure SSL/TLS
3. Set up proper logging
4. Use a production-grade database
5. Implement proper monitoring