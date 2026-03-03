# Quickstart: Task API & Persistence Layer

## Overview

This guide outlines the task management API with user isolation using FastAPI, SQLModel, and Neon Serverless PostgreSQL. The API enforces user ownership of tasks through authenticated identity derived from JWT claims.

## API Endpoints

### Task Operations
- **GET** `/api/{user_id}/tasks` - List all tasks for the authenticated user
- **POST** `/api/{user_id}/tasks` - Create a new task for the authenticated user
- **GET** `/api/{user_id}/tasks/{id}` - Get a specific task owned by the user
- **PUT** `/api/{user_id}/tasks/{id}` - Update a task owned by the user
- **DELETE** `/api/{user_id}/tasks/{id}` - Delete a task owned by the user
- **PATCH** `/api/{user_id}/tasks/{id}/complete` - Toggle completion status of a task

## Authentication Requirements

- All endpoints require a valid JWT token in the Authorization header: `Authorization: Bearer {token}`
- The user_id in the URL must match the user_id in the JWT claims
- Requests with mismatched user_ids will be rejected with 403 Forbidden

## Task Entity Structure

Each task includes:
- `id`: Unique identifier
- `title`: Task title (required)
- `description`: Task description (optional)
- `completed`: Boolean completion status
- `created_at`: Timestamp when task was created
- `updated_at`: Timestamp when task was last updated
- `user_id`: Owner's user identifier

## Error Responses

- **401 Unauthorized**: No valid JWT token provided
- **403 Forbidden**: User attempting to access tasks not owned by them
- **404 Not Found**: Requested task does not exist
- **422 Unprocessable Entity**: Invalid request data

## Security Guarantees

- User data isolation: Users can only access their own tasks
- Ownership validation: All operations verify user owns the task
- Authentication enforcement: All endpoints require valid JWT
- Cross-user access prevention: Robust validation prevents unauthorized access