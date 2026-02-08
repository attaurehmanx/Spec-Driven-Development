# Quickstart: Frontend Application & User Experience

## Overview

This guide provides a step-by-step setup and validation of the frontend application for the Multi-User Todo Web Application. The application uses Next.js with App Router, Better Auth for authentication, and consumes the authenticated task API.

## Prerequisites

- Node.js 18+ and npm
- Next.js 16+
- Better Auth configured with JWT plugin
- Backend API endpoints available (from Spec 2)

## Setup

### 1. Environment Configuration

First, set up your environment variables:

```bash
# Copy the example environment file
cp .env.example .env.local

# Edit the .env.local file and set your configuration
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3000
```

### 2. Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run dev
   ```

## Validation Steps

### 1. Authentication Flow Validation

1. Navigate to the sign-up page in your browser: `http://localhost:3000/(auth)/sign-up`
2. Register a new user with valid credentials:
   - Email: test@example.com
   - Password: SecurePass123! (must include uppercase, lowercase, number, and special character)
   - Name: Test User
3. Verify that:
   - Registration succeeds without errors
   - JWT token is stored securely in browser storage
   - User is redirected to the dashboard
   - User information is correctly displayed

### 2. Task Management Flow Validation

1. Navigate to the dashboard page: `http://localhost:3000/dashboard`
2. Create a new task with valid details:
   - Title: "Test Task"
   - Description: "Test Description"
   - Verify that the task appears in the task list immediately
3. Update an existing task:
   - Click edit on a task
   - Modify the title or description
   - Verify that the changes are saved and reflected in the UI
4. Toggle task completion:
   - Click the completion toggle for a task
   - Verify that the task status updates in the UI
5. Delete a task:
   - Click delete on a task
   - Confirm deletion
   - Verify that the task is removed from the list

### 3. API Integration Validation

1. Verify that JWT tokens are attached to all API requests:
   - Open browser developer tools
   - Navigate to Network tab
   - Perform a task operation
   - Verify that the Authorization header contains a valid JWT token
2. Test API error handling:
   - Simulate an API error (e.g., by temporarily stopping the backend)
   - Verify that appropriate error messages are displayed to the user
3. Test authentication state persistence:
   - Refresh the page while logged in
   - Verify that the user remains authenticated

### 4. Security Validation

1. Try to access the dashboard without authentication:
   - Navigate directly to `/dashboard` without logging in
   - Verify that you're redirected to the sign-in page
2. Test expired token handling:
   - Manually expire a JWT token in browser storage
   - Try to make an API request
   - Verify that you're redirected to re-authenticate
3. Test logout functionality:
   - Click the logout button
   - Verify that the JWT token is cleared from storage
   - Verify that you're redirected to the public landing page

## API Integration Points

### Authentication Endpoints (via Better Auth)
- `/api/auth/register` - User registration
- `/api/auth/login` - User login

### Task API Endpoints
- `GET /api/{user_id}/tasks` - List user's tasks
- `POST /api/{user_id}/tasks` - Create a new task
- `GET /api/{user_id}/tasks/{id}` - Get a specific task
- `PUT /api/{user_id}/tasks/{id}` - Update a task
- `DELETE /api/{user_id}/tasks/{id}` - Delete a task
- `PATCH /api/{user_id}/tasks/{id}/complete` - Toggle task completion

## User Experience Requirements

### Responsive Design
- Application must be usable on screen sizes from 320px to 1920px
- Navigation adapts to mobile and desktop layouts
- Touch targets are appropriately sized for mobile devices

### Loading States
- Show loading indicators during API requests
- Disable interactive elements during operations
- Display skeleton screens for content loading

### Error Handling
- Clear error messages for authentication failures
- Appropriate feedback for API errors (401, 403, 404, 500)
- Graceful degradation when API is unavailable

## Troubleshooting

- If authentication isn't working, check that the BETTER_AUTH_URL matches your backend URL
- If API requests are failing, verify that NEXT_PUBLIC_API_BASE_URL is correctly configured
- If tasks aren't appearing, ensure the JWT token is valid and attached to requests
- For styling issues, check that responsive breakpoints are correctly implemented