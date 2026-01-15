# Quickstart: Authentication & Identity Boundary

## Overview

This guide provides a step-by-step setup and validation of the authentication and identity boundary system. The system uses Better Auth for frontend authentication and JWT tokens for secure communication between frontend and backend services.

## Prerequisites

- Node.js 18+ and npm
- Python 3.9+
- Better Auth configured with JWT plugin
- BETTER_AUTH_SECRET environment variable set

## Setup

### 1. Environment Configuration

First, set up your environment variables:

```bash
# Copy the example environment file
cp .env.example .env

# Edit the .env file and set your secret
BETTER_AUTH_SECRET=your-super-secret-jwt-key-here-replace-with-secure-value
BETTER_AUTH_URL=http://localhost:3000
```

### 2. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Start the backend server:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

### 3. Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the frontend development server:
   ```bash
   npm run dev
   ```

## Validation Steps

### 1. Registration Flow Validation

1. Navigate to the registration page in your frontend application
2. Register a new user with valid credentials:
   - Email: test@example.com
   - Password: SecurePass123! (must include uppercase, lowercase, number, and special character)
   - Name: Test User
3. Verify that:
   - Registration succeeds without errors
   - JWT token is issued and stored securely
   - User is redirected to the dashboard or appropriate page

### 2. Login Flow Validation

1. Navigate to the login page
2. Log in with the registered user's credentials
3. Verify that:
   - Login succeeds without errors
   - JWT token is updated/refreshed if needed
   - User is redirected to the dashboard or appropriate page
   - User information is correctly displayed

### 3. Protected API Access Validation

1. Try to access a protected endpoint without a token:
   - Make a request to `/protected/user-profile/{user_id}`
   - Verify that you receive a 401 Unauthorized response

2. Access a protected endpoint with a valid token:
   - Ensure the token is attached to the request header as `Authorization: Bearer {token}`
   - Verify that the request succeeds
   - Verify that you can only access your own user profile (user_id in URL matches token subject)

3. Try to access another user's data with your token:
   - Verify that you receive a 403 Forbidden response

### 4. Token Expiration Validation

1. Wait for the token to expire (default is 24 hours, but you can test with shorter times)
2. Try to access a protected endpoint with an expired token
3. Verify that:
   - You receive a 401 Unauthorized response
   - The frontend redirects to the login page
   - The token is cleared from storage

### 5. Error Handling Validation

1. Try to register with invalid data:
   - Invalid email format
   - Weak password
   - Missing required fields
2. Verify that appropriate error messages are returned

3. Try to log in with invalid credentials:
   - Verify that appropriate error messages are returned

## API Endpoints

### Authentication Endpoints
- `POST /auth/register` - User registration
- `POST /auth/login` - User login

### Protected Endpoints
- `GET /protected/user-profile/{user_id}` - Get user profile
- `GET /protected/user-data/{user_id}` - Get user data
- `POST /protected/user-data/{user_id}` - Update user data

## Security Considerations

- JWT tokens are stateless and validated using the shared BETTER_AUTH_SECRET
- All protected endpoints require a valid JWT token in the Authorization header
- User ID in URL parameters is validated against the JWT subject to prevent unauthorized access
- Tokens are stored in sessionStorage for better security than localStorage
- Token expiration is enforced both in JWT claims and client-side validation

## Troubleshooting

- If you receive 401 Unauthorized errors, ensure your JWT token is valid and not expired
- If you receive 403 Forbidden errors, verify that the user_id in the URL matches the user_id in your JWT token
- If authentication isn't working, check that the BETTER_AUTH_SECRET is consistent between frontend and backend
- If frontend components can't connect to the backend, ensure the API base URL is correctly configured