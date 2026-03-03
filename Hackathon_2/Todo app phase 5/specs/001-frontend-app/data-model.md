# Data Model: Frontend Application & User Experience

## Task Entity

**Task**: User's to-do item with properties including ID, title, description, completion status, and creation timestamp
- **id**: Unique identifier (string or number)
- **title**: Task title (string, required, max length 255)
- **description**: Task description (string, optional, max length 1000)
- **completed**: Completion status (boolean, default false)
- **createdAt**: Creation timestamp (ISO string)
- **updatedAt**: Last update timestamp (ISO string)

## Authenticated User Entity

**Authenticated User**: Verified identity managed by Better Auth with JWT token for API authentication
- **id**: User unique identifier (string)
- **email**: User email address (string)
- **name**: User display name (string, optional)
- **isLoggedIn**: Authentication status (boolean)
- **token**: JWT token for API authentication (string, optional)

## User Session Entity

**User Session**: Authenticated state maintained by JWT token with associated user data and preferences
- **userId**: Associated user ID (string)
- **token**: JWT token (string)
- **expiresAt**: Token expiration timestamp (ISO string)
- **lastActivity**: Last activity timestamp (ISO string)
- **preferences**: User preferences object (optional)

## API Response Format

**Standard API Response**:
- **data**: Response data (any type based on request)
- **success**: Success status (boolean)
- **message**: Optional message (string)
- **errors**: Optional array of errors (array of objects with code and message)

## Error Response Format

**Standard Error Response**:
- **error**: Error code (string)
- **message**: Error description (string)
- **details**: Additional error details (object, optional)

## Validation Rules

**Task Creation**:
- Title is required and must be between 1-255 characters
- Description is optional and must be under 1000 characters if provided
- Completed status defaults to false

**User Authentication**:
- Email must be valid email format
- Password must meet security requirements (defined by Better Auth)
- JWT token must be valid and not expired

## State Management Patterns

**Authentication State**:
- Global state managed by React Context
- Includes user data and authentication status
- Persists across page navigations

**Task State**:
- Managed at component level or with custom hooks
- Cached to prevent unnecessary API calls
- Synchronized with backend on mutations