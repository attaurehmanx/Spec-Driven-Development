# Data Model: Clerk Authentication for Docusaurus

## Entities

### Authentication State
**Description**: Represents the current authentication status of the user
**Fields**:
- isAuthenticated: boolean (indicates if user is currently authenticated)
- userId: string (unique identifier for authenticated user, null if unauthenticated)
- sessionId: string (current session identifier, null if unauthenticated)
- user: object (user profile data when authenticated)

### Protected Route
**Description**: A Docusaurus page or route that requires authentication to access
**Fields**:
- path: string (the URL path that requires protection)
- requiresAuth: boolean (always true for protected routes)
- redirectPath: string (where to redirect if not authenticated, typically /sign-in)

### Public Route
**Description**: Authentication-related routes that remain accessible without authentication
**Fields**:
- path: string (the URL path that remains public)
- requiresAuth: boolean (always false for public routes)
- allowed: boolean (always true for public routes)

## State Transitions

### Authentication Flow
1. **Unauthenticated State**: User visits any route
2. **Check Authentication**: System verifies authentication status
3. **If Unauthenticated**: Redirect to /sign-in
4. **If Authenticated**: Allow access to requested route
5. **On Sign-out**: Return to unauthenticated state

## Validation Rules

- All routes except /sign-in and /sign-up must require authentication
- Authentication state must be checked before rendering protected content
- Redirect must occur before any sensitive content is displayed
- Environment variable CLERK_PUBLISHABLE_KEY must be present for initialization