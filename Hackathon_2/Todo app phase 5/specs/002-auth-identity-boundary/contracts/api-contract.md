# API Contract: Authentication & Identity Boundary

## Authentication Endpoints (Handled by Better Auth)

### Registration
- **POST** `/api/auth/register`
- **Request**: User registration details
- **Response**: JWT token upon successful registration

### Login
- **POST** `/api/auth/login`
- **Request**: User credentials
- **Response**: JWT token upon successful authentication

## Protected API Endpoints (Require JWT Authentication)

### General Authentication Requirements
- **Header**: `Authorization: Bearer {jwt_token}`
- **Verification**: JWT signature, expiry, and claims validation
- **Identity Extraction**: User ID from JWT 'sub' claim

### Response Codes
- **200 OK**: Valid JWT, authorized access
- **401 Unauthorized**: Missing, invalid, or expired JWT
- **403 Forbidden**: JWT valid but user lacks specific resource access (future scope)

## User Identity Verification Contract

### Request Processing Flow
1. Extract JWT from Authorization header
2. Verify JWT signature using BETTER_AUTH_SECRET
3. Validate JWT expiry and integrity
4. Extract user identity from JWT claims
5. Compare URL user_id with JWT subject (sub) claim
6. Proceed with request if identities match, reject otherwise

### Identity Claims Expected
- **sub**: User identifier (matches user_id in URL)
- **email**: User email address
- **exp**: Token expiration timestamp