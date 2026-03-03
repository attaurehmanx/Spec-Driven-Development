# Data Model: Authentication & Identity Boundary

## JWT Token Structure

**JWT Token**: Self-contained credential containing user identity information
- **Header**: Algorithm (HS256) and token type
- **Payload**: Claims including user identity
  - `sub`: Subject (user ID)
  - `email`: User email address
  - `name`: User display name
  - `iat`: Issued at timestamp
  - `exp`: Expiration timestamp
- **Signature**: Verified using BETTER_AUTH_SECRET

## Authenticated User Identity

**Authenticated User**: Verified identity derived from JWT claims
- **userId**: Unique identifier extracted from JWT 'sub' claim
- **email**: Email address extracted from JWT
- **authenticatedAt**: Timestamp when authentication was verified
- **isValid**: Boolean indicating JWT validity status

## Protected Endpoint Access

**Protected Endpoint Request**:
- **headers.authorization**: Bearer token containing JWT
- **userIdFromUrl**: User ID from URL parameter (must match JWT identity)
- **verifiedUserId**: User ID extracted from verified JWT claims
- **accessGranted**: Boolean indicating if access should be allowed