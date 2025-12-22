# Quickstart: Clerk Authentication for Docusaurus

## Prerequisites

1. Node.js and npm installed
2. Docusaurus project already set up
3. Clerk account and application created

## Setup Steps

### 1. Install Clerk Dependencies

```bash
npm install @clerk/clerk-react
```

### 2. Create Clerk Application

1. Go to https://dashboard.clerk.com/
2. Create a new application
3. Copy the Publishable Key

### 3. Configure Environment Variables

Add to your `.env` file:
```
CLERK_PUBLISHABLE_KEY=your_publishable_key_here
```

### 4. Wrap Application with ClerkProvider

In your Docusaurus app, wrap the root component with ClerkProvider:

```jsx
import { ClerkProvider } from '@clerk/clerk-react';

function App() {
  return (
    <ClerkProvider publishableKey={process.env.CLERK_PUBLISHABLE_KEY}>
      {/* Your Docusaurus content */}
    </ClerkProvider>
  );
}
```

### 5. Create Sign-in and Sign-up Pages

Create `pages/sign-in.jsx` and `pages/sign-up.jsx` as public routes.

### 6. Protect All Other Routes

Use Clerk's components to protect routes:

```jsx
import { SignedIn, SignedOut, RedirectToSignIn } from '@clerk/clerk-react';

function ProtectedContent() {
  return (
    <>
      <SignedOut>
        <RedirectToSignIn />
      </SignedOut>
      <SignedIn>
        {/* Your protected content */}
      </SignedIn>
    </>
  );
}
```

## Testing

1. Visit any Docusaurus page - should redirect to sign-in if not authenticated
2. Visit /sign-in - should be accessible without authentication
3. Sign in and verify access to protected content