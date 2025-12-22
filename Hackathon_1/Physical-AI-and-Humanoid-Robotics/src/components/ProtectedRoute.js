import React from 'react';
import { useAuth } from '@clerk/clerk-react';
import { Navigate, useLocation } from '@docusaurus/router';

// Alternative component that uses Clerk's RequireAuth if needed for specific pages
export function RequireAuth({ children }) {
  const { isSignedIn, isLoaded, isSignedOut, user } = useAuth();
  const location = useLocation();

  if (!isLoaded) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
        <div>Loading authentication...</div>
      </div>
    );
  }

  if (isSignedOut || !isSignedIn) {
    // Store the attempted path so we can redirect back after login
    localStorage.setItem('clerk_redirect_path', location.pathname);
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
        <div>
          <p>You need to sign in to access this content.</p>
          <a href="/sign-in">Sign In</a>
        </div>
      </div>
    );
  }

  // Additional check: ensure user object is available
  if (!user) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100vh' }}>
        <div>
          <p>Authentication issue. Please sign in again.</p>
          <a href="/sign-in">Sign In</a>
        </div>
      </div>
    );
  }

  return children;
}