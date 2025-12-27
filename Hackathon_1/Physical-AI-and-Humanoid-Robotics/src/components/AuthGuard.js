import React, { useEffect, useState } from 'react';
import { useAuth } from '@clerk/clerk-react';
import { Redirect } from '@docusaurus/router';
import { useLocation } from '@docusaurus/router';

// This component checks authentication after ClerkProvider is set up
export function AuthGuard({ children, publishableKey }) {
  const { isSignedIn, isLoaded, isSignedOut } = useAuth();
  const location = useLocation();
  const [checked, setChecked] = useState(false);

  // Check if current path is a public page (sign-in, sign-up, etc.)
  const isPublicPage = location.pathname.startsWith('/sign-in') || location.pathname.startsWith('/sign-up');

  useEffect(() => {
    if (isLoaded) {
      setChecked(true);
    }
  }, [isLoaded]);

  if (!checked && !isPublicPage) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', minHeight: '100vh' }}>
        Loading authentication...
      </div>
    );
  }

  // If user is signed in and on a public auth page, redirect to home
  if (isLoaded && isSignedIn && isPublicPage) {
    // Check if there's a stored redirect path from before sign-in
    const storedRedirectPath = localStorage.getItem('clerk_redirect_path');
    if (storedRedirectPath && storedRedirectPath !== '/sign-in' && storedRedirectPath !== '/sign-up') {
      localStorage.removeItem('clerk_redirect_path'); // Clean up
      return <Redirect to={storedRedirectPath} />;
    }
    // Default redirect to home if no specific path
    return <Redirect to="/" />;
  }

  // If user is not signed in and not on a public page, redirect to sign-in
  if (isLoaded && !isSignedIn && !isPublicPage) {
    // Store the attempted path so we can redirect back after login
    localStorage.setItem('clerk_redirect_path', location.pathname);
    return <Redirect to="/sign-in" />;
  }

  return children;
}