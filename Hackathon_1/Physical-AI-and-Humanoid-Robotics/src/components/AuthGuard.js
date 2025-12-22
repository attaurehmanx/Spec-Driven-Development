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

  // If user is not signed in and not on a public page, redirect to sign-in
  if (isLoaded && !isSignedIn && !isPublicPage) {
    return <Redirect to="/sign-in" />;
  }

  return children;
}