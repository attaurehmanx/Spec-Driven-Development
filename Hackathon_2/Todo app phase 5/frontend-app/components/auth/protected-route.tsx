'use client';

import React, { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import useAuth from '../../hooks/use-auth';
import LoadingSpinner from '../ui/loading-spinner';

interface ProtectedRouteProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  redirectTo?: string;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  fallback = <LoadingSpinner label="Checking authentication..." />,
  redirectTo = '/auth/sign-in',
}) => {
  const { isAuthenticated, isLoading, user } = useAuth();
  const router = useRouter();

  useEffect(() => {
    if (!isLoading && (!isAuthenticated || !user)) {
      router.push(redirectTo);
    }
  }, [isAuthenticated, isLoading, user, redirectTo, router]);

  // Show fallback while checking authentication status
  if (isLoading) {
    return fallback;
  }

  // Redirect if not authenticated or user is null
  if (!isAuthenticated || !user) {
    return null;
  }

  // Render children if authenticated
  return <>{children}</>;
};

export default ProtectedRoute;