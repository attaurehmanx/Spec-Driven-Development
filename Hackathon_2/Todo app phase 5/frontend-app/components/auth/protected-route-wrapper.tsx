'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import useAuth from '../../hooks/use-auth';

interface ProtectedRouteWrapperProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  redirectTo?: string;
}

const ProtectedRouteWrapper: React.FC<ProtectedRouteWrapperProps> = ({
  children,
  fallback = (
    <div className="min-h-screen flex items-center justify-center">
      <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
    </div>
  ),
  redirectTo = '/auth/sign-in',
}) => {
  const { isAuthenticated, isLoading, error } = useAuth();
  const router = useRouter();

  useEffect(() => {
    if (!isLoading && (!isAuthenticated || error)) {
      router.push(redirectTo);
    }
  }, [isAuthenticated, isLoading, error, redirectTo, router]);

  if (isLoading) {
    return fallback;
  }

  if (!isAuthenticated || error) {
    return null;
  }

  return <>{children}</>;
};

export default ProtectedRouteWrapper;