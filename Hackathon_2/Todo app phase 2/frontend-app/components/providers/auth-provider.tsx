'use client';

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { UserSession, AuthUIState } from '../../../types';
import { apiClient } from '../../services/api-client';
import { tokenStorage } from '@/lib/token-storage';

interface AuthContextType {
  session: UserSession;
  authState: AuthUIState;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string) => Promise<void>;
  signOut: () => Promise<void>;
  updateSession: (session: Partial<UserSession>) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [session, setSession] = useState<UserSession>({
    userId: '',
    email: '',
    token: '',
    expiresAt: new Date(0),
    isLoggedIn: false,
    isLoading: true,
  });

  const [authState, setAuthState] = useState<AuthUIState>({
    loading: false,
    error: undefined,
    successMessage: undefined,
    showPasswordReset: false,
  });

  // Load session from localStorage on initial render
  useEffect(() => {
    // Try to get token from shared token storage first (fallback to localStorage)
    let storedToken = tokenStorage.getAccessToken();
    if (!storedToken) {
      storedToken = localStorage.getItem('auth_token');
    }

    const storedUserData = localStorage.getItem('user_data');

    if (storedToken && storedUserData) {
      try {
        const userData = JSON.parse(storedUserData);
        setSession({
          userId: userData.userId,
          email: userData.email,
          token: storedToken,
          expiresAt: new Date(userData.expiresAt),
          isLoggedIn: true,
          isLoading: false,
        });
      } catch (error) {
        console.error('Failed to parse stored user data:', error);
        // Clear invalid data
        tokenStorage.clearTokens(); // Clear shared token storage
        localStorage.removeItem('auth_token');
        localStorage.removeItem('user_data');
        setSession({
          userId: '',
          email: '',
          token: '',
          expiresAt: new Date(0),
          isLoggedIn: false,
          isLoading: false,
        });
      }
    } else {
      setSession((prev) => ({
        ...prev,
        isLoading: false,
      }));
    }
  }, []);

  const updateSession = (sessionUpdate: Partial<UserSession>) => {
    setSession((prev) => ({ ...prev, ...sessionUpdate }));
  };

  const signIn = async (email: string, password: string) => {
    setAuthState((prev) => ({ ...prev, loading: true, error: undefined }));

    try {
      const response = await apiClient.signIn(email, password);

      if (response.token) {
        const { token, user } = response;

        // Store token in localStorage for backward compatibility
        localStorage.setItem('auth_token', token);
        // Also store in the shared token storage system
        tokenStorage.setAccessToken(token);

        // Store user data in localStorage
        const userData = {
          userId: user.id,
          email: user.email,
          expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString(), // 24 hours
        };
        localStorage.setItem('user_data', JSON.stringify(userData));

        setSession({
          userId: user.id,
          email: user.email,
          token,
          expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
          isLoggedIn: true,
          isLoading: false,
        });

        setAuthState((prev) => ({
          ...prev,
          loading: false,
          successMessage: 'Successfully signed in',
        }));
      } else {
        throw new Error('Invalid response from server');
      }
    } catch (error: any) {
      console.error('Sign in error:', error);
      setAuthState((prev) => ({
        ...prev,
        loading: false,
        error: error.message || 'Failed to sign in',
      }));
      throw error;
    }
  };

  const signUp = async (email: string, password: string) => {
    setAuthState((prev) => ({ ...prev, loading: true, error: undefined }));

    try {
      const response = await apiClient.signUp(email, password);

      if (response.token) {
        const { token, user } = response;

        // Store token in localStorage for backward compatibility
        localStorage.setItem('auth_token', token);
        // Also store in the shared token storage system
        tokenStorage.setAccessToken(token);

        // Store user data in localStorage
        const userData = {
          userId: user.id,
          email: user.email,
          expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString(), // 24 hours
        };
        localStorage.setItem('user_data', JSON.stringify(userData));

        setSession({
          userId: user.id,
          email: user.email,
          token,
          expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000), // 24 hours
          isLoggedIn: true,
          isLoading: false,
        });

        setAuthState((prev) => ({
          ...prev,
          loading: false,
          successMessage: 'Successfully signed up',
        }));
      } else {
        throw new Error('Invalid response from server');
      }
    } catch (error: any) {
      console.error('Sign up error:', error);
      setAuthState((prev) => ({
        ...prev,
        loading: false,
        error: error.message || 'Failed to sign up',
      }));
      throw error;
    }
  };

  const signOut = async () => {
    setAuthState((prev) => ({ ...prev, loading: true }));

    try {
      await apiClient.signOut();
    } catch (error) {
      console.error('Sign out error (proceeding anyway):', error);
      // Continue with local cleanup even if API call fails
    }

    // Clear local storage
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_data');
    // Also clear the shared token storage
    tokenStorage.clearTokens();

    // Update session
    setSession({
      userId: '',
      email: '',
      token: '',
      expiresAt: new Date(0),
      isLoggedIn: false,
      isLoading: false,
    });

    setAuthState((prev) => ({
      ...prev,
      loading: false,
      successMessage: 'Successfully signed out',
    }));
  };

  const value = {
    session,
    authState,
    signIn,
    signUp,
    signOut,
    updateSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};