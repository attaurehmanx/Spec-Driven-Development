'use client';

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { apiClient } from '../lib/api/client';

interface UserProfile {
  id: string;
  email: string;
  name?: string;
  createdAt: Date;
  updatedAt: Date;
}

interface UserProfileContextType {
  profile: UserProfile | null;
  loading: boolean;
  error: string | null;
  refreshProfile: () => Promise<void>;
  updateProfile: (data: Partial<UserProfile>) => Promise<void>;
}

const UserProfileContext = createContext<UserProfileContextType | undefined>(undefined);

interface UserProfileProviderProps {
  children: ReactNode;
}

export const UserProfileProvider: React.FC<UserProfileProviderProps> = ({ children }) => {
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  // Load user profile on initial render
  useEffect(() => {
    const loadProfile = async () => {
      try {
        setLoading(true);
        setError(null);

        // Get the user ID from the auth token or elsewhere
        // This would depend on your specific implementation
        // For now, we'll simulate fetching the profile

        // In a real implementation, you would get the user ID from the auth context
        // and call an API endpoint to fetch the user profile
        // const response = await apiClient.getUserProfile(userId);
        // setProfile(response.profile);

        // For now, set a mock profile
        setProfile({
          id: 'mock-user-id',
          email: 'user@example.com',
          name: 'Mock User',
          createdAt: new Date(),
          updatedAt: new Date(),
        });
      } catch (err: any) {
        setError(err.message || 'Failed to load user profile');
        console.error('Error loading user profile:', err);
      } finally {
        setLoading(false);
      }
    };

    loadProfile();
  }, []);

  const refreshProfile = async () => {
    try {
      setLoading(true);
      setError(null);

      // In a real implementation:
      // const response = await apiClient.getUserProfile(profile!.id);
      // setProfile(response.profile);
    } catch (err: any) {
      setError(err.message || 'Failed to refresh user profile');
      console.error('Error refreshing user profile:', err);
    } finally {
      setLoading(false);
    }
  };

  const updateProfile = async (data: Partial<UserProfile>) => {
    try {
      setLoading(true);
      setError(null);

      // In a real implementation:
      // const response = await apiClient.updateUserProfile(profile!.id, data);
      // setProfile(response.profile);
    } catch (err: any) {
      setError(err.message || 'Failed to update user profile');
      console.error('Error updating user profile:', err);
    } finally {
      setLoading(false);
    }
  };

  const value = {
    profile,
    loading,
    error,
    refreshProfile,
    updateProfile,
  };

  return (
    <UserProfileContext.Provider value={value}>
      {children}
    </UserProfileContext.Provider>
  );
};

export const useUserProfile = (): UserProfileContextType => {
  const context = useContext(UserProfileContext);
  if (context === undefined) {
    throw new Error('useUserProfile must be used within a UserProfileProvider');
  }
  return context;
};