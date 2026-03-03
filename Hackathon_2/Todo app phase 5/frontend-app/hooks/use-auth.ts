import { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { apiClient } from '@/services/api-client';
import { tokenStorage } from '@/lib/token-storage';

interface UseAuthReturn {
  isAuthenticated: boolean;
  isLoading: boolean;
  user: any | null; // Replace with proper User type
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, firstName: string, lastName: string) => Promise<void>;
  signOut: () => Promise<void>;
  updateUserProfile: (firstName: string, lastName: string, phone?: string, address?: string, avatar?: string) => Promise<void>;
  uploadUserAvatar: (userId: string, avatarFile: File) => Promise<any>;
  deleteUserAccount: () => Promise<void>;
  error: string | null;
}

const useAuth = (): UseAuthReturn => {
  const router = useRouter();
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [user, setUser] = useState<any | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Function to fetch user data using the token
  const fetchUserData = async (token: string) => {
    try {
      // Use the token verification method from tokenStorage
      let userData = await tokenStorage.verifyAndValidateToken();

      // If direct verification fails, try using refresh token
      if (!userData) {
        const refreshToken = tokenStorage.getRefreshToken();
        if (refreshToken) {
          // Try to get a new access token using the refresh token
          try {
            const refreshResponse = await fetch(`${process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8001'}/auth/refresh`, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({ refresh_token: refreshToken }),
            });

            if (refreshResponse.ok) {
              const newTokens = await refreshResponse.json();

              // Store the new tokens
              if (newTokens.access_token) {
                tokenStorage.setAccessToken(newTokens.access_token);
              }
              if (newTokens.refresh_token) {
                tokenStorage.setRefreshToken(newTokens.refresh_token);
              }

              // Now try to verify the new access token
              userData = await tokenStorage.verifyAndValidateToken();
            }
          } catch (refreshErr) {
            console.error('Token refresh failed:', refreshErr);
            // If refresh fails, clear all tokens
            tokenStorage.clearTokens();
            return null;
          }
        } else {
          // No refresh token available, clear the expired token
          tokenStorage.clearTokens();
          return null;
        }
      }

      if (userData && userData.user) {
        return userData.user; // Return the actual user object from the response
      } else {
        return null;
      }
    } catch (err) {
      console.error('Failed to fetch user data:', err);
      tokenStorage.clearTokens();
      return null;
    }
  };

  // Check if user is authenticated on mount
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        // Use the improved token verification method
        const token = tokenStorage.getAccessToken();
        if (token) {
          // Fetch user data using the token - this will handle token validation
          const userData = await fetchUserData(token);
          if (userData) {
            setIsAuthenticated(true);
            setUser(userData);
          } else {
            // If token is invalid, the fetchUserData function would have cleared tokens
            setIsAuthenticated(false);
            setUser(null);
          }
        } else {
          // If no token exists, ensure we set the proper initial state
          setIsAuthenticated(false);
          setUser(null);
        }
      } catch (err) {
        console.error('Auth check failed:', err);
        setIsAuthenticated(false);
        setUser(null);
      } finally {
        setIsLoading(false);
      }
    };

    // Listen for token invalidation events
    const handleTokenInvalidation = () => {
      setIsAuthenticated(false);
      setUser(null);
      tokenStorage.clearTokens();
      router.push('/auth/sign-in');
    };

    checkAuthStatus();

    // Add event listener
    window.addEventListener('auth-token-invalidated', handleTokenInvalidation);

    // Cleanup function
    return () => {
      window.removeEventListener('auth-token-invalidated', handleTokenInvalidation);
    };
  }, []);

  const signIn = async (email: string, password: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await apiClient.signIn(email, password);

      if (response.access_token) {
        // Use the same token storage mechanism as the API client
        tokenStorage.setAccessToken(response.access_token);

        // Fetch user data using the token
        const userData = await fetchUserData(response.access_token);
        setIsAuthenticated(true);
        setUser(userData);
      }
    } catch (err: any) {
      console.error('Sign in failed:', err);
      setError(err.message || 'Sign in failed');
      setIsAuthenticated(false);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  };

  const signUp = async (email: string, password: string, firstName: string, lastName: string) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await apiClient.signUp(email, password, firstName, lastName);

      if (response.access_token) {
        // Use the same token storage mechanism as the API client
        tokenStorage.setAccessToken(response.access_token);

        // Fetch user data using the token
        const userData = await fetchUserData(response.access_token);
        setIsAuthenticated(true);
        setUser(userData);
      }
    } catch (err: any) {
      console.error('Sign up failed:', err);
      setError(err.message || 'Sign up failed');
      setIsAuthenticated(false);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  };

  const signOut = async () => {
    setIsLoading(true);

    // Clear tokens using the token storage mechanism
    tokenStorage.clearTokens();

    setIsAuthenticated(false);
    setUser(null);
    setIsLoading(false);

    // Redirect to sign in page
    router.push('/auth/sign-in');
  };

  const updateUserProfile = async (firstName: string, lastName: string, phone?: string, address?: string, avatar?: string) => {
    setIsLoading(true);
    setError(null);

    try {
      if (!user) {
        throw new Error('No user is currently authenticated');
      }

      const profileData: any = {
        first_name: firstName,
        last_name: lastName
      };

      // Only include phone and address if they are provided
      if (phone !== undefined) {
        profileData.phone = phone;
      }
      if (address !== undefined) {
        profileData.address = address;
      }
      if (avatar !== undefined) {
        profileData.avatar = avatar;
      }

      const response = await apiClient.updateUserProfile(user.id, profileData);

      // Update the user in the state with new data from the response
      // The updated user data comes from the API response
      const updatedUser = response.user || {
        ...user,
        first_name: firstName,
        last_name: lastName,
        phone: phone,
        address: address,
        avatar: avatar
      };

      setUser(updatedUser);

      // Update token storage if needed
      tokenStorage.setAccessToken(tokenStorage.getAccessToken());

      console.log('Profile updated successfully:', response);
    } catch (err: any) {
      console.error('Profile update failed:', err);
      setError(err.message || 'Profile update failed');
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const uploadUserAvatar = async (userId: string, avatarFile: File) => {
    setIsLoading(true);
    setError(null);

    try {
      if (!user) {
        throw new Error('No user is currently authenticated');
      }

      const response = await apiClient.uploadUserAvatar(userId, avatarFile);

      // Update the user state with the new avatar URL
      if (response.avatar_url) {
        setUser((prevUser: any) => prevUser ? { ...prevUser, avatar: response.avatar_url } : null);
      }

      console.log('Avatar uploaded successfully:', response);
      return response;
    } catch (err: any) {
      console.error('Avatar upload failed:', err);
      setError(err.message || 'Avatar upload failed');
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const deleteUserAccount = async () => {
    setIsLoading(true);
    setError(null);

    try {
      if (!user) {
        throw new Error('No user is currently authenticated');
      }

      const response = await apiClient.deleteUserAccount(user.id);
      console.log('User account deleted successfully:', response);

      // Clear tokens and state
      tokenStorage.clearTokens();
      setIsAuthenticated(false);
      setUser(null);

      // Redirect to home page
      router.push('/');
    } catch (err: any) {
      console.error('Account deletion failed:', err);
      setError(err.message || 'Account deletion failed');
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  return {
    isAuthenticated,
    isLoading,
    user,
    signIn,
    signUp,
    signOut,
    updateUserProfile,
    uploadUserAvatar,
    deleteUserAccount,
    error,
  };
};

export default useAuth;