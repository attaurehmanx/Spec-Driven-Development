import { tokenStorage } from '@/lib/token-storage';

/**
 * API Client with JWT Token Attachment
 * Handles making authenticated API requests with automatic token inclusion
 */
class ApiClient {
  constructor(baseURL = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8000') {
    this.baseURL = baseURL;
  }

  /**
   * Internal method to make API requests
   */
  async request(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;

    // Prepare headers
    const headers = {
      'Content-Type': 'application/json',
      ...options.headers,
    };

    // Add authorization header if token exists and is not expired
    const token = tokenStorage.getAccessToken();
    if (token && !tokenStorage.isTokenExpired(token)) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    // Merge options
    const requestOptions = {
      ...options,
      headers,
    };

    try {
      let response = await fetch(url, requestOptions);

      // Handle 401 Unauthorized responses - attempt token refresh
      if (response.status === 401) {
        const refreshToken = tokenStorage.getRefreshToken();

        // If we have a refresh token, try to get a new access token
        if (refreshToken) {
          const refreshSuccess = await this.refreshAccessToken(refreshToken);

          if (refreshSuccess) {
            // Retry the original request with the new token
            const newToken = tokenStorage.getAccessToken();
            if (newToken) {
              // Update the authorization header with the new token
              requestOptions.headers['Authorization'] = `Bearer ${newToken}`;
              response = await fetch(url, requestOptions);

              // If the retry succeeds, proceed normally
              if (response.ok) {
                return await this.parseResponse(response);
              }
            }
          }
        }

        // If refresh failed or no refresh token, clear all tokens and redirect
        tokenStorage.clearTokens();

        // Update auth state if possible to trigger redirect
        if (typeof window !== 'undefined') {
          // Dispatch a custom event to notify auth providers
          window.dispatchEvent(new CustomEvent('auth-token-invalidated'));
        }

        throw new Error('Unauthorized: Invalid or expired token');
      }

      // Handle 403 Forbidden responses
      if (response.status === 403) {
        throw new Error('Forbidden: Insufficient permissions');
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await this.parseResponse(response);
    } catch (error) {
      console.error(`API request failed: ${endpoint}`, error);
      throw error;
    }
  }

  /**
   * Parse response based on content type and status
   */
  async parseResponse(response) {
    if (response.status === 204) {
      // For 204 No Content responses, return null
      return null;
    }

    const contentType = response.headers.get('content-type');

    if (contentType && contentType.includes('application/json')) {
      return await response.json();
    } else {
      // For non-JSON responses, get text
      return await response.text();
    }
  }

  /**
   * Refresh access token using refresh token
   */
  async refreshAccessToken(refreshToken) {
    try {
      const refreshResponse = await fetch(`${this.baseURL}/auth/refresh`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ refresh_token: refreshToken }),
      });

      if (refreshResponse.ok) {
        const data = await refreshResponse.json();

        // Store the new tokens
        if (data.access_token) {
          tokenStorage.setAccessToken(data.access_token);
        }

        if (data.refresh_token) {
          tokenStorage.setRefreshToken(data.refresh_token);
        }

        return true;
      } else {
        // Refresh failed, clear tokens
        tokenStorage.clearTokens();
        return false;
      }
    } catch (error) {
      console.error('Token refresh failed:', error);
      tokenStorage.clearTokens();
      return false;
    }
  }

  /**
   * GET request
   */
  async get(endpoint, params = {}) {
    const queryString = new URLSearchParams(params).toString();
    const url = queryString ? `${endpoint}?${queryString}` : endpoint;

    return this.request(url, {
      method: 'GET',
    });
  }

  /**
   * POST request
   */
  async post(endpoint, data) {
    return this.request(endpoint, {
      method: 'POST',
      body: JSON.stringify(data),
    });
  }

  /**
   * PUT request
   */
  async put(endpoint, data) {
    return this.request(endpoint, {
      method: 'PUT',
      body: JSON.stringify(data),
    });
  }

  /**
   * PATCH request
   */
  async patch(endpoint, data) {
    return this.request(endpoint, {
      method: 'PATCH',
      body: JSON.stringify(data),
    });
  }

  /**
   * DELETE request
   */
  async delete(endpoint) {
    return this.request(endpoint, {
      method: 'DELETE',
    });
  }

  /**
   * Upload file with authentication
   */
  async upload(endpoint, file, fieldName = 'file') {
    // Create form data
    const formData = new FormData();
    formData.append(fieldName, file);

    // Make request without JSON content type
    const url = `${this.baseURL}${endpoint}`;

    const token = tokenStorage.getAccessToken();
    let headers = {};

    if (token && !tokenStorage.isTokenExpired(token)) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    try {
      let response = await fetch(url, {
        method: 'POST',
        headers,
        body: formData,
      });

      if (response.status === 401) {
        const refreshToken = tokenStorage.getRefreshToken();

        // If we have a refresh token, try to get a new access token
        if (refreshToken) {
          const refreshSuccess = await this.refreshAccessToken(refreshToken);

          if (refreshSuccess) {
            // Retry the upload with the new token
            const newToken = tokenStorage.getAccessToken();
            if (newToken) {
              headers = { ...headers, 'Authorization': `Bearer ${newToken}` };

              response = await fetch(url, {
                method: 'POST',
                headers,
                body: formData,
              });

              if (response.ok) {
                const responseData = await response.json();
                return responseData;
              }
            }
          }
        }

        // If refresh failed or no refresh token, clear all tokens and redirect
        tokenStorage.clearTokens();

        // Dispatch a custom event to notify auth providers
        if (typeof window !== 'undefined') {
          window.dispatchEvent(new CustomEvent('auth-token-invalidated'));
        }

        throw new Error('Unauthorized: Invalid or expired token');
      }

      const responseData = await response.json();

      if (!response.ok) {
        throw new Error(responseData.message || `HTTP error! status: ${response.status}`);
      }

      return responseData;
    } catch (error) {
      console.error(`File upload failed: ${endpoint}`, error);
      throw error;
    }
  }

  /**
   * Upload user avatar
   */
  async uploadUserAvatar(userId, avatarFile) {
    return this.upload(`/api/${userId}/avatar`, avatarFile, 'file');
  }

  /**
   * Verify if the current token is valid
   */
  async verifyToken() {
    try {
      const response = await this.get('/auth/verify-token');
      return response;
    } catch (error) {
      console.error('Token verification failed:', error);
      throw error;
    }
  }

  /**
   * Get user tasks
   */
  async getUserTasks(userId) {
    return this.get(`/api/tasks`);
  }

  /**
   * Create a new task for a user
   */
  async createUserTask(userId, taskData) {
    return this.post(`/api/tasks`, taskData);
  }

  /**
   * Update a user task
   */
  async updateUserTask(userId, taskId, taskData) {
    return this.put(`/api/tasks/${taskId}`, taskData);
  }

  /**
   * Delete a user task
   */
  async deleteUserTask(userId, taskId) {
    return this.delete(`/api/tasks/${taskId}`);
  }

  /**
   * Get a specific user task by ID
   */
  async getUserTaskById(userId, taskId) {
    return this.get(`/api/tasks/${taskId}`);
  }

  /**
   * Toggle task completion
   */
  async toggleTaskCompletion(userId, taskId) {
    return this.patch(`/api/tasks/${taskId}/complete`, {});
  }

  /**
   * Sign in user
   */
  async signIn(email, password) {
    const response = await this.post('/auth/login', {
      email,
      password
    });

    // Store the tokens
    if (response.access_token) {
      tokenStorage.setAccessToken(response.access_token);
    }

    if (response.refresh_token) {
      tokenStorage.setRefreshToken(response.refresh_token);
    }

    return response;
  }

  /**
   * Sign up user
   */
  async signUp(email, password, firstName, lastName) {
    const response = await this.post('/auth/register', {
      email,
      password,
      first_name: firstName,
      last_name: lastName
    });

    // Store the tokens if returned
    if (response.access_token) {
      tokenStorage.setAccessToken(response.access_token);
    }

    if (response.refresh_token) {
      tokenStorage.setRefreshToken(response.refresh_token);
    }

    return response;
  }

  /**
   * Sign out user
   */
  async signOut() {
    // Clear tokens
    tokenStorage.clearTokens();

    // Dispatch a custom event to notify auth providers
    if (typeof window !== 'undefined') {
      window.dispatchEvent(new CustomEvent('auth-token-invalidated'));
    }

    return { success: true };
  }

  /**
   * Update user profile
   */
  async updateUserProfile(userId, profileData) {
    const response = await this.put(`/api/${userId}/profile`, profileData);
    return response; // Returns the full response which includes the updated user data
  }

  /**
   * Delete user account
   */
  async deleteUserAccount(userId) {
    const response = await this.delete(`/api/${userId}`);
    // Clear tokens after account deletion
    tokenStorage.clearTokens();
    return response;
  }

  /**
   * Send a chat message to the AI assistant
   * @param {string} userId - Authenticated user's ID
   * @param {Object} request - Chat request with message and optional conversation_id
   * @param {string} request.message - User's message text (1-10,000 characters)
   * @param {number|null} [request.conversation_id] - Optional conversation ID (null for new conversation)
   * @returns {Promise<Object>} Promise resolving to chat response with conversation_id, response, and tool_calls
   */
  async postChat(userId, request) {
    return this.post(`/api/${userId}/chat`, request);
  }
}

export const apiClient = new ApiClient();
export default apiClient;