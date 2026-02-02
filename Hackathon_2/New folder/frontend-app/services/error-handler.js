/**
 * Error Handler for API Responses
 * Centralized error handling for different HTTP status codes
 */

class ErrorHandler {
  /**
   * Handle API errors based on status code
   */
  static handle(error, context = '') {
    console.error(`API Error in ${context}:`, error);

    // If it's an HTTP error with a status code
    if (error.status) {
      switch (error.status) {
        case 400:
          return this.handleBadRequest(error);
        case 401:
          return this.handleUnauthorized(error);
        case 403:
          return this.handleForbidden(error);
        case 404:
          return this.handleNotFound(error);
        case 422:
          return this.handleValidation(error);
        case 500:
          return this.handleServerError(error);
        default:
          return this.handleGenericError(error);
      }
    }

    // If it's a network error or other non-HTTP error
    return this.handleNetworkError(error);
  }

  /**
   * Handle 400 Bad Request errors
   */
  static handleBadRequest(error) {
    return {
      type: 'validation_error',
      message: error.message || 'Bad request - please check your input',
      status: 400,
      details: error.details || null
    };
  }

  /**
   * Handle 401 Unauthorized errors
   */
  static handleUnauthorized(error) {
    // Clear any stored tokens as they may be invalid
    if (typeof window !== 'undefined' && window.localStorage) {
      localStorage.removeItem('better_auth_token');
      localStorage.removeItem('better_refresh_token');
    }

    // Optionally redirect to login page
    // if (typeof window !== 'undefined') {
    //   window.location.href = '/login';
    // }

    return {
      type: 'unauthorized',
      message: error.message || 'Unauthorized access - please log in',
      status: 401,
      redirect: '/login'
    };
  }

  /**
   * Handle 403 Forbidden errors
   */
  static handleForbidden(error) {
    // Check if this is a user ID mismatch error
    const errorMessage = error.message || '';
    const isUserIdMismatch = errorMessage.toLowerCase().includes('user id') && errorMessage.toLowerCase().includes('match');

    return {
      type: isUserIdMismatch ? 'user_id_mismatch' : 'forbidden',
      message: error.message || (isUserIdMismatch
        ? 'User ID in token does not match URL parameter - access denied'
        : 'Access forbidden - insufficient permissions'),
      status: 403,
      ...(isUserIdMismatch && { reason: 'user_id_mismatch' })
    };
  }

  /**
   * Handle 404 Not Found errors
   */
  static handleNotFound(error) {
    return {
      type: 'not_found',
      message: error.message || 'Resource not found',
      status: 404
    };
  }

  /**
   * Handle 422 Validation errors
   */
  static handleValidation(error) {
    return {
      type: 'validation_error',
      message: error.message || 'Validation failed',
      status: 422,
      details: error.details || error.validation_errors || null
    };
  }

  /**
   * Handle 500 Server errors
   */
  static handleServerError(error) {
    return {
      type: 'server_error',
      message: error.message || 'Server error occurred',
      status: 500
    };
  }

  /**
   * Handle network errors (no connection, timeout, etc.)
   */
  static handleNetworkError(error) {
    return {
      type: 'network_error',
      message: error.message || 'Network error - please check your connection',
      status: null
    };
  }

  /**
   * Handle generic errors
   */
  static handleGenericError(error) {
    return {
      type: 'generic_error',
      message: error.message || 'An unexpected error occurred',
      status: error.status || null
    };
  }

  /**
   * Log error to external service (like Sentry, etc.)
   */
  static logError(error, context = '', additionalData = {}) {
    // In a real application, you would send this to an error tracking service
    console.group('Error Log');
    console.error('Context:', context);
    console.error('Error:', error);
    console.error('Additional Data:', additionalData);
    console.groupEnd();
  }

  /**
   * Format error for display to user
   */
  static formatUserMessage(errorInfo) {
    switch (errorInfo.type) {
      case 'validation_error':
        return 'Please check your input and try again.';
      case 'unauthorized':
        return 'Your session has expired. Please log in again.';
      case 'forbidden':
        return 'You do not have permission to perform this action.';
      case 'not_found':
        return 'The requested resource was not found.';
      case 'server_error':
        return 'A server error occurred. Please try again later.';
      case 'network_error':
        return 'Please check your internet connection and try again.';
      default:
        return errorInfo.message || 'An error occurred. Please try again.';
    }
  }
}

// Export a convenience function for handling errors
const handleApiError = (error, context = '') => {
  const errorInfo = ErrorHandler.handle(error, context);
  ErrorHandler.logError(error, context);
  return errorInfo;
};

export { ErrorHandler, handleApiError };
export default ErrorHandler;