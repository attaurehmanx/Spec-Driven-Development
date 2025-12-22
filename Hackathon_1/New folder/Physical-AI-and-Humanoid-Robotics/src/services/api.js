/**
 * API Service for RAG Backend Communication
 * Handles communication with the FastAPI RAG backend
 */

/**
 * Send a query to the RAG backend
 * @param {string} question - The user's question
 * @param {string} selectedText - Optional selected text from the current page for context
 * @param {string} backendUrl - The URL of the backend API
 * @returns {Promise<Object>} The response from the backend
 */
export const sendQuery = async (question, selectedText = '', backendUrl = (typeof process !== 'undefined' && process.env) ? process.env.REACT_APP_BACKEND_URL || 'https://attaurehman-chatbot.hf.space' : 'https://attaurehman-chatbot.hf.space') => {
  try {
    // Construct the request body based on backend API contract
    const requestBody = {
      query: question.trim()  // Backend expects 'query' not 'question'
    };

    // Include selected text if provided
    if (selectedText && selectedText.trim()) {
      requestBody.selected_text = selectedText.trim();  // Backend expects snake_case 'selected_text'
    }

    // Make the API call to the backend
    // Use the backend URL directly - for production deployment
    const apiUrl = `${backendUrl}/v1/query`;
    const response = await fetch(apiUrl, {  // Backend uses '/v1/query' endpoint
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    // Check if the response is successful
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.message || errorData.error || `HTTP error! status: ${response.status}`);
    }

    // Parse and return the JSON response
    const data = await response.json();

    // Transform backend response to match frontend expectations
    // Backend returns: {response_text, citations, query_id, timestamp, ...}
    // Frontend expects: {response, sources, ...}
    return {
      response: data.response_text || data.answer,  // Map 'response_text' or 'answer' to 'response'
      sources: data.citations?.map(citation => ({
        title: citation.title || citation.document_title,
        url: citation.url || citation.document_url,
        content: citation.text || citation.text_snippet
      })),  // Map 'citations' to 'sources'
      queryId: data.query_id,
      timestamp: data.timestamp,
      ...data  // Include other fields as needed
    };
  } catch (error) {
    // Handle network errors and other exceptions
    console.error('Error sending query to backend:', error);
    throw error;
  }
};

/**
 * Check if the backend is healthy
 * @param {string} backendUrl - The URL of the backend API
 * @returns {Promise<boolean>} True if backend is healthy, false otherwise
 */
export const checkHealth = async (backendUrl = (typeof process !== 'undefined' && process.env) ? process.env.REACT_APP_BACKEND_URL || 'https://attaurehman-chatbot.hf.space' : 'https://attaurehman-chatbot.hf.space') => {
  try {
    // Use the backend URL directly - for production deployment
    const healthUrl = `${backendUrl}/health`;
    const response = await fetch(healthUrl);  // Use backend's health endpoint
    return response.ok;
  } catch (error) {
    console.error('Error checking backend health:', error);
    return false;
  }
};