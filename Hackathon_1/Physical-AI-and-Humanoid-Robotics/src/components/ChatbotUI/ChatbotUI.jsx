import React, { useState, useEffect, useRef } from 'react';
import { getSelectedText, addTextSelectionListener } from './utils/textSelection';
import { sendQuery } from '../../services/api';
import styles from './ChatbotUI.module.css';

/**
 * ChatbotUI Component
 * A minimal React component for interacting with the RAG backend
 */
const ChatbotUI = ({
  backendUrl = (typeof process !== 'undefined' && process.env) ? process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000' : 'http://localhost:8000',
  placeholder = 'Ask a question about the content...',
  showSelectedTextHint = true
}) => {
  // Component state as defined in data-model.md
  const [question, setQuestion] = useState('');
  const [conversations, setConversations] = useState([]); // Store conversation history
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');

  // Effect to set up text selection listener
  useEffect(() => {
    // Function to handle text selection
    const handleTextSelection = (text) => {
      setSelectedText(text);
    };

    // Add the text selection listener
    const removeListener = addTextSelectionListener(handleTextSelection);

    // Cleanup function to remove the listener
    return () => {
      if (removeListener) {
        removeListener();
      }
    };
  }, []);

  // Function to handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    setIsLoading(true);
    setError(null);

    // Add user's question to the conversation history
    const userQuestion = {
      id: Date.now(),
      type: 'user',
      content: question,
      timestamp: new Date().toISOString()
    };

    setConversations(prev => [...prev, userQuestion]);

    try {
      // Send the query to the backend with selected text context
      const result = await sendQuery(question, selectedText, backendUrl);

      // Add the response to the conversation history
      if (result.response) {
        const botResponse = {
          id: Date.now() + 1,
          type: 'bot',
          content: result.response,
          sources: result.sources,
          timestamp: new Date().toISOString()
        };

        setConversations(prev => [...prev, botResponse]);
      } else {
        const errorResponse = {
          id: Date.now() + 1,
          type: 'error',
          content: result.error || 'No response received from backend',
          timestamp: new Date().toISOString()
        };

        setConversations(prev => [...prev, errorResponse]);
      }
    } catch (err) {
      const errorResponse = {
        id: Date.now() + 1,
        type: 'error',
        content: err.message || 'An error occurred while sending the query',
        timestamp: new Date().toISOString()
      };

      setConversations(prev => [...prev, errorResponse]);
      console.error('Query error:', err);
    } finally {
      setQuestion(''); // Clear the question input
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.chatMessages}>
        <div className={styles.welcomeMessage}>
          <p>Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. Ask me anything about robotics, AI, or the book content.</p>
        </div>

        {conversations.length === 0 ? (
          <div className={styles.welcomeMessage}>
            <p>Ask a question about the content, and I'll help you find the answer.</p>
          </div>
        ) : (
          conversations.map((item) => (
            <div
              key={item.id}
              className={`${styles.messageContainer} ${styles[`${item.type}Message`]}`}
            >
              <div className={styles.messageContent}>
                {item.type === 'user' ? (
                  <div className={styles.userQuestion}>
                    {item.content}
                  </div>
                ) : item.type === 'bot' ? (
                  <div className={styles.botResponse}>
                    {item.content}
                    {/* Sources/citations are available but not displayed per user request */}
                  </div>
                ) : (
                  <div className={styles.errorMessage}>
                    {item.content}
                  </div>
                )}
              </div>
            </div>
          ))
        )}

        {error && !conversations.length && (
          <div className={styles.errorMessage}>
            <div className={styles.messageBubbleError}>
              {error}
            </div>
          </div>
        )}

        {isLoading && (
          <div className={styles.botMessage}>
            <div className={styles.messageBubbleBot}>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
      </div>

      <form onSubmit={handleSubmit} className={styles.inputContainer}>
        {showSelectedTextHint && selectedText && (
          <div className={styles.selectedTextHint} aria-label="Selected text for context">
            Selected text: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
          </div>
        )}
        <input
          type="text"
          className={styles.questionInput}
          value={question}
          onChange={(e) => {
            setQuestion(e.target.value);
            setError(null); // Clear error when user types
          }}
          placeholder={placeholder}
          disabled={isLoading}
          onKeyPress={(e) => {
            if (e.key === 'Enter' && !isLoading && question.trim()) {
              handleSubmit(e);
            }
          }}
        />
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading || !question.trim()}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default ChatbotUI;