import React, { createContext, useContext, useReducer } from 'react';

// Create the context
const ChatbotContext = createContext();

// Reducer for chatbot state management
const chatbotReducer = (state, action) => {
  switch (action.type) {
    case 'SET_MESSAGES':
      return {
        ...state,
        messages: action.payload,
      };
    case 'ADD_MESSAGE':
      return {
        ...state,
        messages: [...state.messages, action.payload],
      };
    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload,
      };
    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
      };
    case 'CLEAR_HISTORY':
      return {
        ...state,
        messages: [],
        error: null,
      };
    default:
      return state;
  }
};

// Initial state
const initialState = {
  messages: [],
  isLoading: false,
  error: null,
};

// Provider component
export const ChatbotProvider = ({ children }) => {
  const [state, dispatch] = useReducer(chatbotReducer, initialState);

  // Actions
  const setMessages = (messages) => {
    dispatch({ type: 'SET_MESSAGES', payload: messages });
  };

  const addMessage = (message) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const setLoading = (isLoading) => {
    dispatch({ type: 'SET_LOADING', payload: isLoading });
  };

  const setError = (error) => {
    dispatch({ type: 'SET_ERROR', payload: error });
  };

  const clearHistory = () => {
    dispatch({ type: 'CLEAR_HISTORY' });
  };

  const value = {
    ...state,
    setMessages,
    addMessage,
    setLoading,
    setError,
    clearHistory,
  };

  return (
    <ChatbotContext.Provider value={value}>
      {children}
    </ChatbotContext.Provider>
  );
};

// Custom hook to use the chatbot context
export const useChatbotContext = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbotContext must be used within a ChatbotProvider');
  }
  return context;
};

export default ChatbotContext;