'use client';

import { useState, useCallback, useEffect } from 'react';
import { apiClient } from '@/services/api-client';
import { ChatMessage, UseChatReturn } from '@/types';
import useAuth from './use-auth';

const CONVERSATION_STORAGE_KEY = 'chat_conversation_id';

/**
 * Load conversation ID from localStorage
 */
const loadConversationId = (): number | null => {
  if (typeof window === 'undefined') return null;

  try {
    const stored = localStorage.getItem(CONVERSATION_STORAGE_KEY);
    if (!stored) return null;

    const parsed = JSON.parse(stored);

    // Optional: Expire after 24 hours
    if (parsed.timestamp) {
      const hoursSinceUpdate = (Date.now() - parsed.timestamp) / (1000 * 60 * 60);
      if (hoursSinceUpdate > 24) {
        localStorage.removeItem(CONVERSATION_STORAGE_KEY);
        return null;
      }
    }

    return parsed.conversationId;
  } catch (error) {
    console.error('Failed to load conversation ID:', error);
    return null;
  }
};

/**
 * Save conversation ID to localStorage
 */
const saveConversationId = (conversationId: number | null): void => {
  if (typeof window === 'undefined') return;

  try {
    if (conversationId === null) {
      localStorage.removeItem(CONVERSATION_STORAGE_KEY);
    } else {
      localStorage.setItem(
        CONVERSATION_STORAGE_KEY,
        JSON.stringify({
          conversationId,
          timestamp: Date.now()
        })
      );
    }
  } catch (error) {
    console.error('Failed to save conversation ID:', error);
  }
};

/**
 * Custom hook for managing chat state and interactions
 * Handles conversation state, message history, and API communication
 */
const useChat = (): UseChatReturn => {
  const { user } = useAuth();
  const [conversationId, setConversationId] = useState<number | null>(null);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  // Load conversation ID from localStorage on mount
  useEffect(() => {
    const storedId = loadConversationId();
    if (storedId !== null) {
      setConversationId(storedId);
    }
  }, []);

  // Save conversation ID to localStorage when it changes
  useEffect(() => {
    saveConversationId(conversationId);
  }, [conversationId]);

  /**
   * Send a message to the AI assistant
   */
  const sendMessage = useCallback(async (content: string) => {
    if (!user) {
      setError('You must be signed in to send messages');
      return;
    }

    // Validate message content
    const trimmedContent = content.trim();
    if (trimmedContent.length === 0) {
      setError('Message cannot be empty');
      return;
    }

    if (trimmedContent.length > 10000) {
      setError('Message is too long (max 10,000 characters)');
      return;
    }

    try {
      setIsLoading(true);
      setError(null);

      // Create user message with optimistic update
      const userMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'user',
        content: trimmedContent,
        timestamp: new Date(),
        status: 'sending'
      };

      // Add user message to UI immediately
      setMessages(prev => [...prev, userMessage]);

      // Send to backend
      const response = await apiClient.postChat(user.id, {
        message: trimmedContent,
        conversation_id: conversationId
      });

      // Update conversation ID if this was a new conversation
      if (conversationId === null && response.conversation_id) {
        setConversationId(response.conversation_id);
      }

      // Update user message status to 'sent'
      setMessages(prev =>
        prev.map(msg =>
          msg.id === userMessage.id
            ? { ...msg, status: 'sent' }
            : msg
        )
      );

      // Add AI response
      const aiMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: response.response,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, aiMessage]);

      // Trigger task refresh if AI performed data modifications
      if (response.tool_calls && response.tool_calls.some(tool =>
        ['add_task', 'update_task', 'delete_task', 'complete_task'].includes(tool)
      )) {
        window.dispatchEvent(new CustomEvent('tasks-updated'));
      }

    } catch (err: any) {
      console.error('Failed to send message:', err);

      // Determine error message based on error type
      let errorMessage = 'Failed to send message. Please try again.';

      if (err.message?.includes('Unauthorized')) {
        errorMessage = 'Your session has expired. Please sign in again.';
      } else if (err.message?.includes('Forbidden')) {
        errorMessage = 'Access denied. Please sign in again.';
      } else if (err.message?.includes('503')) {
        errorMessage = 'AI assistant is temporarily unavailable. Please try again in a moment.';
      } else if (err.message?.includes('500')) {
        errorMessage = 'Server error. Please try again later.';
      } else if (err.message?.includes('Network')) {
        errorMessage = 'Unable to connect to the server. Please check your internet connection.';
      }

      setError(errorMessage);

      // Update user message status to 'error'
      setMessages(prev =>
        prev.map(msg =>
          msg.status === 'sending'
            ? { ...msg, status: 'error', errorMessage }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  }, [user, conversationId]);

  /**
   * Start a new conversation
   * Clears messages and resets conversation_id
   */
  const startNewConversation = useCallback(() => {
    setConversationId(null);
    setMessages([]);
    setError(null);
  }, []);

  /**
   * Clear error message
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    conversationId,
    messages,
    isLoading,
    error,
    sendMessage,
    startNewConversation,
    clearError
  };
};

export default useChat;
