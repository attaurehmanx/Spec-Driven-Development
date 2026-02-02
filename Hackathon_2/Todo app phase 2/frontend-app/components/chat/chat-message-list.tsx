'use client';

import React, { useEffect, useRef } from 'react';
import { ChatMessage as ChatMessageType } from '@/types';
import { ChatMessage } from './chat-message';
import { ChatLoading } from './chat-loading';

interface ChatMessageListProps {
  messages: ChatMessageType[];
  isLoading: boolean;
}

/**
 * Message history display component
 * Shows all messages in chronological order with auto-scroll
 */
export function ChatMessageList({ messages, isLoading }: ChatMessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading]);

  // Empty state
  if (messages.length === 0 && !isLoading) {
    return (
      <div className="flex-1 flex items-center justify-center p-8">
        <div className="text-center max-w-md">
          <div className="mb-4 text-gray-400">
            <svg
              className="mx-auto h-12 w-12"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={1.5}
                d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z"
              />
            </svg>
          </div>
          <h3 className="text-lg font-medium text-gray-900 mb-2">
            Start a conversation
          </h3>
          <p className="text-sm text-gray-500">
            Ask your AI assistant to help manage your tasks. Try asking "What tasks do I have?" or "Create a task to buy groceries".
          </p>
        </div>
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className="flex-1 overflow-y-auto p-4 space-y-2"
      role="log"
      aria-label="Chat message history"
      aria-live="polite"
    >
      {messages.map((message, index) => (
        <ChatMessage
          key={message.id}
          message={message}
          isLatest={index === messages.length - 1}
        />
      ))}

      {/* Loading indicator */}
      {isLoading && <ChatLoading />}

      {/* Scroll anchor */}
      <div ref={messagesEndRef} />
    </div>
  );
}
