'use client';

import React from 'react';
import { ChatMessage as ChatMessageType } from '@/types';
import { cn } from '@/lib/utils';
import { CheckCircle2, AlertCircle, Loader2 } from 'lucide-react';

interface ChatMessageProps {
  message: ChatMessageType;
  isLatest?: boolean;
}

/**
 * Individual message bubble component
 * Displays user or assistant messages with role-based styling
 * Memoized for performance with long conversations
 */
const ChatMessageComponent = ({ message, isLatest }: ChatMessageProps) => {
  const isUser = message.role === 'user';
  const isAssistant = message.role === 'assistant';

  // Format timestamp
  const formatTime = (date: Date) => {
    return new Date(date).toLocaleTimeString('en-US', {
      hour: 'numeric',
      minute: '2-digit',
      hour12: true
    });
  };

  return (
    <div
      className={cn(
        'flex w-full mb-4',
        isUser ? 'justify-end' : 'justify-start'
      )}
      role="article"
      aria-label={`${isUser ? 'Your' : 'Assistant'} message`}
    >
      <div
        className={cn(
          'max-w-[80%] rounded-lg px-4 py-3 shadow-sm',
          isUser
            ? 'bg-primary-500 text-white'
            : 'bg-gray-100 text-gray-900 border border-gray-200'
        )}
      >
        {/* Message content */}
        <div className="text-sm whitespace-pre-wrap break-words">
          {message.content}
        </div>

        {/* Timestamp and status */}
        <div
          className={cn(
            'flex items-center gap-2 mt-2 text-xs',
            isUser ? 'text-primary-50' : 'text-gray-500'
          )}
        >
          <span>{formatTime(message.timestamp)}</span>

          {/* Status indicator for user messages */}
          {isUser && message.status && (
            <>
              {message.status === 'sending' && (
                <Loader2 className="h-3 w-3 animate-spin" />
              )}
              {message.status === 'sent' && (
                <CheckCircle2 className="h-3 w-3" />
              )}
              {message.status === 'error' && (
                <AlertCircle className="h-3 w-3 text-red-300" />
              )}
            </>
          )}
        </div>

        {/* Error message if present */}
        {message.status === 'error' && message.errorMessage && (
          <div className="mt-2 text-xs text-red-300 flex items-start gap-1">
            <AlertCircle className="h-3 w-3 mt-0.5 flex-shrink-0" />
            <span>{message.errorMessage}</span>
          </div>
        )}
      </div>
    </div>
  );
};

// Memoize component to prevent unnecessary re-renders in long conversations
export const ChatMessage = React.memo(ChatMessageComponent, (prevProps, nextProps) => {
  // Only re-render if message content or status changes
  return (
    prevProps.message.id === nextProps.message.id &&
    prevProps.message.content === nextProps.message.content &&
    prevProps.message.status === nextProps.message.status &&
    prevProps.isLatest === nextProps.isLatest
  );
});
