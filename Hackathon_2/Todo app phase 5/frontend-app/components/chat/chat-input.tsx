'use client';

import React, { useState, KeyboardEvent, useRef, useEffect } from 'react';
import { Button } from '@/components/ui/button';
import { Send } from 'lucide-react';

interface ChatInputProps {
  onSendMessage: (content: string) => void;
  disabled: boolean;
  placeholder?: string;
}

/**
 * Message input component with validation and submit handling
 */
export function ChatInput({
  onSendMessage,
  disabled,
  placeholder = 'Type your message...'
}: ChatInputProps) {
  const [message, setMessage] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea based on content
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
    }
  }, [message]);

  const handleSubmit = () => {
    const trimmedMessage = message.trim();

    // Validate message
    if (trimmedMessage.length === 0) {
      return;
    }

    if (trimmedMessage.length > 10000) {
      alert('Message is too long (max 10,000 characters)');
      return;
    }

    // Send message
    onSendMessage(trimmedMessage);

    // Clear input
    setMessage('');

    // Reset textarea height
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    // Submit on Enter (without Shift)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className="border-t border-gray-200 dark:border-gray-700 bg-white dark:bg-slate-900 p-3">
      <div className="flex gap-2 items-end">
        <div className="flex-1 relative">
          <textarea
            ref={textareaRef}
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder={placeholder}
            disabled={disabled}
            rows={1}
            className="w-full resize-none rounded-lg border border-gray-300 dark:border-gray-600 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 px-3 py-2 pr-12 text-sm focus:outline-none focus:ring-2 focus:ring-primary-500 focus:border-transparent disabled:bg-gray-100 dark:disabled:bg-gray-800 disabled:cursor-not-allowed max-h-32 overflow-y-auto"
            style={{ minHeight: '40px' }}
            aria-label="Message input"
            aria-describedby="chat-input-help"
          />

          {/* Character count (show when approaching limit) */}
          {message.length > 9000 && (
            <div className="absolute bottom-2 right-2 text-xs text-gray-400">
              {message.length}/10,000
            </div>
          )}
        </div>

        <Button
          onClick={handleSubmit}
          disabled={disabled || message.trim().length === 0}
          size="default"
          className="h-10 px-3"
          title="Send message (Enter)"
        >
          <Send className="h-4 w-4" />
        </Button>
      </div>

      {/* Helper text */}
      <div id="chat-input-help" className="mt-1.5 text-xs text-gray-500">
        Press Enter to send, Shift+Enter for new line
      </div>
    </div>
  );
}
