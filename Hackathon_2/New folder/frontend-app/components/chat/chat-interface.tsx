'use client';

import React, { useState, useEffect } from 'react';
import { cn } from '@/lib/utils';
import useChat from '@/hooks/use-chat';
import { ChatHeader } from './chat-header';
import { ChatMessageList } from './chat-message-list';
import { ChatInput } from './chat-input';
import { AlertCircle, CheckCircle2 } from 'lucide-react';

interface ChatInterfaceProps {
  isOpen: boolean;
  onClose: () => void;
  onTasksModified?: () => void;
  className?: string;
}

/**
 * Main chat interface component
 * Integrates all chat subcomponents and manages chat state
 */
export function ChatInterface({
  isOpen,
  onClose,
  onTasksModified,
  className
}: ChatInterfaceProps) {
  const {
    conversationId,
    messages,
    isLoading,
    error,
    sendMessage,
    startNewConversation,
    clearError
  } = useChat();

  const [showTaskUpdateNotification, setShowTaskUpdateNotification] = useState(false);

  // Listen for task updates to show notification
  useEffect(() => {
    const handleTasksUpdated = () => {
      setShowTaskUpdateNotification(true);
      // Hide notification after 3 seconds
      setTimeout(() => {
        setShowTaskUpdateNotification(false);
      }, 3000);
    };

    window.addEventListener('tasks-updated', handleTasksUpdated);
    return () => {
      window.removeEventListener('tasks-updated', handleTasksUpdated);
    };
  }, []);

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={cn(
        'flex flex-col h-full bg-white shadow-lg',
        className
      )}
      role="dialog"
      aria-label="AI Assistant Chat"
      aria-modal="true"
    >
      {/* Header */}
      <ChatHeader
        onClose={onClose}
        onNewConversation={startNewConversation}
        conversationId={conversationId}
      />

      {/* Error banner */}
      {error && (
        <div className="bg-red-50 border-b border-red-200 px-4 py-3">
          <div className="flex items-start gap-2">
            <AlertCircle className="h-5 w-5 text-red-600 flex-shrink-0 mt-0.5" />
            <div className="flex-1">
              <p className="text-sm text-red-800">{error}</p>
            </div>
            <button
              onClick={clearError}
              className="text-red-600 hover:text-red-800 text-sm font-medium"
            >
              Dismiss
            </button>
          </div>
        </div>
      )}

      {/* Task update notification */}
      {showTaskUpdateNotification && (
        <div className="bg-green-50 border-b border-green-200 px-4 py-3">
          <div className="flex items-center gap-2">
            <CheckCircle2 className="h-5 w-5 text-green-600 flex-shrink-0" />
            <p className="text-sm text-green-800">Task list updated</p>
          </div>
        </div>
      )}

      {/* Message list */}
      <ChatMessageList messages={messages} isLoading={isLoading} />

      {/* Input */}
      <ChatInput
        onSendMessage={sendMessage}
        disabled={isLoading}
        placeholder="Ask me to help with your tasks..."
      />
    </div>
  );
}
