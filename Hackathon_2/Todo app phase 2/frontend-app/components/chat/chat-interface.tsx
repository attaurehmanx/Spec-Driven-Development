'use client';

import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
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
    <motion.div
      initial={{ opacity: 0, x: 20 }}
      animate={{ opacity: 1, x: 0 }}
      exit={{ opacity: 0, x: 20 }}
      transition={{ duration: 0.3, ease: 'easeOut' }}
      className={cn(
        'flex flex-col h-full bg-background/95 dark:bg-background/90 backdrop-blur-sm shadow-2xl border-l border-border',
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
      <AnimatePresence>
        {error && (
          <motion.div
            initial={{ opacity: 0, height: 0 }}
            animate={{ opacity: 1, height: 'auto' }}
            exit={{ opacity: 0, height: 0 }}
            className="bg-destructive/10 dark:bg-destructive/20 border-b border-destructive/20 px-4 py-3"
          >
            <div className="flex items-start gap-2">
              <AlertCircle className="h-5 w-5 text-destructive flex-shrink-0 mt-0.5" />
              <div className="flex-1">
                <p className="text-sm text-destructive">{error}</p>
              </div>
              <button
                onClick={clearError}
                className="text-destructive hover:text-destructive/80 text-sm font-medium transition-colors"
              >
                Dismiss
              </button>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Task update notification */}
      <AnimatePresence>
        {showTaskUpdateNotification && (
          <motion.div
            initial={{ opacity: 0, height: 0 }}
            animate={{ opacity: 1, height: 'auto' }}
            exit={{ opacity: 0, height: 0 }}
            className="bg-green-500/10 dark:bg-green-500/20 border-b border-green-500/20 px-4 py-3"
          >
            <div className="flex items-center gap-2">
              <CheckCircle2 className="h-5 w-5 text-green-600 dark:text-green-400 flex-shrink-0" />
              <p className="text-sm text-green-700 dark:text-green-300">Task list updated</p>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Message list */}
      <ChatMessageList messages={messages} isLoading={isLoading} />

      {/* Input */}
      <ChatInput
        onSendMessage={sendMessage}
        disabled={isLoading}
        placeholder="Ask me to help with your tasks..."
      />
    </motion.div>
  );
}
