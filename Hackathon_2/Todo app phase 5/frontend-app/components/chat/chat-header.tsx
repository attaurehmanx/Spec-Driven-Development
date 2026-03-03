'use client';

import React from 'react';
import { Button } from '@/components/ui/button';
import { X, MessageSquarePlus } from 'lucide-react';

interface ChatHeaderProps {
  onClose: () => void;
  onNewConversation?: () => void;
  conversationId: number | null;
}

/**
 * Chat header component with title, controls, and conversation status
 */
export function ChatHeader({ onClose, onNewConversation, conversationId }: ChatHeaderProps) {
  return (
    <div className="flex items-center justify-between px-4 py-2.5 border-b border-gray-200 dark:border-gray-700 bg-white dark:bg-slate-900">
      <div className="flex items-center gap-2.5">
        <h2 className="text-base font-semibold text-gray-900 dark:text-gray-100">AI Assistant</h2>
        {conversationId !== null && (
          <span className="text-xs text-gray-500 dark:text-gray-400 bg-gray-100 dark:bg-slate-800 px-2 py-1 rounded">
            Conversation #{conversationId}
          </span>
        )}
      </div>

      <div className="flex items-center gap-2">
        {/* New Conversation button */}
        {onNewConversation && (
          <Button
            variant="ghost"
            size="icon"
            onClick={onNewConversation}
            title="Start new conversation"
            className="h-8 w-8 text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-slate-800"
          >
            <MessageSquarePlus className="h-4 w-4" />
          </Button>
        )}

        {/* Close button */}
        <Button
          variant="ghost"
          size="icon"
          onClick={onClose}
          title="Close chat"
          className="h-8 w-8 text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-slate-800"
        >
          <X className="h-4 w-4" />
        </Button>
      </div>
    </div>
  );
}
