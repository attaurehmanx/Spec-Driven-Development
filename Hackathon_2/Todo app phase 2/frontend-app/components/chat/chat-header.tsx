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
    <div className="flex items-center justify-between px-4 py-3 border-b border-gray-200 bg-white">
      <div className="flex items-center gap-3">
        <h2 className="text-lg font-semibold text-gray-900">AI Assistant</h2>
        {conversationId !== null && (
          <span className="text-xs text-gray-500 bg-gray-100 px-2 py-1 rounded">
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
            className="h-8 w-8"
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
          className="h-8 w-8"
        >
          <X className="h-4 w-4" />
        </Button>
      </div>
    </div>
  );
}
