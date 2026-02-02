'use client';

import React from 'react';

/**
 * Typing indicator component
 * Shows animated dots while AI is generating a response
 */
export function ChatLoading() {
  return (
    <div className="flex w-full mb-4 justify-start">
      <div className="max-w-[80%] rounded-lg px-4 py-3 bg-gray-100 border border-gray-200">
        <div className="flex items-center gap-1">
          <div className="flex gap-1">
            <span className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0ms' }}></span>
            <span className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '150ms' }}></span>
            <span className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '300ms' }}></span>
          </div>
          <span className="ml-2 text-xs text-gray-500">AI is thinking...</span>
        </div>
      </div>
    </div>
  );
}
