import React from 'react';
import { ChatbotProvider } from './ChatbotUI/ChatbotContext';

// Root component to wrap the entire Docusaurus app with Chatbot context
export default function Root({ children }) {
  return (
    <ChatbotProvider>
      {children}
    </ChatbotProvider>
  );
}