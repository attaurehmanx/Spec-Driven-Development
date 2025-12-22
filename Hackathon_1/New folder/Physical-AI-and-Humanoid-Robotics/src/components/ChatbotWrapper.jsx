/**
 * ChatbotWrapper Component
 * Docusaurus-compatible wrapper for the ChatbotUI component
 */

import React from 'react';
import ChatbotUI from '@site/src/components/ChatbotUI/ChatbotUI';

/**
 * Wrapper component that makes the ChatbotUI compatible with Docusaurus
 * This component can be imported and used in Docusaurus MDX files
 */
const ChatbotWrapper = (props) => {
  return (
    <div className="chatbot-wrapper">
      <ChatbotUI {...props} />
    </div>
  );
};

export default ChatbotWrapper;