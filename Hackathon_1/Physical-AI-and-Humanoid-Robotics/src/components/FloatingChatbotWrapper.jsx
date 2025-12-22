/**
 * FloatingChatbotWrapper Component
 * Docusaurus-compatible wrapper for the FloatingChatbot component
 * This component can be imported and used in Docusaurus MDX files
 */

import React from 'react';
import FloatingChatbot from '@site/src/components/FloatingChatbot/FloatingChatbot';

/**
 * Wrapper component that makes the FloatingChatbot compatible with Docusaurus
 * This component can be imported and used in Docusaurus MDX files
 */
const FloatingChatbotWrapper = (props) => {
  return (
    <div className="floating-chatbot-wrapper">
      <FloatingChatbot {...props} />
    </div>
  );
};

export default FloatingChatbotWrapper;