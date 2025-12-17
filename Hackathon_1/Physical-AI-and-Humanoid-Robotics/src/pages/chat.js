import React from 'react';
import Layout from '@theme/Layout';
import ChatbotUI from '../components/ChatbotUI/ChatbotUI';

function ChatPage() {
  return (
    <Layout title="AI Assistant" description="Chat with our AI assistant about the robotics content">
      <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
        <h1>AI Assistant</h1>
        <p>Ask questions about the robotics content, and our AI assistant will help you find answers.</p>
        <ChatbotUI />
      </div>
    </Layout>
  );
}

export default ChatPage;