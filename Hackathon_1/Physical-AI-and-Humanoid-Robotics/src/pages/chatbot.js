import React from 'react';
import Layout from '@theme/Layout';
import ChatbotUI from '@site/src/components/ChatbotUI/ChatbotUI';

export default function ChatbotPage() {
  return (
    <Layout title="AI Robotics Chatbot" description="Interactive chatbot for robotics education">
      <div style={{ padding: '2rem', maxWidth: '1200px', margin: '0 auto' }}>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <h1>AI Robotics Chatbot</h1>
              <p>Get help with robotics concepts, AI integration, and course content from our AI assistant.</p>
              <div style={{
                border: '1px solid #ddd',
                borderRadius: '8px',
                padding: '1rem',
                height: '600px',
                backgroundColor: '#f9f9f9'
              }}>
                <ChatbotUI
                  placeholder="Ask about robotics, AI, or the book content..."
                />
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}