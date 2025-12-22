import React, { useState } from 'react';
import ChatbotUI from '../ChatbotUI/ChatbotUI';
import styles from './FloatingChatbot.module.css';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Open chat button - always visible when chat is closed */}
      {!isOpen && (
        <button className={styles.fab} onClick={toggleChat} aria-label="Open chat">
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM9 17L8 16L8.5 15C8.17 14.25 8 13.39 8 12.5C8 10.57 8.93 8.82 10.4 7.6C10.18 8.18 10.07 8.8 10.07 9.44C10.07 10.96 10.69 12.35 11.67 13.33C11.81 13.19 11.93 13.04 12 12.87C12.67 13.54 13.56 14 14.5 14C15.43 14 16.33 13.54 16.99 12.87C17.06 13.04 17.18 13.19 17.32 13.33C18.3 12.35 18.92 10.96 18.92 9.44C18.92 8.8 18.81 8.18 18.59 7.6C20.06 8.82 20.99 10.57 20.99 12.5C20.99 17.52 17.51 20 12.99 20C12.1 20 11.23 19.85 10.42 19.58L9 21V17ZM12.5 7H11.5V9H12.5V7ZM15.5 7H14.5V9H15.5V7Z"
              fill="white"
            />
          </svg>
        </button>
      )}

      {/* Overlay that appears when chat is open */}
      {isOpen && (
        <div className={styles.overlay} onClick={toggleChat} />
      )}

      {/* Chat panel - only shown when open */}
      <div className={`${styles.chatContainer} ${isOpen ? styles.open : ''}`}>
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h2 className={styles.chatTitle}>Robotics AI Assistant</h2>
            <button className={styles.closeButton} onClick={toggleChat} aria-label="Close chat">
              ×
            </button>
          </div>
          <div className={styles.chatContent}>
            <ChatbotUI
              placeholder="Ask about robotics, AI, or the book content..."
            />
          </div>
        </div>
      </div>
    </>
  );
};

export default FloatingChatbot;