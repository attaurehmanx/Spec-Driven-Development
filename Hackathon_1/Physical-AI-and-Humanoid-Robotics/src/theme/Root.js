import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot/FloatingChatbot';

const Root = ({children}) => {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
};

export default Root;