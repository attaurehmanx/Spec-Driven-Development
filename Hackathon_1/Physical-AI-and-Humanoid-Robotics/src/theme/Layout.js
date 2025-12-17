import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatbot from '@site/src/components/FloatingChatbot/FloatingChatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <FloatingChatbot />
      </OriginalLayout>
    </>
  );
}