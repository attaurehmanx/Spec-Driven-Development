import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatbot from '@site/src/components/FloatingChatbot/FloatingChatbot';
import { ClerkProvider } from '@clerk/clerk-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { AuthGuard } from '@site/src/components/AuthGuard';

export default function Layout(props) {
  const { siteConfig } = useDocusaurusContext();
  const publishableKey = siteConfig.customFields?.clerkPublishableKey;

  // Better error handling
  if (!publishableKey) {
    console.error('Clerk publishable key is missing. Please set CLERK_PUBLISHABLE_KEY in your environment variables.');
    return (
      <OriginalLayout {...props}>
        {props.children}
        <FloatingChatbot />
      </OriginalLayout>
    );
  }

  return (
    <ClerkProvider
      publishableKey={publishableKey}
      afterSignOutUrl="/"
      signInUrl="/sign-in"
      signUpUrl="/sign-up"
      signInFallbackRedirectUrl={typeof window !== 'undefined' ? window.location.pathname : '/'}
      signUpFallbackRedirectUrl={typeof window !== 'undefined' ? window.location.pathname : '/'}
      appearance={{
        variables: {
          // Customize the appearance to match your site
        }
      }}
      onError={(error) => {
        console.error('Clerk error:', error);
      }}>
      <AuthGuard publishableKey={publishableKey}>
        <OriginalLayout {...props}>
          {props.children}
          <FloatingChatbot />
        </OriginalLayout>
      </AuthGuard>
    </ClerkProvider>
  );
}