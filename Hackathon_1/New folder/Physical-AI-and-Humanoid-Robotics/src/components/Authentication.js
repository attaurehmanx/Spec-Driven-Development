import React from 'react';
import { SignedIn, SignedOut, SignInButton, UserButton, SignUpButton } from '@clerk/clerk-react';

export default function Authentication() {
  return (
    <div className="auth-component" style={{ display: 'flex', alignItems: 'center', gap: '0.25rem', height: '100%' }}>
      <SignedOut>
        <SignInButton mode="modal">
          <button className="button button--secondary button--sm" style={{ margin: 0, padding: '0.25rem 0.5rem', fontSize: '0.8rem' }}>Sign In</button>
        </SignInButton>
        <SignUpButton mode="modal">
          <button className="button button--primary button--sm" style={{ margin: 0, padding: '0.25rem 0.5rem', fontSize: '0.8rem' }}>Sign Up</button>
        </SignUpButton>
      </SignedOut>
      <SignedIn>
        <div style={{ display: 'flex', alignItems: 'center', height: '100%' }}>
          <UserButton
            afterSignOutUrl="/"
            appearance={{
              elements: {
                avatarBox: "w-8 h-8",
              }
            }}
          />
        </div>
      </SignedIn>
    </div>
  );
}