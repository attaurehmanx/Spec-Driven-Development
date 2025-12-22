import React from 'react';
import { SignUp } from '@clerk/clerk-react';
import Layout from '@theme/Layout';

export default function SignUpPage() {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '70vh',
        padding: '2rem'
      }}>
        <SignUp
          appearance={{
            elements: {
              card: 'shadow-lg',
              headerTitle: 'text-xl font-bold',
              headerSubtitle: 'text-sm',
              socialButtonsBlockButton: 'border border-gray-300 hover:bg-gray-50',
              formButtonPrimary: 'bg-blue-600 hover:bg-blue-700',
              footerActionLink: 'text-blue-600 hover:text-blue-700'
            }
          }}
        />
      </div>
    </Layout>
  );
}