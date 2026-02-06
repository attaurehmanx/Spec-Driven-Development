import './globals.css';
import type { Metadata } from 'next';
import { Inter } from 'next/font/google';
import { AuthProvider } from '../components/providers/auth-provider';

const inter = Inter({ subsets: ['latin'] });

export const metadata: Metadata = {
  title: 'Task Management App',
  description: 'A secure task management application with authentication',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body className={`${inter.className} flex flex-col min-h-screen`}>
        <AuthProvider>
          <div className="flex-grow">
            {children}
          </div>
          <div className="py-4 text-center text-sm text-gray-500 border-t border-gray-200 mt-auto">
            <p>© {new Date().getFullYear()} Task Management App. All rights reserved. | Copyright © Atta-ur-rehman</p>
          </div>
        </AuthProvider>
      </body>
    </html>
  );
}