import './globals.css';
import type { Metadata } from 'next';
import { Inter } from 'next/font/google';
import { AuthProvider } from '../components/providers/auth-provider';
import { ThemeProvider } from '../components/providers/theme-provider';
import { ModeToggle } from '../components/theme/mode-toggle';

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
    <html lang="en" suppressHydrationWarning>
      <body className={`${inter.className} flex flex-col min-h-screen`}>
        <ThemeProvider
          attribute="class"
          defaultTheme="dark"
          enableSystem
          disableTransitionOnChange={false}
        >
          <AuthProvider>
            {/* Floating Navbar */}
            {/* <header className="fixed top-4 left-1/2 -translate-x-1/2 z-50 w-[95%] max-w-7xl">
              <nav className="glass-strong rounded-2xl px-6 py-4 shadow-2xl border-2 border-white/30 dark:border-white/20 backdrop-blur-2xl">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-3">
                    <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-primary-500 to-secondary-500 flex items-center justify-center shadow-lg shadow-primary-500/50">
                      <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
                      </svg>
                    </div>
                    <h1 className="text-2xl font-extrabold bg-gradient-to-r from-primary-600 via-secondary-600 to-accent-600 dark:from-primary-400 dark:via-secondary-400 dark:to-accent-400 bg-clip-text text-transparent">
                      TaskFlow
                    </h1>
                  </div>
                  <ModeToggle />
                </div>
              </nav>
            </header> */}

            {/* Main Content with top padding for floating navbar */}
            <div className="flex-grow pt-24">
              {children}
            </div> 

            {/* Footer */}
            <footer className="py-6 text-center text-sm text-muted-foreground/70 border-t border-border/50 mt-auto backdrop-blur-sm">
              <p className="font-medium">© {new Date().getFullYear()} TaskFlow. All rights reserved. | Copyright © Atta-ur-rehman</p>
            </footer>
          </AuthProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}