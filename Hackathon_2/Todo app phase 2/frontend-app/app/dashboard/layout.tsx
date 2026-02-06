'use client';

import React, { useState, useEffect } from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { useRouter } from 'next/navigation';
import { ResponsiveLayout } from '../../components/ui/responsive-layout';
import Avatar from '../../components/ui/avatar';
import useAuth from '../../hooks/use-auth';
import { ChatInterface } from '../../components/chat/chat-interface';
import { ChatErrorBoundary } from '../../components/chat/chat-error-boundary';
import { Home, ListTodo, User, TrendingUp, MessageCircle, Moon, Sun } from 'lucide-react';
import { useTheme } from 'next-themes';

interface DashboardLayoutProps {
  children: React.ReactNode;
}

const DashboardLayout: React.FC<DashboardLayoutProps> = ({ children }) => {
  const pathname = usePathname();
  const router = useRouter();
  const { user, signOut } = useAuth();
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [isSidebarCollapsed, setIsSidebarCollapsed] = useState(false);
  const { theme, setTheme } = useTheme();
  const [mounted, setMounted] = useState(false);

  // Prevent hydration mismatch for theme
  useEffect(() => {
    setMounted(true);
  }, []);

  const navigationItems = [
    { name: 'Dashboard', href: '/dashboard', active: pathname === '/dashboard' || pathname?.startsWith('/dashboard/') && !pathname.includes('/tasks') && !pathname.includes('/profile') },
    { name: 'Tasks', href: '/dashboard/tasks', active: pathname?.includes('/dashboard/tasks') },
    { name: 'Profile', href: '/dashboard/profile', active: pathname?.includes('/dashboard/profile') },
  ];

  // Close chat on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isChatOpen) {
        setIsChatOpen(false);
      }
    };

    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [isChatOpen]);

  return (
    <div className="flex flex-col min-h-screen">
      <ResponsiveLayout
        title="Dashboard"
        sidebar={
          <div className="flex flex-col h-full p-4 bg-gradient-to-b from-slate-50 to-slate-100 dark:from-slate-900 dark:to-slate-950">
            {/* Sidebar Header with Collapse Toggle */}
            <div className="mb-6">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-3">
                  <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-purple-600 to-pink-600 flex items-center justify-center shadow-lg shadow-purple-500/50">
                    <ListTodo className="w-5 h-5 text-white" />
                  </div>
                  {!isSidebarCollapsed && (
                    <h2 className="text-xl font-extrabold bg-gradient-to-r from-purple-600 to-pink-600 dark:from-purple-400 dark:to-pink-400 bg-clip-text text-transparent">
                      TaskFlow
                    </h2>
                  )}
                </div>
                <button
                  onClick={() => setIsSidebarCollapsed(!isSidebarCollapsed)}
                  className="p-2 rounded-lg hover:bg-white/50 dark:hover:bg-slate-800/50 transition-all"
                  aria-label="Toggle sidebar"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    className={`w-5 h-5 text-gray-600 dark:text-gray-400 transition-transform ${isSidebarCollapsed ? 'rotate-180' : ''}`}
                    fill="none"
                    viewBox="0 0 24 24"
                    stroke="currentColor"
                  >
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M11 19l-7-7 7-7m8 14l-7-7 7-7" />
                  </svg>
                </button>
              </div>
              {!isSidebarCollapsed && (
                <p className="text-sm text-gray-600 dark:text-gray-400 ml-13">
                  Welcome, {user?.first_name || 'User'}!
                </p>
              )}
            </div>

            {/* Navigation */}
            <nav className="flex-1">
              <ul className="space-y-2">
                {navigationItems.map((item) => (
                  <li key={item.name}>
                    <Link
                      href={item.href}
                      className={`flex items-center gap-3 px-4 py-3 text-base font-bold rounded-xl transition-all duration-200 group relative ${
                        item.active
                          ? 'bg-gradient-to-r from-purple-600 to-pink-600 text-white shadow-lg shadow-purple-500/50 scale-105'
                          : 'text-gray-700 dark:text-gray-300 hover:bg-white/70 dark:hover:bg-slate-800/70 hover:scale-102'
                      }`}
                      title={isSidebarCollapsed ? item.name : undefined}
                    >
                      {/* Active indicator - left border glow */}
                      {item.active && (
                        <div className="absolute left-0 top-1/2 -translate-y-1/2 w-1 h-8 bg-white rounded-r-full shadow-lg shadow-white/50" />
                      )}

                      <span className="flex-shrink-0">
                        {item.name === 'Dashboard' ? <Home className="w-5 h-5" /> :
                         item.name === 'Tasks' ? <ListTodo className="w-5 h-5" /> :
                         <User className="w-5 h-5" />}
                      </span>

                      {!isSidebarCollapsed && (
                        <>
                          <span>{item.name}</span>
                          {item.active && (
                            <div className="ml-auto w-2 h-2 rounded-full bg-white animate-pulse" />
                          )}
                        </>
                      )}
                    </Link>
                  </li>
                ))}
              </ul>
            </nav>

            {/* User Info Card at Bottom */}
            <div className="mt-auto pt-4 border-t-2 border-gray-300 dark:border-gray-700">
              <div className="bg-white/70 dark:bg-slate-800/70 backdrop-blur-sm rounded-2xl p-4 border border-gray-200 dark:border-gray-700 shadow-lg">
                <div className={`flex items-center gap-3 ${isSidebarCollapsed ? 'justify-center' : ''} mb-3`}>
                  <Avatar
                    src={user?.avatar ? (user.avatar.startsWith('http') ? user.avatar : `${process.env.NEXT_PUBLIC_BACKEND_URL}${user.avatar}`) : `https://ui-avatars.com/api/?name=${user?.first_name && user?.last_name ? `${encodeURIComponent(user.first_name)}+${encodeURIComponent(user.last_name)}` : encodeURIComponent(user?.first_name || user?.last_name || 'User')}&background=8B5CF6&color=fff`}
                    alt={`${user?.first_name || ''} ${user?.last_name || ''}`.trim() || 'User'}
                    className="w-10 h-10 rounded-xl border-2 border-purple-500/50 shadow-lg"
                  />
                  {!isSidebarCollapsed && (
                    <div className="flex-1 min-w-0">
                      <p className="text-sm font-bold text-gray-900 dark:text-gray-100 truncate">
                        {user?.first_name || user?.email || 'User'}
                      </p>
                      <p className="text-xs text-purple-600 dark:text-purple-400 font-semibold">
                        Free Tier
                      </p>
                    </div>
                  )}
                </div>
                {!isSidebarCollapsed && (
                  <div className="flex items-center gap-2 text-xs text-gray-600 dark:text-gray-400">
                    <TrendingUp className="w-4 h-4 text-green-500" />
                    <span>Active</span>
                  </div>
                )}
              </div>
            </div>
          </div>
        }
      >
        {/* Top Navbar */}
        <header className="sticky top-0 z-50 w-full border-b border-gray-200 dark:border-gray-800 bg-white/95 dark:bg-slate-900/95 backdrop-blur-xl shadow-sm">
          <div className="flex items-center justify-between h-16 px-4 md:px-6">
            {/* Logo - Mobile Only */}
            <div className="flex items-center gap-3 md:hidden">
              <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-purple-600 to-pink-600 flex items-center justify-center shadow-lg shadow-purple-500/50">
                <ListTodo className="w-4 h-4 text-white" />
              </div>
              <h1 className="text-lg font-extrabold bg-gradient-to-r from-purple-600 to-pink-600 dark:from-purple-400 dark:to-pink-400 bg-clip-text text-transparent">
                TaskFlow
              </h1>
            </div>

            {/* Search Bar - Desktop */}
            <div className="hidden md:flex flex-1 max-w-md">
              <div className="relative w-full">
                <input
                  type="text"
                  placeholder="Search tasks..."
                  className="w-full px-4 py-2 pl-10 text-sm bg-gray-100 dark:bg-slate-800 border border-gray-200 dark:border-gray-700 rounded-xl focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-transparent transition-all"
                />
                <svg
                  className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
                </svg>
              </div>
            </div>

            {/* Right Side Actions */}
            <div className="flex items-center gap-2 md:gap-3">
              {/* Search Icon - Mobile */}
              <button className="md:hidden p-2 rounded-lg hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors">
                <svg
                  className="w-5 h-5 text-gray-600 dark:text-gray-400"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
                </svg>
              </button>

              {/* Dark Mode Toggle */}
              {mounted && (
                <button
                  onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}
                  className="p-2 rounded-lg hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors text-gray-600 dark:text-gray-400"
                  title={theme === 'dark' ? 'Light mode' : 'Dark mode'}
                >
                  {theme === 'dark' ? <Sun className="w-5 h-5" /> : <Moon className="w-5 h-5" />}
                </button>
              )}

              {/* User Avatar */}
              <div className="relative group">
                <button className="flex items-center gap-2 p-1 rounded-lg hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors">
                  <Avatar
                    src={user?.avatar ? (user.avatar.startsWith('http') ? user.avatar : `${process.env.NEXT_PUBLIC_BACKEND_URL}${user.avatar}`) : `https://ui-avatars.com/api/?name=${user?.first_name && user?.last_name ? `${encodeURIComponent(user.first_name)}+${encodeURIComponent(user.last_name)}` : encodeURIComponent(user?.first_name || user?.last_name || 'User')}&background=8B5CF6&color=fff`}
                    alt={`${user?.first_name || ''} ${user?.last_name || ''}`.trim() || 'User'}
                    className="w-8 h-8 rounded-lg border-2 border-purple-500/50"
                  />
                </button>

                {/* Dropdown Menu */}
                <div className="absolute right-0 mt-2 w-48 bg-white dark:bg-slate-800 rounded-xl shadow-lg border border-gray-200 dark:border-gray-700 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all duration-200">
                  <div className="p-3 border-b border-gray-200 dark:border-gray-700">
                    <p className="text-sm font-semibold text-gray-900 dark:text-gray-100">
                      {user?.first_name || user?.email || 'User'}
                    </p>
                    <p className="text-xs text-gray-500 dark:text-gray-400">
                      {user?.email}
                    </p>
                  </div>
                  <div className="p-2">
                    <Link
                      href="/dashboard/profile"
                      className="block px-3 py-2 text-sm text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-slate-700 rounded-lg transition-colors"
                    >
                      Profile Settings
                    </Link>
                    <button
                      onClick={() => signOut()}
                      className="w-full text-left px-3 py-2 text-sm text-red-600 dark:text-red-400 hover:bg-red-50 dark:hover:bg-red-900/20 rounded-lg transition-colors"
                    >
                      Sign Out
                    </button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </header>

        <div className="flex-grow relative">
          {children}

          {/* Floating Chat Button - Upgraded UI */}
          <div className="fixed bottom-6 right-6 z-[100]">
            {/* Animated gradient ring */}
            <div className={`absolute inset-0 rounded-full bg-gradient-to-r from-purple-500 via-pink-500 to-cyan-500 blur-lg opacity-75 ${!isChatOpen ? 'animate-pulse' : ''}`}></div>

            {/* Main button */}
            <button
              onClick={() => setIsChatOpen(!isChatOpen)}
              className={`relative w-16 h-16 rounded-full backdrop-blur-xl border-2 transition-all duration-500 transform group ${
                isChatOpen
                  ? 'bg-gradient-to-br from-red-500 via-red-600 to-red-700 border-red-400/50 scale-100 shadow-2xl shadow-red-500/50'
                  : 'bg-gradient-to-br from-purple-600 via-pink-600 to-purple-700 border-purple-400/50 hover:scale-110 hover:rotate-12 shadow-2xl shadow-purple-500/50'
              }`}
              title="AI Assistant"
            >
              {/* Inner glow effect */}
              <div className="absolute inset-0 rounded-full bg-gradient-to-br from-white/20 to-transparent opacity-50"></div>

              {/* Icon with animation */}
              <div className={`relative flex items-center justify-center transition-transform duration-500 ${isChatOpen ? '' : 'group-hover:scale-110'}`}>
                {isChatOpen ? (
                  <svg className="w-8 h-8 text-white font-bold" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth={3}>
                    <path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
                  </svg>
                ) : (
                  <MessageCircle className="w-7 h-7 text-white drop-shadow-lg" />
                )}
              </div>

              {/* Notification badge (optional - can be shown when there are new messages) */}
              {!isChatOpen && (
                <div className="absolute -top-1 -right-1 w-4 h-4 bg-red-500 rounded-full border-2 border-white dark:border-slate-900 animate-bounce">
                  <div className="absolute inset-0 bg-red-500 rounded-full animate-ping opacity-75"></div>
                </div>
              )}
            </button>

            {/* Hover label */}
            <div className={`absolute right-20 top-1/2 -translate-y-1/2 px-4 py-2 bg-slate-900 dark:bg-white text-white dark:text-slate-900 text-sm font-semibold rounded-lg shadow-xl whitespace-nowrap transition-all duration-300 ${
              isChatOpen ? 'opacity-0 pointer-events-none translate-x-2' : 'opacity-0 group-hover:opacity-100 pointer-events-none'
            }`}>
              AI Assistant
              <div className="absolute right-0 top-1/2 -translate-y-1/2 translate-x-1/2 rotate-45 w-2 h-2 bg-slate-900 dark:bg-white"></div>
            </div>
          </div>

          {/* Floating Chat Interface */}
          {isChatOpen && (
            <>
              {/* Mobile overlay */}
              <div
                className="fixed inset-0 bg-black/50 backdrop-blur-sm z-[60] md:hidden animate-fade-in"
                onClick={() => setIsChatOpen(false)}
              />

              {/* Floating Chat Panel - Desktop */}
              <div className="fixed bottom-28 right-6 w-[380px] h-[600px] max-h-[calc(100vh-140px)] z-[70] md:block hidden">
                <div className="relative h-full rounded-3xl glass-strong border-2 border-purple-400/40 dark:border-purple-500/50 shadow-2xl shadow-purple-500/30 overflow-hidden">
                  {/* Decorative gradient glow */}
                  <div className="absolute inset-0 bg-gradient-to-br from-purple-500/10 via-pink-500/10 to-cyan-500/10 pointer-events-none"></div>

                  <ChatErrorBoundary>
                    <ChatInterface
                      isOpen={isChatOpen}
                      onClose={() => setIsChatOpen(false)}
                      className="h-full relative z-10"
                    />
                  </ChatErrorBoundary>
                </div>
              </div>

              {/* Mobile Full Screen Chat */}
              <div className="fixed inset-4 top-20 bottom-24 z-[70] md:hidden">
                <div className="relative h-full rounded-3xl glass-strong border-2 border-purple-400/40 dark:border-purple-500/50 shadow-2xl shadow-purple-500/30 overflow-hidden">
                  <div className="absolute inset-0 bg-gradient-to-br from-purple-500/10 via-pink-500/10 to-cyan-500/10 pointer-events-none"></div>

                  <ChatErrorBoundary>
                    <ChatInterface
                      isOpen={isChatOpen}
                      onClose={() => setIsChatOpen(false)}
                      className="h-full relative z-10"
                    />
                  </ChatErrorBoundary>
                </div>
              </div>
            </>
          )}
        </div>
      </ResponsiveLayout>
    </div>
  );
};

export default DashboardLayout;