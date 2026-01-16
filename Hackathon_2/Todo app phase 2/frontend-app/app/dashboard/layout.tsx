'use client';

import React, { useState, useEffect } from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { useRouter } from 'next/navigation';
import { ResponsiveLayout, SidebarItem } from '../../components/ui/responsive-layout';
import { Button } from '../../components/ui/button';
import Avatar from '../../components/ui/avatar';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger
} from '../../components/ui/dropdown';
import useAuth from '../../hooks/use-auth';

// Prevent SSR for components that cause hydration mismatches
const ClientOnlyDropdown = ({ user, router, signOut }: { user: any, router: any, signOut: () => Promise<void> }) => {
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    // Render a placeholder during SSR
    return (
      <div className="w-10 h-10 rounded-full cursor-pointer border-2 border-gray-200 animate-pulse bg-gray-200"></div>
    );
  }

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <button className="focus:outline-none">
          <Avatar
            src={user?.avatar ? (user.avatar.startsWith('http') ? user.avatar : `${process.env.NEXT_PUBLIC_BACKEND_URL}${user.avatar}`) : `https://ui-avatars.com/api/?name=${user?.first_name && user?.last_name ? `${encodeURIComponent(user.first_name)}+${encodeURIComponent(user.last_name)}` : encodeURIComponent(user?.first_name || user?.last_name || 'User')}&background=0D8ABC&color=fff`}
            alt={`${user?.first_name || ''} ${user?.last_name || ''}`.trim() || 'User'}
            className="w-10 h-10 rounded-full cursor-pointer border-2 border-gray-200 hover:border-blue-400"
          />
        </button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end" className="w-48 mt-2 mr-2 bg-white rounded-md shadow-lg border border-gray-200">
        <DropdownMenuItem
          onClick={() => router.push('/dashboard/profile')}
          className="flex items-center px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 cursor-pointer"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="mr-3 h-4 w-4" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 7a4 4 0 11-8 0 4 4 0 018 0zM12 14a7 7 0 00-7 7h14a7 7 0 00-7-7z" />
          </svg>
          Profile
        </DropdownMenuItem>
        <DropdownMenuItem
          onClick={() => {
            signOut();
          }}
          className="flex items-center px-4 py-2 text-sm text-red-600 hover:bg-red-50 cursor-pointer"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="mr-3 h-4 w-4" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M17 16l4-4m0 0l-4-4m4 4H7m6 4v1a3 3 0 01-3 3H6a3 3 0 01-3-3V7a3 3 0 013-3h4a3 3 0 013 3v1" />
          </svg>
          Logout
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
};

interface DashboardLayoutProps {
  children: React.ReactNode;
}

const DashboardLayout: React.FC<DashboardLayoutProps> = ({ children }) => {
  const pathname = usePathname();
  const router = useRouter();
  const { user, signOut } = useAuth();

  const navigationItems = [
    { name: 'Dashboard', href: '/dashboard', active: pathname === '/dashboard' || pathname?.startsWith('/dashboard/') && !pathname.includes('/tasks') && !pathname.includes('/profile') },
    { name: 'Tasks', href: '/dashboard/tasks', active: pathname?.includes('/dashboard/tasks') },
    { name: 'Profile', href: '/dashboard/profile', active: pathname?.includes('/dashboard/profile') },
  ];

  return (
    <div className="flex flex-col min-h-screen">
      <ResponsiveLayout
        title="Dashboard"
        sidebar={
          <div className="flex flex-col h-full">
            <div className="p-4">
              <h2 className="text-lg font-semibold">Welcome, {user?.first_name || user?.email || 'User'}!</h2>
            </div>
            <nav className="flex-1 px-2 py-4">
              <ul className="space-y-1">
                {navigationItems.map((item) => (
                  <li key={item.name}>
                    <SidebarItem
                      href={item.href}
                      active={item.active}
                    >
                      {item.name}
                    </SidebarItem>
                  </li>
                ))}
              </ul>
            </nav>
            <div className="p-4 border-t border-gray-200">
              <p className="text-xs text-gray-500">Manage account settings in your profile</p>
            </div>
          </div>
        }
        header={
          <header className="bg-white shadow-sm">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
              <div className="flex justify-between h-16">
                <div className="flex">
                  <div className="flex-shrink-0 flex items-center">
                    <Link href="/dashboard" className="text-xl font-bold text-blue-600">
                      Task Manager
                    </Link>
                  </div>
                </div>
                <div className="flex items-center">
                  <div className="ml-3 relative">
                    <ClientOnlyDropdown user={user} router={router} signOut={signOut} />
                  </div>
                </div>
              </div>
            </div>
          </header>
        }
      >
        <div className="flex-grow">
          {children}
        </div>
      </ResponsiveLayout>
    </div>
  );
};

export default DashboardLayout;