'use client';

import { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import useAuth from '../../hooks/use-auth';
import { taskService } from '../../lib/api/task-service';
import { TaskModal } from '../../components/tasks/task-modal';
import { ToastContainer } from '../../components/ui/toast';

export default function DashboardPage() {
  const { user, isLoading, isAuthenticated, error } = useAuth();
  const router = useRouter();
  const [stats, setStats] = useState({ total: 0, completed: 0, pending: 0 });
  const [statsLoading, setStatsLoading] = useState(true);
  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false);
  const [toasts, setToasts] = useState<Array<{ id: string; type: 'success' | 'error' | 'warning' | 'info'; message: string }>>([]);

  // Toast helper functions
  const addToast = (type: 'success' | 'error' | 'warning' | 'info', message: string) => {
    const id = Date.now().toString();
    setToasts((prev) => [...prev, { id, type, message }]);
  };

  const removeToast = (id: string) => {
    setToasts((prev) => prev.filter((toast) => toast.id !== id));
  };

  // Fetch task statistics
  const fetchStats = async () => {
    if (isAuthenticated && user?.id) {
      try {
        setStatsLoading(true);
        const statsData = await taskService.getTaskStats(user.id);
        setStats(statsData);
      } catch (err) {
        console.error('Error fetching task stats:', err);
        // Set default values if there's an error
        setStats({ total: 0, completed: 0, pending: 0 });
      } finally {
        setStatsLoading(false);
      }
    }
  };

  // If not authenticated, redirect to sign-in
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      router.push('/auth/sign-in');
    }
  }, [isAuthenticated, isLoading, router]);

  // Fetch task statistics when user is authenticated
  useEffect(() => {
    fetchStats();
  }, [isAuthenticated, user?.id]);

  // Handle create task
  const handleCreateTask = async (data: { title: string; description: string; completed: boolean }) => {
    if (!user?.id) return;

    try {
      await taskService.createTask(user.id, data);
      await fetchStats(); // Refresh stats after creating task
      addToast('success', 'Task created successfully!');
    } catch (err: any) {
      console.error('Error creating task:', err);
      addToast('error', err.message || 'Failed to create task');
      throw err;
    }
  };

  if (isLoading) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-purple-50 via-pink-50 to-cyan-50 dark:from-slate-900 dark:via-purple-900/20 dark:to-slate-900 flex items-center justify-center relative overflow-hidden">
        {/* Animated background elements */}
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute -top-40 -right-40 w-80 h-80 bg-purple-400/30 dark:bg-purple-600/20 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute -bottom-40 -left-40 w-80 h-80 bg-pink-400/30 dark:bg-pink-600/20 rounded-full blur-3xl animate-pulse delay-1000"></div>
          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-96 h-96 bg-cyan-400/20 dark:bg-cyan-600/10 rounded-full blur-3xl animate-pulse delay-500"></div>
        </div>

        {/* Loading content */}
        <div className="relative z-10 text-center">
          {/* Animated logo/icon */}
          <div className="relative mb-8">
            {/* Outer rotating ring */}
            <div className="absolute inset-0 w-32 h-32 mx-auto">
              <div className="w-full h-full border-4 border-purple-200 dark:border-purple-800 border-t-purple-600 dark:border-t-purple-400 rounded-full animate-spin"></div>
            </div>

            {/* Middle pulsing ring */}
            <div className="absolute inset-0 w-32 h-32 mx-auto flex items-center justify-center">
              <div className="w-24 h-24 border-4 border-pink-200 dark:border-pink-800 border-b-pink-600 dark:border-b-pink-400 rounded-full animate-spin" style={{animationDirection: 'reverse', animationDuration: '1.5s'}}></div>
            </div>

            {/* Center icon */}
            <div className="relative w-32 h-32 mx-auto flex items-center justify-center">
              <div className="w-16 h-16 rounded-2xl bg-gradient-to-br from-purple-600 to-pink-600 flex items-center justify-center shadow-2xl shadow-purple-500/50 animate-pulse">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
                </svg>
              </div>
            </div>
          </div>

          {/* Loading text */}
          <div className="space-y-3">
            <h2 className="text-3xl font-extrabold bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent animate-pulse">
              Loading Dashboard
            </h2>
            <p className="text-gray-600 dark:text-gray-400 text-lg font-medium">
              Preparing your workspace
            </p>

            {/* Animated dots */}
            <div className="flex items-center justify-center gap-2 mt-4">
              <div className="w-2 h-2 bg-purple-600 dark:bg-purple-400 rounded-full animate-bounce"></div>
              <div className="w-2 h-2 bg-pink-600 dark:bg-pink-400 rounded-full animate-bounce" style={{animationDelay: '0.1s'}}></div>
              <div className="w-2 h-2 bg-cyan-600 dark:bg-cyan-400 rounded-full animate-bounce" style={{animationDelay: '0.2s'}}></div>
            </div>
          </div>

          {/* Progress bar */}
          <div className="mt-8 w-64 mx-auto">
            <div className="h-2 bg-gray-200 dark:bg-gray-800 rounded-full overflow-hidden">
              <div className="h-full bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 rounded-full animate-pulse" style={{width: '100%'}}></div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 flex items-center justify-center p-4">
        <div className="max-w-md w-full bg-white rounded-xl shadow-lg p-8 border border-red-100">
          <div className="text-center">
            <div className="mx-auto flex items-center justify-center h-12 w-12 rounded-full bg-red-100">
              <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-red-600" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
            </div>
            <h3 className="mt-4 text-lg font-medium text-gray-900">Access Error</h3>
            <p className="mt-2 text-sm text-gray-500">{error}</p>
            <div className="mt-6">
              <button
                onClick={() => router.push('/auth/sign-in')}
                className="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md shadow-sm text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
              >
                Sign In Again
              </button>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen">
      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-2 min-h-[calc(100vh-56px)] flex flex-col justify-center items-center">
        {/* Stats Overview - Interactive Clickable Cards */}
        <div className="w-full flex flex-col md:grid md:grid-cols-3 gap-4 md:gap-5 mb-6 md:mb-8 items-center justify-center">
          {/* Total Tasks Card */}
          <button
            onClick={() => router.push('/dashboard/tasks')}
            className="group relative overflow-hidden rounded-2xl bg-gradient-to-br from-purple-600 to-purple-700 p-6 md:p-8 shadow-2xl shadow-purple-500/40 hover:shadow-purple-500/60 transition-all duration-300 hover:-translate-y-2 hover:scale-105 text-left cursor-pointer w-full max-w-sm"
          >
            <div className="absolute top-0 right-0 w-24 h-24 md:w-32 md:h-32 bg-white/10 rounded-full -mr-12 md:-mr-16 -mt-12 md:-mt-16 group-hover:scale-150 transition-transform duration-500"></div>
            <div className="relative z-10">
              <div className="flex items-center justify-between mb-2 md:mb-3">
                <div className="w-10 h-10 md:w-14 md:h-14 bg-white/20 backdrop-blur-sm rounded-xl flex items-center justify-center shadow-lg">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5 md:h-7 md:w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
                  </svg>
                </div>
              </div>
              <p className="text-white/90 text-xs md:text-sm font-bold uppercase tracking-wider mb-1 md:mb-2">Total Tasks</p>
              {statsLoading ? (
                <p className="text-4xl md:text-6xl font-extrabold text-white animate-pulse">...</p>
              ) : (
                <>
                  <p className="text-4xl md:text-6xl font-extrabold text-white mb-1">{ stats.total}</p>
                  <p className="text-xs md:text-sm text-white/70 font-medium">Click to view all</p>
                </>
              )}
            </div>
          </button>

          {/* Completed Tasks Card */}
          <button
            onClick={() => router.push('/dashboard/tasks?filter=completed')}
            className="group relative overflow-hidden rounded-2xl bg-gradient-to-br from-green-600 to-green-700 p-6 md:p-8 shadow-2xl shadow-green-500/40 hover:shadow-green-500/60 transition-all duration-300 hover:-translate-y-2 hover:scale-105 text-left cursor-pointer w-full max-w-sm"
          >
            <div className="absolute top-0 right-0 w-24 h-24 md:w-32 md:h-32 bg-white/10 rounded-full -mr-12 md:-mr-16 -mt-12 md:-mt-16 group-hover:scale-150 transition-transform duration-500"></div>
            <div className="relative z-10">
              <div className="flex items-center justify-between mb-2 md:mb-3">
                <div className="w-10 h-10 md:w-14 md:h-14 bg-white/20 backdrop-blur-sm rounded-xl flex items-center justify-center shadow-lg">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5 md:h-7 md:w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                </div>
              </div>
              <p className="text-white/90 text-xs md:text-sm font-bold uppercase tracking-wider mb-1 md:mb-2">Completed</p>
              {statsLoading ? (
                <p className="text-4xl md:text-6xl font-extrabold text-white animate-pulse">...</p>
              ) : (
                <>
                  <p className="text-4xl md:text-6xl font-extrabold text-white mb-1">{stats.completed}</p>
                  <p className="text-xs md:text-sm text-white/70 font-medium">
                    {stats.total > 0 ? `${Math.round((stats.completed / stats.total) * 100)}% complete` : 'No tasks yet'}
                  </p>
                </>
              )}
            </div>
          </button>

          {/* Pending Tasks Card */}
          <button
            onClick={() => router.push('/dashboard/tasks?filter=pending')}
            className="group relative overflow-hidden rounded-2xl bg-gradient-to-br from-pink-600 to-pink-700 p-6 md:p-8 shadow-2xl shadow-pink-500/40 hover:shadow-pink-500/60 transition-all duration-300 hover:-translate-y-2 hover:scale-105 text-left cursor-pointer w-full max-w-sm"
          >
            <div className="absolute top-0 right-0 w-24 h-24 md:w-32 md:h-32 bg-white/10 rounded-full -mr-12 md:-mr-16 -mt-12 md:-mt-16 group-hover:scale-150 transition-transform duration-500"></div>
            <div className="relative z-10">
              <div className="flex items-center justify-between mb-2 md:mb-3">
                <div className="w-10 h-10 md:w-14 md:h-14 bg-white/20 backdrop-blur-sm rounded-xl flex items-center justify-center shadow-lg">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5 md:h-7 md:w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                </div>
              </div>
              <p className="text-white/90 text-xs md:text-sm font-bold uppercase tracking-wider mb-1 md:mb-2">Pending</p>
              {statsLoading ? (
                <p className="text-4xl md:text-6xl font-extrabold text-white animate-pulse">...</p>
              ) : (
                <>
                  <p className="text-4xl md:text-6xl font-extrabold text-white mb-1">{stats.pending}</p>
                  <p className="text-xs md:text-sm text-white/70 font-medium">
                    {stats.pending > 0 ? 'Needs attention' : 'All caught up!'}
                  </p>
                </>
              )}
            </div>
          </button>
        </div>

        {/* Welcome Section - Compact & Action-Focused */}
        <div className="w-full relative overflow-hidden rounded-2xl glass-strong p-3 shadow-xl border-2 border-white/30 dark:border-white/20">
          {/* Decorative gradient orbs */}
          <div className="absolute top-0 right-0 w-48 h-48 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-full blur-3xl -mr-24 -mt-24"></div>
          <div className="absolute bottom-0 left-0 w-48 h-48 bg-gradient-to-tr from-cyan-500/20 to-green-500/20 rounded-full blur-3xl -ml-24 -mb-24"></div>

          <div className="relative z-10">
            <div className="flex flex-col md:flex-row items-center justify-between gap-4">
              {/* Left Side - Welcome Message */}
              <div className="flex items-center gap-3">
                <div className="w-12 h-12 bg-gradient-to-br from-purple-600 to-pink-600 rounded-2xl flex items-center justify-center shadow-lg shadow-purple-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                  </svg>
                </div>
                <div>
                  <h2 className="text-xl font-extrabold bg-gradient-to-r from-purple-600 to-pink-600 dark:from-purple-400 dark:to-pink-400 bg-clip-text text-transparent">
                    Welcome back, {user?.first_name || 'User'}!
                  </h2>
                  <p className="text-sm text-gray-600 dark:text-gray-400 font-medium">
                    Ready to tackle your tasks?
                  </p>
                </div>
              </div>

              {/* Right Side - Action Buttons */}
              <div className="flex flex-col sm:flex-row gap-2">
                {/* Primary CTA - Create Task */}
                <button
                  onClick={() => setIsCreateModalOpen(true)}
                  className="group relative overflow-hidden bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-500 hover:to-pink-500 text-white px-5 py-2.5 rounded-xl shadow-lg shadow-purple-500/50 hover:shadow-xl hover:shadow-purple-500/60 transition-all duration-300 transform hover:scale-105"
                >
                  <div className="absolute inset-0 bg-gradient-to-r from-white/0 via-white/20 to-white/0 translate-x-[-200%] group-hover:translate-x-[200%] transition-transform duration-700"></div>
                  <div className="relative z-10 flex items-center justify-center gap-2">
                    <svg xmlns="http://www.w3.org/2000/svg" className="h-4 w-4" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
                    </svg>
                    <span className="font-bold">Create Task</span>
                  </div>
                </button>

                {/* Secondary CTA - View Tasks */}
                <button
                  onClick={() => router.push('/dashboard/tasks')}
                  className="px-5 py-2.5 rounded-xl border-2 border-purple-500 dark:border-purple-400 bg-white/50 dark:bg-slate-800/50 hover:bg-purple-50 dark:hover:bg-slate-700 text-purple-700 dark:text-purple-300 font-bold shadow-md hover:shadow-lg transition-all duration-300 transform hover:scale-105"
                >
                  View All Tasks
                </button>
              </div>
            </div>
          </div>
        </div>
      </main>

      {/* Task Creation Modal */}
      <TaskModal
        isOpen={isCreateModalOpen}
        onClose={() => setIsCreateModalOpen(false)}
        onSubmit={handleCreateTask}
        mode="create"
      />

      {/* Toast Notifications */}
      <ToastContainer toasts={toasts} onClose={removeToast} />
    </div>
  );
}