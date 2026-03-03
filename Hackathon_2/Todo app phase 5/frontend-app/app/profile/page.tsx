'use client';

import { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { motion } from 'framer-motion';
import axios from 'axios';
import ProfileHeader from '@/components/profile/profile-header';
import StatsGrid from '@/components/profile/stats-grid';
import SettingsPanel from '@/components/profile/settings-panel';
import Spinner from '@/components/ui/spinner';
import { taskService } from '@/lib/api/task-service';

const ProfilePage = () => {
  const [user, setUser] = useState<any>(null);
  const [stats, setStats] = useState({ total: 0, completed: 0, pending: 0 });
  const [loading, setLoading] = useState(true);
  const [statsLoading, setStatsLoading] = useState(true);
  const [error, setError] = useState('');
  const router = useRouter();

  useEffect(() => {
    const fetchUserAndStats = async () => {
      try {
        const token = localStorage.getItem('token');
        if (!token) {
          router.push('/auth/sign-in');
          return;
        }

        // Fetch user profile
        const backendUrl = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8001';
        const response = await axios.get(`${backendUrl}/api/profile`, {
          headers: {
            'Authorization': `Bearer ${token}`
          }
        });

        setUser(response.data.user);

        // Fetch task statistics
        if (response.data.user?.id) {
          try {
            setStatsLoading(true);
            const statsData = await taskService.getTaskStats(response.data.user.id);
            setStats(statsData);
          } catch (err) {
            console.error('Error fetching task stats:', err);
            setStats({ total: 0, completed: 0, pending: 0 });
          } finally {
            setStatsLoading(false);
          }
        }
      } catch (err: any) {
        const errorMsg = err.response?.data?.detail || err.response?.data?.message || 'Failed to load user data';
        setError(errorMsg);
        localStorage.removeItem('token');
        router.push('/auth/sign-in');
      } finally {
        setLoading(false);
      }
    };

    fetchUserAndStats();
  }, [router]);

  if (loading) {
    return (
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        className="min-h-screen flex items-center justify-center"
      >
        <Spinner size="lg" label="Loading your profile..." />
      </motion.div>
    );
  }

  if (error) {
    return (
      <motion.div
        initial={{ opacity: 0, scale: 0.95 }}
        animate={{ opacity: 1, scale: 1 }}
        className="min-h-screen flex items-center justify-center p-4"
      >
        <div className="glass-strong rounded-3xl p-12 max-w-md w-full text-center">
          <div className="mx-auto flex items-center justify-center h-16 w-16 rounded-full bg-red-100 dark:bg-red-900/30 mb-4">
            <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-red-600 dark:text-red-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
          </div>
          <h3 className="text-2xl font-bold text-gray-900 dark:text-gray-100 mb-2">Access Error</h3>
          <p className="text-gray-600 dark:text-gray-400 mb-6">{error}</p>
          <button
            onClick={() => router.push('/auth/sign-in')}
            className="px-6 py-3 rounded-xl font-bold text-white bg-gradient-to-r from-purple-500 to-pink-500 hover:from-purple-400 hover:to-pink-400 shadow-lg shadow-purple-500/50 transition-all"
          >
            Sign In Again
          </button>
        </div>
      </motion.div>
    );
  }

  return (
    <motion.div
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      className="min-h-screen py-8 px-4"
    >
      <div className="max-w-7xl mx-auto space-y-8">
        {/* Page Title */}
        <motion.div
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
        >
          <h1 className="text-5xl font-extrabold mb-2 bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent">
            Your Profile
          </h1>
          <p className="text-lg text-gray-600 dark:text-gray-400">
            Manage your account settings and view your productivity stats
          </p>
        </motion.div>

        {/* Profile Header */}
        {user && <ProfileHeader user={user} />}

        {/* Stats Grid */}
        <div>
          <motion.h2
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.3 }}
            className="text-3xl font-bold mb-6 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent"
          >
            Your Statistics
          </motion.h2>
          <StatsGrid stats={stats} loading={statsLoading} />
        </div>

        {/* Activity Summary */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.5 }}
          className="glass-strong rounded-3xl p-8 border-2 border-white/30 dark:border-white/20 shadow-2xl"
        >
          <h2 className="text-3xl font-bold mb-6 bg-gradient-to-r from-cyan-600 to-blue-600 bg-clip-text text-transparent">
            Recent Activity
          </h2>
          <div className="space-y-4">
            {stats.completed > 0 ? (
              <motion.div
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.6 }}
                className="flex items-center gap-4 p-4 rounded-xl bg-white/50 dark:bg-slate-800/50"
              >
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-green-500 to-emerald-500 flex items-center justify-center shadow-lg shadow-green-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="w-6 h-6 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                </div>
                <div className="flex-1">
                  <h3 className="font-bold text-gray-900 dark:text-gray-100">
                    Completed {stats.completed} {stats.completed === 1 ? 'task' : 'tasks'}
                  </h3>
                  <p className="text-sm text-gray-600 dark:text-gray-400">
                    Great job staying productive!
                  </p>
                </div>
              </motion.div>
            ) : (
              <motion.div
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.6 }}
                className="flex items-center gap-4 p-4 rounded-xl bg-white/50 dark:bg-slate-800/50"
              >
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-purple-500 to-pink-500 flex items-center justify-center shadow-lg shadow-purple-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="w-6 h-6 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                  </svg>
                </div>
                <div className="flex-1">
                  <h3 className="font-bold text-gray-900 dark:text-gray-100">
                    Ready to get started?
                  </h3>
                  <p className="text-sm text-gray-600 dark:text-gray-400">
                    Create your first task and start being productive!
                  </p>
                </div>
              </motion.div>
            )}

            {stats.pending > 0 && (
              <motion.div
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.7 }}
                className="flex items-center gap-4 p-4 rounded-xl bg-white/50 dark:bg-slate-800/50"
              >
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-yellow-500 to-orange-500 flex items-center justify-center shadow-lg shadow-yellow-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="w-6 h-6 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                  </svg>
                </div>
                <div className="flex-1">
                  <h3 className="font-bold text-gray-900 dark:text-gray-100">
                    {stats.pending} {stats.pending === 1 ? 'task' : 'tasks'} in progress
                  </h3>
                  <p className="text-sm text-gray-600 dark:text-gray-400">
                    Keep up the momentum!
                  </p>
                </div>
              </motion.div>
            )}
          </div>
        </motion.div>

        {/* Settings Panel */}
        <div>
          <motion.h2
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.6 }}
            className="text-3xl font-bold mb-6 bg-gradient-to-r from-pink-600 to-rose-600 bg-clip-text text-transparent"
          >
            Settings
          </motion.h2>
          <SettingsPanel />
        </div>
      </div>
    </motion.div>
  );
};

export default ProfilePage;
