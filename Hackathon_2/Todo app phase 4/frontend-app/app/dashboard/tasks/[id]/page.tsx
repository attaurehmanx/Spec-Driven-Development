'use client';

import React, { useState, useEffect } from 'react';
import { useParams, useRouter } from 'next/navigation';
import { motion, AnimatePresence } from 'framer-motion';
import ProtectedRoute from '@/components/auth/protected-route';
import { Card, CardContent, CardHeader, CardTitle, CardDescription, CardFooter } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import Spinner from '@/components/ui/spinner';
import { TaskLoadingCard } from '@/components/ui/loading-card';
import ErrorMessage from '@/components/ui/error-message';
import { Badge } from '@/components/ui/badge';
import { Task } from '@/types';
import useAuth from '@/hooks/use-auth';
import { taskService } from '@/lib/api/task-service';
import { triggerConfetti } from '@/lib/confetti';
import { ChevronLeft, Edit2, Trash2, Check, Clock, Calendar, RefreshCw } from 'lucide-react';

const TaskDetailPage: React.FC = () => {
  const { id } = useParams<{ id: string }>();
  const router = useRouter();
  const { user } = useAuth();

  const [task, setTask] = useState<Task | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [isToggling, setIsToggling] = useState(false);

  useEffect(() => {
    const fetchTask = async () => {
      if (!id || !user?.id) return;

      try {
        setLoading(true);
        setError(null);

        const response = await taskService.getTaskById(user.id, id);
        setTask(response);
      } catch (err: any) {
        console.error('Error fetching task:', err);
        setError(err.message || 'Failed to load task');
      } finally {
        setLoading(false);
      }
    };

    fetchTask();
  }, [id, user]);

  const handleToggleCompletion = async () => {
    if (!task || !user?.id || isToggling) return;

    try {
      setIsToggling(true);
      const response = await taskService.toggleTaskCompletion(user.id, task.id);

      // Trigger confetti if task is being marked as completed
      if (!task.completed && response.completed) {
        triggerConfetti();
      }

      setTask(response);
    } catch (err: any) {
      console.error('Error toggling task completion:', err);
      setError(err.message || 'Failed to update task');
    } finally {
      setIsToggling(false);
    }
  };

  const handleDelete = async () => {
    if (!task || !user?.id) return;

    if (window.confirm('Are you sure you want to delete this task?')) {
      try {
        await taskService.deleteTask(user.id, task.id);
        router.push('/dashboard/tasks');
      } catch (err: any) {
        console.error('Error deleting task:', err);
        setError(err.message || 'Failed to delete task');
      }
    }
  };

  if (loading) {
    return (
      <ProtectedRoute>
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          className="max-w-4xl mx-auto px-4 py-8"
        >
          <div className="flex justify-center items-center h-64">
            <Spinner size="lg" label="Loading task details..." />
          </div>
        </motion.div>
      </ProtectedRoute>
    );
  }

  if (error) {
    return (
      <ProtectedRoute>
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="max-w-2xl mx-auto px-4 py-8"
        >
          <ErrorMessage message={error} />
          <Button
            onClick={() => router.back()}
            className="mt-4"
          >
            <ChevronLeft className="w-4 h-4 mr-2" />
            Go Back
          </Button>
        </motion.div>
      </ProtectedRoute>
    );
  }

  if (!task) {
    return (
      <ProtectedRoute>
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          className="max-w-2xl mx-auto text-center py-12 px-4"
        >
          <div className="glass-strong rounded-3xl p-12">
            <h2 className="text-2xl font-bold mb-4 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent">
              Task not found
            </h2>
            <p className="text-gray-600 dark:text-gray-400 mb-6">
              The task you're looking for doesn't exist or has been deleted.
            </p>
            <Button onClick={() => router.push('/dashboard/tasks')}>
              <ChevronLeft className="w-4 h-4 mr-2" />
              Back to Tasks
            </Button>
          </div>
        </motion.div>
      </ProtectedRoute>
    );
  }

  return (
    <ProtectedRoute>
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        className="max-w-5xl mx-auto px-4 py-8"
      >
        {/* Breadcrumb Navigation */}
        <motion.div
          initial={{ opacity: 0, x: -20 }}
          animate={{ opacity: 1, x: 0 }}
          transition={{ delay: 0.1 }}
          className="mb-6"
        >
          <button
            onClick={() => router.push('/dashboard/tasks')}
            className="flex items-center gap-2 text-sm font-medium text-gray-600 dark:text-gray-400 hover:text-purple-600 dark:hover:text-purple-400 transition-colors"
          >
            <ChevronLeft className="w-4 h-4" />
            Back to Tasks
          </button>
        </motion.div>

        {/* Main Content */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2 }}
        >
          {/* Gradient Header */}
          <div className="relative overflow-hidden rounded-3xl bg-gradient-to-br from-purple-500 via-pink-500 to-cyan-500 p-1 mb-8 shadow-2xl shadow-purple-500/50">
            <div className="relative bg-white dark:bg-slate-900 rounded-3xl p-8">
              {/* Decorative gradient orbs */}
              <div className="absolute top-0 right-0 w-64 h-64 bg-gradient-to-br from-purple-500/20 to-pink-500/20 rounded-full blur-3xl -mr-32 -mt-32"></div>
              <div className="absolute bottom-0 left-0 w-64 h-64 bg-gradient-to-tr from-cyan-500/20 to-purple-500/20 rounded-full blur-3xl -ml-32 -mb-32"></div>

              <div className="relative z-10">
                <div className="flex items-start justify-between mb-4">
                  <div className="flex-1">
                    <motion.h1
                      initial={{ opacity: 0, y: 10 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ delay: 0.3 }}
                      className={`text-4xl font-extrabold mb-3 ${
                        task.completed
                          ? 'line-through text-gray-500 dark:text-gray-600'
                          : 'bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent'
                      }`}
                    >
                      {task.title}
                    </motion.h1>
                    {task.description && (
                      <motion.p
                        initial={{ opacity: 0 }}
                        animate={{ opacity: 1 }}
                        transition={{ delay: 0.4 }}
                        className="text-lg text-gray-700 dark:text-gray-300"
                      >
                        {task.description}
                      </motion.p>
                    )}
                  </div>
                  <motion.div
                    initial={{ opacity: 0, scale: 0.8 }}
                    animate={{ opacity: 1, scale: 1 }}
                    transition={{ delay: 0.3, type: 'spring' }}
                  >
                    <Badge
                      className={`text-base px-6 py-2 font-bold ${
                        task.completed
                          ? 'bg-gradient-to-r from-green-500 to-emerald-500 text-white shadow-lg shadow-green-500/50'
                          : 'bg-gradient-to-r from-yellow-500 to-orange-500 text-white shadow-lg shadow-yellow-500/50'
                      }`}
                    >
                      {task.completed ? (
                        <span className="flex items-center gap-2">
                          <Check className="w-5 h-5" />
                          Completed
                        </span>
                      ) : (
                        <span className="flex items-center gap-2">
                          <Clock className="w-5 h-5" />
                          Pending
                        </span>
                      )}
                    </Badge>
                  </motion.div>
                </div>
              </div>
            </div>
          </div>

          {/* Status Cards Grid */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.4 }}
            className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-8"
          >
            {/* Status Card */}
            <motion.div
              whileHover={{ scale: 1.02, y: -5 }}
              className="glass-strong rounded-2xl p-6 border-2 border-white/30 dark:border-white/20 shadow-xl"
            >
              <div className="flex items-center gap-3 mb-2">
                {task.completed ? (
                  <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-green-500 to-emerald-500 flex items-center justify-center shadow-lg shadow-green-500/50">
                    <Check className="w-6 h-6 text-white" />
                  </div>
                ) : (
                  <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-yellow-500 to-orange-500 flex items-center justify-center shadow-lg shadow-yellow-500/50">
                    <Clock className="w-6 h-6 text-white" />
                  </div>
                )}
                <div>
                  <h3 className="text-sm font-medium text-gray-500 dark:text-gray-400">Status</h3>
                  <p className={`text-xl font-bold ${task.completed ? 'text-green-600 dark:text-green-400' : 'text-yellow-600 dark:text-yellow-400'}`}>
                    {task.completed ? 'Completed' : 'Pending'}
                  </p>
                </div>
              </div>
            </motion.div>

            {/* Created Date Card */}
            <motion.div
              whileHover={{ scale: 1.02, y: -5 }}
              className="glass-strong rounded-2xl p-6 border-2 border-white/30 dark:border-white/20 shadow-xl"
            >
              <div className="flex items-center gap-3 mb-2">
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-purple-500 to-pink-500 flex items-center justify-center shadow-lg shadow-purple-500/50">
                  <Calendar className="w-6 h-6 text-white" />
                </div>
                <div>
                  <h3 className="text-sm font-medium text-gray-500 dark:text-gray-400">Created</h3>
                  <p className="text-xl font-bold text-gray-900 dark:text-gray-100">
                    {new Date(task.created_at).toLocaleDateString()}
                  </p>
                </div>
              </div>
            </motion.div>

            {/* Updated Date Card */}
            <motion.div
              whileHover={{ scale: 1.02, y: -5 }}
              className="glass-strong rounded-2xl p-6 border-2 border-white/30 dark:border-white/20 shadow-xl"
            >
              <div className="flex items-center gap-3 mb-2">
                <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-cyan-500 to-blue-500 flex items-center justify-center shadow-lg shadow-cyan-500/50">
                  <RefreshCw className="w-6 h-6 text-white" />
                </div>
                <div>
                  <h3 className="text-sm font-medium text-gray-500 dark:text-gray-400">Updated</h3>
                  <p className="text-xl font-bold text-gray-900 dark:text-gray-100">
                    {new Date(task.updated_at).toLocaleDateString()}
                  </p>
                </div>
              </div>
            </motion.div>
          </motion.div>

          {/* Description Section */}
          {task.description && (
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.5 }}
              className="glass-strong rounded-3xl p-8 mb-8 border-2 border-white/30 dark:border-white/20 shadow-2xl"
            >
              <h3 className="text-2xl font-bold mb-4 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent">
                Description
              </h3>
              <div className="prose max-w-none">
                <p className="text-gray-700 dark:text-gray-300 text-lg whitespace-pre-wrap leading-relaxed">
                  {task.description}
                </p>
              </div>
            </motion.div>
          )}

          {/* Action Buttons */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.6 }}
            className="flex flex-col sm:flex-row gap-4"
          >
            <motion.button
              whileHover={{ scale: 1.02, y: -2 }}
              whileTap={{ scale: 0.98 }}
              onClick={handleToggleCompletion}
              disabled={isToggling}
              className={`flex-1 relative overflow-hidden px-8 py-4 rounded-2xl font-bold text-white shadow-2xl transition-all ${
                task.completed
                  ? 'bg-gradient-to-r from-yellow-500 to-orange-500 hover:from-yellow-400 hover:to-orange-400 shadow-yellow-500/50'
                  : 'bg-gradient-to-r from-green-500 to-emerald-500 hover:from-green-400 hover:to-emerald-400 shadow-green-500/50'
              }`}
            >
              <span className="relative z-10 flex items-center justify-center gap-2">
                {isToggling ? (
                  <>
                    <RefreshCw className="w-5 h-5 animate-spin" />
                    Updating...
                  </>
                ) : task.completed ? (
                  <>
                    <Clock className="w-5 h-5" />
                    Mark as Pending
                  </>
                ) : (
                  <>
                    <Check className="w-5 h-5" />
                    Mark as Complete
                  </>
                )}
              </span>
              <div className="absolute inset-0 bg-gradient-to-r from-white/0 via-white/20 to-white/0 translate-x-[-200%] group-hover:translate-x-[200%] transition-transform duration-700"></div>
            </motion.button>

            <motion.button
              whileHover={{ scale: 1.02, y: -2 }}
              whileTap={{ scale: 0.98 }}
              onClick={handleDelete}
              className="relative overflow-hidden px-8 py-4 rounded-2xl font-bold text-white bg-gradient-to-r from-red-500 to-pink-500 hover:from-red-400 hover:to-pink-400 shadow-2xl shadow-red-500/50 transition-all"
            >
              <span className="relative z-10 flex items-center justify-center gap-2">
                <Trash2 className="w-5 h-5" />
                Delete Task
              </span>
            </motion.button>
          </motion.div>
        </motion.div>
      </motion.div>
    </ProtectedRoute>
  );
};

export default TaskDetailPage;
