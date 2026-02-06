'use client';

import React, { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { motion } from 'framer-motion';
import { Plus, Edit, Trash2, CheckCircle2, Circle } from 'lucide-react';
import ProtectedRoute from '@/components/auth/protected-route';
import { Button } from '@/components/ui/button';
import LoadingSpinner from '@/components/ui/loading-spinner';
import ErrorMessage from '@/components/ui/error-message';
import { Task } from '@/types';
import useAuth from '@/hooks/use-auth';
import { taskService } from '@/lib/api/task-service';
import { useTaskRefresh } from '@/hooks/use-task-refresh';
import { EmptyStateIllustration } from '@/components/ui/empty-state-illustration';
import { TaskModal } from '@/components/tasks/task-modal';
import { DeleteConfirmationModal } from '@/components/tasks/delete-confirmation-modal';
import { ToastContainer } from '@/components/ui/toast';

const TaskListPage: React.FC = () => {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [filter, setFilter] = useState<'all' | 'completed' | 'pending'>('all');

  // Modal states
  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false);
  const [isEditModalOpen, setIsEditModalOpen] = useState(false);
  const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
  const [selectedTask, setSelectedTask] = useState<Task | null>(null);

  // Toast states
  const [toasts, setToasts] = useState<Array<{ id: string; type: 'success' | 'error' | 'warning' | 'info'; message: string }>>([]);

  const router = useRouter();
  const { user } = useAuth();

  // Toast helper functions
  const addToast = (type: 'success' | 'error' | 'warning' | 'info', message: string) => {
    const id = Date.now().toString();
    setToasts((prev) => [...prev, { id, type, message }]);
  };

  const removeToast = (id: string) => {
    setToasts((prev) => prev.filter((toast) => toast.id !== id));
  };

  // Fetch tasks function
  const fetchTasks = async () => {
    if (!user?.id) return;

    try {
      setLoading(true);
      setError(null);

      const response = await taskService.getUserTasks(user.id);
      setTasks(response.tasks);
    } catch (err: any) {
      console.error('Error fetching tasks:', err);
      setError(err.message || 'Failed to load tasks');
      addToast('error', 'Failed to load tasks');
    } finally {
      setLoading(false);
    }
  };

  // Initial fetch on mount
  useEffect(() => {
    fetchTasks();
  }, [user]);

  // Listen for task updates from chat interface
  useTaskRefresh(fetchTasks);

  // Filter tasks based on selected filter
  const filteredTasks = tasks.filter(task => {
    if (filter === 'completed') return task.completed;
    if (filter === 'pending') return !task.completed;
    return true;
  });

  const completedCount = tasks.filter(task => task.completed).length;
  const pendingCount = tasks.filter(task => !task.completed).length;

  // Handle create task
  const handleCreateTask = async (data: { title: string; description: string; completed: boolean }) => {
    if (!user?.id) return;

    try {
      await taskService.createTask(user.id, data);
      await fetchTasks();
      addToast('success', 'Task created successfully!');
    } catch (err: any) {
      console.error('Error creating task:', err);
      addToast('error', err.message || 'Failed to create task');
      throw err;
    }
  };

  // Handle edit task
  const handleEditTask = async (data: { title: string; description: string; completed: boolean }) => {
    if (!user?.id || !selectedTask) return;

    try {
      await taskService.updateTask(user.id, selectedTask.id, data);
      await fetchTasks();
      addToast('success', 'Task updated successfully!');
    } catch (err: any) {
      console.error('Error updating task:', err);
      addToast('error', err.message || 'Failed to update task');
      throw err;
    }
  };

  // Handle delete task
  const handleDeleteTask = async () => {
    if (!user?.id || !selectedTask) return;

    try {
      await taskService.deleteTask(user.id, selectedTask.id);
      await fetchTasks();
      addToast('success', 'Task deleted successfully!');
    } catch (err: any) {
      console.error('Error deleting task:', err);
      addToast('error', err.message || 'Failed to delete task');
      throw err;
    }
  };

  // Handle toggle completion
  const handleToggleComplete = async (task: Task) => {
    if (!user?.id) return;

    try {
      await taskService.updateTask(user.id, task.id, {
        title: task.title,
        description: task.description || '',
        completed: !task.completed,
      });
      await fetchTasks();
      addToast('success', task.completed ? 'Task marked as pending' : 'Task completed!');
    } catch (err: any) {
      console.error('Error toggling task:', err);
      addToast('error', 'Failed to update task');
    }
  };

  if (loading) {
    return (
      <ProtectedRoute>
        <div className="flex justify-center items-center h-64">
          <LoadingSpinner label="Loading tasks..." />
        </div>
      </ProtectedRoute>
    );
  }

  return (
    <ProtectedRoute>
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        className="space-y-3 pb-24 md:pb-6"
      >
        {/* Header */}
        <motion.div
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          className="flex flex-col md:flex-row md:justify-between md:items-center gap-3"
        >
          <div>
            <h1 className="text-3xl font-extrabold bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent">
              My Tasks
            </h1>
            <p className="text-gray-600 dark:text-gray-400 mt-0.5">
              Manage your tasks efficiently
            </p>
          </div>

          <div className="flex flex-col sm:flex-row gap-2">
            {/* Filter Buttons */}
            <div className="flex gap-1 p-1 glass rounded-xl">
              <button
                className={`px-4 py-2 text-sm font-bold rounded-lg transition-all duration-200 ${
                  filter === 'all'
                    ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50'
                    : 'text-gray-600 dark:text-gray-400 hover:bg-white/50 dark:hover:bg-slate-800/50'
                }`}
                onClick={() => setFilter('all')}
              >
                All ({tasks.length})
              </button>
              <button
                className={`px-4 py-2 text-sm font-bold rounded-lg transition-all duration-200 ${
                  filter === 'pending'
                    ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50'
                    : 'text-gray-600 dark:text-gray-400 hover:bg-white/50 dark:hover:bg-slate-800/50'
                }`}
                onClick={() => setFilter('pending')}
              >
                Pending ({pendingCount})
              </button>
              <button
                className={`px-4 py-2 text-sm font-bold rounded-lg transition-all duration-200 ${
                  filter === 'completed'
                    ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50'
                    : 'text-gray-600 dark:text-gray-400 hover:bg-white/50 dark:hover:bg-slate-800/50'
                }`}
                onClick={() => setFilter('completed')}
              >
                Completed ({completedCount})
              </button>
            </div>

            {/* Create Button */}
            <button
              onClick={() => setIsCreateModalOpen(true)}
              className="inline-flex items-center justify-center gap-2 px-5 py-2.5 rounded-xl font-bold text-white bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-500 hover:to-pink-500 shadow-lg shadow-purple-500/50 hover:shadow-xl hover:shadow-purple-500/60 transition-all duration-300 hover:scale-105 hover:-translate-y-1"
            >
              <Plus className="w-4 h-4" />
              Create Task
            </button>
          </div>
        </motion.div>

        {error && <ErrorMessage message={error} />}

        {/* Empty State */}
        {filteredTasks.length === 0 ? (
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 0.3 }}
            className="relative overflow-hidden rounded-3xl glass-strong border-2 border-white/30 dark:border-white/20 shadow-2xl"
          >
            {/* Gradient background */}
            <div className="absolute inset-0 bg-gradient-to-br from-purple-500/10 via-pink-500/10 to-cyan-500/10"></div>

            <div className="relative flex flex-col items-center justify-center py-12 px-8">
              <EmptyStateIllustration />

              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.3 }}
                className="text-center mt-6"
              >
                <h3 className="text-xl font-bold text-gray-900 dark:text-gray-100 mb-1.5">
                  {filter === 'all'
                    ? 'No tasks yet'
                    : filter === 'pending'
                      ? 'No pending tasks'
                      : 'No completed tasks'}
                </h3>
                <p className="text-gray-600 dark:text-gray-400 mb-4 max-w-md">
                  {filter === 'all'
                    ? 'Get started by creating your first task and boost your productivity!'
                    : filter === 'pending'
                      ? 'Great job! All tasks are completed. Time to add new ones!'
                      : 'Start working on some tasks to see them here!'}
                </p>

                <Button
                  onClick={() => {
                    if (filter === 'completed') {
                      setFilter('all');
                    } else {
                      setIsCreateModalOpen(true);
                    }
                  }}
                  size="lg"
                  className="bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-500 hover:to-pink-500 text-white shadow-lg shadow-purple-500/50"
                >
                  {filter === 'completed' ? 'View All Tasks' : (
                    <>
                      <Plus className="w-5 h-5" />
                      Create Your First Task
                    </>
                  )}
                </Button>
              </motion.div>
            </div>
          </motion.div>
        ) : (
          /* Task Grid */
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.2 }}
            className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4"
          >
            {filteredTasks.map((task, index) => (
              <motion.div
                key={task.id}
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                whileHover={{ y: -5, scale: 1.02 }}
                className={`relative overflow-hidden rounded-2xl glass-strong border-2 shadow-xl transition-all duration-300 ${
                  task.completed
                    ? 'border-green-500/50 shadow-green-500/20'
                    : 'border-purple-500/50 shadow-purple-500/20'
                }`}
              >
                {/* Gradient background */}
                <div className={`absolute inset-0 ${
                  task.completed
                    ? 'bg-gradient-to-br from-green-500/10 to-emerald-500/10'
                    : 'bg-gradient-to-br from-purple-500/10 to-pink-500/10'
                }`}></div>

                <div className="relative p-4">
                  {/* Header with status indicator */}
                  <div className="flex items-start justify-between mb-3">
                    <button
                      onClick={() => handleToggleComplete(task)}
                      className="flex-shrink-0 mr-3 transition-transform hover:scale-110"
                    >
                      {task.completed ? (
                        <CheckCircle2 className="w-6 h-6 text-green-500" />
                      ) : (
                        <Circle className="w-6 h-6 text-gray-400 dark:text-gray-500" />
                      )}
                    </button>

                    <div className="flex-1 min-w-0">
                      <h3 className={`font-bold text-base mb-0.5 ${
                        task.completed
                          ? 'line-through text-gray-500 dark:text-gray-400'
                          : 'text-gray-900 dark:text-gray-100'
                      }`}>
                        {task.title}
                      </h3>

                      <span className={`inline-flex items-center px-2 py-0.5 rounded-full text-xs font-bold ${
                        task.completed
                          ? 'bg-green-100 dark:bg-green-900/30 text-green-800 dark:text-green-300'
                          : 'bg-purple-100 dark:bg-purple-900/30 text-purple-800 dark:text-purple-300'
                      }`}>
                        {task.completed ? 'Completed' : 'Pending'}
                      </span>
                    </div>
                  </div>

                  {/* Description */}
                  {task.description && (
                    <p className="text-gray-600 dark:text-gray-400 text-sm mb-3 line-clamp-2">
                      {task.description}
                    </p>
                  )}

                  {/* Metadata */}
                  <div className="flex items-center justify-between text-xs text-gray-500 dark:text-gray-400 mb-3">
                    <span>Updated: {new Date(task.updated_at).toLocaleDateString()}</span>
                  </div>

                  {/* Action Buttons */}
                  <div className="flex gap-2">
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => {
                        setSelectedTask(task);
                        setIsEditModalOpen(true);
                      }}
                      className="flex-1"
                    >
                      <Edit className="w-4 h-4" />
                      Edit
                    </Button>
                    <Button
                      variant="destructive"
                      size="sm"
                      onClick={() => {
                        setSelectedTask(task);
                        setIsDeleteModalOpen(true);
                      }}
                      className="flex-1"
                    >
                      <Trash2 className="w-4 h-4" />
                      Delete
                    </Button>
                  </div>
                </div>
              </motion.div>
            ))}
          </motion.div>
        )}

        {/* Show total tasks count when filtered */}
        {filter !== 'all' && filteredTasks.length > 0 && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            className="text-center text-sm text-gray-500 dark:text-gray-400"
          >
            Showing {filteredTasks.length} of {tasks.length} tasks
          </motion.div>
        )}
      </motion.div>

      {/* Modals */}
      <TaskModal
        isOpen={isCreateModalOpen}
        onClose={() => setIsCreateModalOpen(false)}
        onSubmit={handleCreateTask}
        mode="create"
      />

      <TaskModal
        isOpen={isEditModalOpen}
        onClose={() => {
          setIsEditModalOpen(false);
          setSelectedTask(null);
        }}
        onSubmit={handleEditTask}
        task={selectedTask}
        mode="edit"
      />

      <DeleteConfirmationModal
        isOpen={isDeleteModalOpen}
        onClose={() => {
          setIsDeleteModalOpen(false);
          setSelectedTask(null);
        }}
        onConfirm={handleDeleteTask}
        taskTitle={selectedTask?.title || ''}
      />

      {/* Toast Notifications */}
      <ToastContainer toasts={toasts} onClose={removeToast} />
    </ProtectedRoute>
  );
};

export default TaskListPage;
