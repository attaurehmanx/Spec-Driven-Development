'use client';

import React, { useState, useEffect } from 'react';
import Link from 'next/link';
import { useRouter } from 'next/navigation';
import ProtectedRoute from '@/components/auth/protected-route';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import LoadingSpinner from '@/components/ui/loading-spinner';
import ErrorMessage from '@/components/ui/error-message';
import { Badge } from '@/components/ui/badge';
import { Task } from '@/types';
import useAuth from '@/hooks/use-auth';
import { taskService } from '@/lib/api/task-service';
import { useTaskRefresh } from '@/hooks/use-task-refresh';

const TaskListPage: React.FC = () => {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [filter, setFilter] = useState<'all' | 'completed' | 'pending'>('all');

  const router = useRouter();
  const { user } = useAuth();

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
      <div className="space-y-6">
        <div className="flex flex-col md:flex-row md:justify-between md:items-center gap-4">
          <div>
            <h1 className="text-3xl font-bold">My Tasks</h1>
            <p className="text-gray-600 mt-1">
              Manage your tasks efficiently
            </p>
          </div>

          <div className="flex flex-col sm:flex-row gap-3">
            <div className="flex gap-1 p-1 bg-gray-100 rounded-lg">
              <button
                className={`px-3 py-1 text-sm rounded-md ${
                  filter === 'all'
                    ? 'bg-white shadow-sm text-gray-900'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
                onClick={() => setFilter('all')}
              >
                All ({tasks.length})
              </button>
              <button
                className={`px-3 py-1 text-sm rounded-md ${
                  filter === 'pending'
                    ? 'bg-white shadow-sm text-gray-900'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
                onClick={() => setFilter('pending')}
              >
                Pending ({pendingCount})
              </button>
              <button
                className={`px-3 py-1 text-sm rounded-md ${
                  filter === 'completed'
                    ? 'bg-white shadow-sm text-gray-900'
                    : 'text-gray-600 hover:text-gray-900'
                }`}
                onClick={() => setFilter('completed')}
              >
                Completed ({completedCount})
              </button>
            </div>

            <Button onClick={() => router.push('/dashboard/tasks/create')}>
              Create New Task
            </Button>
          </div>
        </div>

        {error && <ErrorMessage message={error} />}

        {filteredTasks.length === 0 ? (
          <Card className="border-2 border-dashed border-gray-300">
            <CardContent className="flex flex-col items-center justify-center py-16">
              <div className="text-center">
                <h3 className="text-lg font-medium text-gray-900 mb-1">
                  {filter === 'all'
                    ? 'No tasks yet'
                    : filter === 'pending'
                      ? 'No pending tasks'
                      : 'No completed tasks'}
                </h3>
                <p className="text-gray-500 mb-6">
                  {filter === 'all'
                    ? 'Get started by creating your first task'
                    : filter === 'pending'
                      ? 'Great job! All tasks are completed.'
                      : 'Start working on some tasks!'}
                </p>

                <Button
                  onClick={() => {
                    if (filter === 'completed') {
                      setFilter('all');
                    } else {
                      router.push('/dashboard/tasks/create');
                    }
                  }}
                >
                  {filter === 'completed' ? 'View All Tasks' : 'Create Task'}
                </Button>
              </div>
            </CardContent>
          </Card>
        ) : (
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {filteredTasks.map((task) => (
              <Card
                key={task.id}
                className={`overflow-hidden transition-all duration-300 hover:shadow-lg ${
                  task.completed
                    ? 'border-green-200 bg-green-50/30'
                    : 'border-yellow-200 bg-yellow-50/30'
                }`}
              >
                <div className="p-5">
                  <div className="flex justify-between items-start mb-3">
                    <div className="flex-1 min-w-0">
                      <h3 className={`font-semibold truncate ${
                        task.completed
                          ? 'line-through text-gray-500'
                          : 'text-gray-900'
                      }`}>
                        {task.title}
                      </h3>

                      <div className="flex items-center mt-2 space-x-2">
                        <Badge
                          variant={task.completed ? 'default' : 'secondary'}
                          className={`text-xs ${
                            task.completed
                              ? 'bg-green-100 text-green-800'
                              : 'bg-yellow-100 text-yellow-800'
                          }`}
                        >
                          {task.completed ? 'Completed' : 'Pending'}
                        </Badge>

                        <span className="text-xs text-gray-500">
                          Updated: {new Date(task.updated_at).toLocaleDateString()}
                        </span>
                      </div>
                    </div>

                    <div className="ml-3">
                      <div className={`w-3 h-3 rounded-full ${
                        task.completed ? 'bg-green-500' : 'bg-yellow-500'
                      }`}></div>
                    </div>
                  </div>

                  {task.description && (
                    <p className="text-gray-600 text-sm mb-4 line-clamp-2">
                      {task.description.substring(0, 100)}{task.description.length > 100 ? '...' : ''}
                    </p>
                  )}

                  <Button
                    variant="outline"
                    size="sm"
                    className="w-full"
                    onClick={() => router.push(`/dashboard/tasks/${task.id}`)}
                  >
                    View Details
                  </Button>
                </div>
              </Card>
            ))}
          </div>
        )}

        {/* Show total tasks count when filtered */}
        {filter !== 'all' && filteredTasks.length > 0 && (
          <div className="text-center text-sm text-gray-500">
            Showing {filteredTasks.length} of {tasks.length} tasks
          </div>
        )}
      </div>
    </ProtectedRoute>
  );
};

export default TaskListPage;