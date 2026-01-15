'use client';

import React, { useState, useEffect } from 'react';
import { useParams, useRouter } from 'next/navigation';
import ProtectedRoute from '@/components/auth/protected-route';
import { Card, CardContent, CardHeader, CardTitle, CardDescription, CardFooter } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import LoadingSpinner from '@/components/ui/loading-spinner';
import ErrorMessage from '@/components/ui/error-message';
import { Badge } from '@/components/ui/badge';
import { Task } from '@/types';
import useAuth from '@/hooks/use-auth';
import { taskService } from '@/lib/api/task-service';

const TaskDetailPage: React.FC = () => {
  const { id } = useParams<{ id: string }>();
  const router = useRouter();
  const { user } = useAuth();

  const [task, setTask] = useState<Task | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

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
    if (!task || !user?.id) return;

    try {
      const response = await taskService.toggleTaskCompletion(user.id, task.id);
      setTask(response);
    } catch (err: any) {
      console.error('Error toggling task completion:', err);
      setError(err.message || 'Failed to update task');
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
        <div className="flex justify-center items-center h-64">
          <LoadingSpinner label="Loading task..." />
        </div>
      </ProtectedRoute>
    );
  }

  if (error) {
    return (
      <ProtectedRoute>
        <div className="max-w-2xl mx-auto">
          <ErrorMessage message={error} />
          <Button
            onClick={() => router.back()}
            className="mt-4"
          >
            Go Back
          </Button>
        </div>
      </ProtectedRoute>
    );
  }

  if (!task) {
    return (
      <ProtectedRoute>
        <div className="max-w-2xl mx-auto text-center py-12">
          <h2 className="text-xl font-semibold mb-2">Task not found</h2>
          <Button
            onClick={() => router.back()}
          >
            Go Back
          </Button>
        </div>
      </ProtectedRoute>
    );
  }

  return (
    <ProtectedRoute>
      <div className="max-w-3xl mx-auto">
        <Card className="shadow-lg">
          <CardHeader className="border-b border-gray-200">
            <div className="flex justify-between items-start">
              <div className="flex-1">
                <CardTitle className={`text-2xl ${task.completed ? 'line-through text-gray-500' : 'text-gray-900'}`}>
                  {task.title}
                </CardTitle>
                <CardDescription className="mt-2">
                  {task.description || 'No description provided'}
                </CardDescription>
              </div>
              <Badge
                variant={task.completed ? 'default' : 'secondary'}
                className={`${task.completed ? 'bg-green-100 text-green-800' : 'bg-yellow-100 text-yellow-800'}`}
              >
                {task.completed ? 'Completed' : 'Pending'}
              </Badge>
            </div>
          </CardHeader>

          <CardContent className="py-6">
            <div className="space-y-6">
              {/* Status Section */}
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="bg-gray-50 p-4 rounded-lg">
                  <h3 className="text-sm font-medium text-gray-500 mb-1">Status</h3>
                  <p className={`font-semibold ${task.completed ? 'text-green-600' : 'text-yellow-600'}`}>
                    {task.completed ? 'Completed' : 'Pending'}
                  </p>
                </div>

                <div className="bg-gray-50 p-4 rounded-lg">
                  <h3 className="text-sm font-medium text-gray-500 mb-1">Created</h3>
                  <p className="text-gray-900">
                    {new Date(task.created_at).toLocaleDateString()}
                  </p>
                </div>

                <div className="bg-gray-50 p-4 rounded-lg">
                  <h3 className="text-sm font-medium text-gray-500 mb-1">Updated</h3>
                  <p className="text-gray-900">
                    {new Date(task.updated_at).toLocaleDateString()}
                  </p>
                </div>
              </div>

              {/* Description Section */}
              {task.description && (
                <div className="space-y-2">
                  <h3 className="text-lg font-medium text-gray-900">Description</h3>
                  <div className="prose max-w-none bg-white p-4 rounded-lg border border-gray-200">
                    <p className="text-gray-700 whitespace-pre-wrap">{task.description}</p>
                  </div>
                </div>
              )}

              {/* Created/Updated Details */}
              <div className="space-y-2">
                <h3 className="text-lg font-medium text-gray-900">Details</h3>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  <div>
                    <h4 className="text-sm font-medium text-gray-500">Created At</h4>
                    <p className="text-gray-900">
                      {new Date(task.created_at).toLocaleString()}
                    </p>
                  </div>
                  <div>
                    <h4 className="text-sm font-medium text-gray-500">Last Updated</h4>
                    <p className="text-gray-900">
                      {new Date(task.updated_at).toLocaleString()}
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </CardContent>

          <CardFooter className="flex flex-col sm:flex-row justify-between items-center pt-6 border-t border-gray-200 space-y-4 sm:space-y-0">
            <div className="flex space-x-3">
              <Button
                variant={task.completed ? 'outline' : 'default'}
                onClick={handleToggleCompletion}
              >
                {task.completed ? 'Mark as Pending' : 'Mark as Complete'}
              </Button>
              <Button
                variant="destructive"
                onClick={handleDelete}
              >
                Delete Task
              </Button>
            </div>
            <Button
              variant="outline"
              onClick={() => router.push('/dashboard/tasks')}
            >
              Back to Tasks
            </Button>
          </CardFooter>
        </Card>
      </div>
    </ProtectedRoute>
  );
};

export default TaskDetailPage;