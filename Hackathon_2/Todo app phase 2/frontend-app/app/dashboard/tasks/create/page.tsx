'use client';

import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
import ProtectedRoute from '../../../../components/auth/protected-route';
import { Card, CardContent, CardHeader, CardTitle } from '../../../../components/ui/card';
import { Button } from '../../../../components/ui/button';
import Input from '../../../../components/ui/input';
import ErrorMessage from '../../../../components/ui/error-message';
import SuccessMessage from '../../../../components/ui/success-message';
import useAuth from '../../../../hooks/use-auth';
import { taskService } from '../../../../lib/api/task-service';

const TaskCreationPage: React.FC = () => {
  const [formData, setFormData] = useState({
    title: '',
    description: '',
    completed: false,
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  const router = useRouter();
  const { user } = useAuth();

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;
    const val = type === 'checkbox' ? (e.target as HTMLInputElement).checked : value;

    setFormData(prev => ({ ...prev, [name]: val }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!user?.id) {
      setError('User not authenticated');
      return;
    }

    setLoading(true);
    setError(null);
    setSuccess(null);

    try {
      await taskService.createTask(user.id, formData);

      setSuccess('Task created successfully!');
      // Reset form
      setFormData({
        title: '',
        description: '',
        completed: false,
      });

      // Redirect after a short delay
      setTimeout(() => {
        router.push('/dashboard/tasks');
      }, 1500);
    } catch (err: any) {
      console.error('Error creating task:', err);
      setError(err.message || 'Failed to create task');
    } finally {
      setLoading(false);
    }
  };

  return (
    <ProtectedRoute>
      <div className="max-w-2xl mx-auto">
        <Card>
          <CardHeader>
            <CardTitle>Create New Task</CardTitle>
          </CardHeader>
          <CardContent>
            <form onSubmit={handleSubmit} className="space-y-4">
              {error && <ErrorMessage message={error} />}
              {success && <SuccessMessage message={success} />}

              <Input
                label="Title"
                type="text"
                name="title"
                value={formData.title}
                onChange={handleChange}
                required
                fullWidth
              />

              <div className="space-y-2">
                <label htmlFor="description" className="text-sm font-medium leading-none peer-disabled:cursor-not-allowed peer-disabled:opacity-70">
                  Description
                </label>
                <textarea
                  id="description"
                  name="description"
                  value={formData.description}
                  onChange={handleChange}
                  rows={4}
                  className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background file:border-0 file:bg-transparent file:text-sm file:font-medium placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:opacity-50 w-full"
                />
              </div>

              <div className="flex items-center">
                <input
                  type="checkbox"
                  id="completed"
                  name="completed"
                  checked={formData.completed}
                  onChange={handleChange}
                  className="h-4 w-4 rounded border-gray-300 text-blue-600 focus:ring-blue-500"
                />
                <label htmlFor="completed" className="ml-2 block text-sm text-gray-900">
                  Mark as completed
                </label>
              </div>

              <div className="flex space-x-4">
                <Button
                  type="submit"
                  isLoading={loading}
                  disabled={loading}
                >
                  {loading ? 'Creating...' : 'Create Task'}
                </Button>

                <Button
                  type="button"
                  variant="secondary"
                  onClick={() => router.push('/dashboard/tasks')}
                  disabled={loading}
                >
                  Cancel
                </Button>
              </div>
            </form>
          </CardContent>
        </Card>
      </div>
    </ProtectedRoute>
  );
};

export default TaskCreationPage;