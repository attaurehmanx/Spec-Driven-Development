'use client';

import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
import { Button } from '../ui/button';
import Input from '../ui/input';
import ErrorMessage from '../ui/error-message';
import SuccessMessage from '../ui/success-message';
import { Task } from '../../types';
import useAuth from '../../hooks/use-auth';
import { taskService } from '../../lib/api/task-service';

interface TaskUpdateFormProps {
  task: Task;
  onSuccess?: (updatedTask: Task) => void;
  onError?: (error: string) => void;
  onCancel?: () => void;
}

const TaskUpdateForm: React.FC<TaskUpdateFormProps> = ({ task, onSuccess, onError, onCancel }) => {
  const [formData, setFormData] = useState({
    title: task.title || '',
    description: task.description || '',
    completed: task.completed || false,
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

    if (!user?.id || !task.id) {
      const errorMsg = 'User not authenticated or task ID missing';
      setError(errorMsg);
      onError?.(errorMsg);
      return;
    }

    setLoading(true);
    setError(null);
    setSuccess(null);

    try {
      const response = await taskService.updateTask(user.id, task.id, formData);
      setSuccess('Task updated successfully!');

      // Call success callback with the updated task
      onSuccess?.(response);

      // Reset form with new values
      setFormData({
        title: response.title,
        description: response.description || '',
        completed: response.completed,
      });
    } catch (err: any) {
      console.error('Error updating task:', err);
      const errorMsg = err.message || 'Failed to update task';
      setError(errorMsg);
      onError?.(errorMsg);
    } finally {
      setLoading(false);
    }
  };

  return (
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
          className="flex h-20 w-full rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background file:border-0 file:bg-transparent file:text-sm file:font-medium placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:opacity-50 w-full"
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
          {loading ? 'Updating...' : 'Update Task'}
        </Button>

        {onCancel && (
          <Button
            type="button"
            variant="secondary"
            onClick={onCancel}
            disabled={loading}
          >
            Cancel
          </Button>
        )}
      </div>
    </form>
  );
};

export default TaskUpdateForm;