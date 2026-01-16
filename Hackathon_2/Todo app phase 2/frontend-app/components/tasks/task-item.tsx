'use client';

import React, { useState } from 'react';
import Link from 'next/link';
import { Task } from '../../types';
import { Button } from '../ui/button';
import { Card, CardContent, CardFooter } from '../ui/card';

interface TaskItemProps {
  task: Task;
  onToggle?: (task: Task) => void;
  onDelete?: (taskId: string) => void;
}

export const TaskItem: React.FC<TaskItemProps> = ({ task, onToggle, onDelete }) => {
  const [isDeleting, setIsDeleting] = useState(false);

  const handleToggle = () => {
    onToggle?.(task);
  };

  const handleDelete = async () => {
    if (!onDelete) return;

    setIsDeleting(true);
    try {
      onDelete(task.id);
    } catch (error) {
      console.error('Error deleting task:', error);
      setIsDeleting(false);
    }
  };

  return (
    <Card className={task.completed ? 'border-green-200 bg-green-50' : ''}>
      <CardContent className="pt-6">
        <div className="flex items-start">
          <input
            type="checkbox"
            checked={task.completed}
            onChange={handleToggle}
            className="h-5 w-5 rounded border-gray-300 text-blue-600 focus:ring-blue-500 mt-1"
          />
          <div className="ml-3 flex-1">
            <h3 className={`text-lg font-medium ${task.completed ? 'line-through text-gray-500' : 'text-gray-900'}`}>
              {task.title}
            </h3>
            {task.description && (
              <p className={`mt-1 text-sm ${task.completed ? 'text-gray-400' : 'text-gray-500'}`}>
                {task.description}
              </p>
            )}
            <div className="mt-2 flex items-center text-xs text-gray-500">
              <span>Created: {new Date(task.created_at).toLocaleDateString()}</span>
              {task.updated_at !== task.created_at && (
                <span className="ml-2">
                  Updated: {new Date(task.updated_at).toLocaleDateString()}
                </span>
              )}
            </div>
          </div>
        </div>
      </CardContent>
      <CardFooter className="flex justify-between">
        <div>
          <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
            task.completed
              ? 'bg-green-100 text-green-800'
              : 'bg-yellow-100 text-yellow-800'
          }`}>
            {task.completed ? 'Completed' : 'Pending'}
          </span>
        </div>
        <div className="flex space-x-2">
          <Link href={`/dashboard/tasks/${task.id}`}>
            <Button variant="outline" size="sm">
              View
            </Button>
          </Link>
          <Button
            variant="destructive"
            size="sm"
            onClick={handleDelete}
            disabled={isDeleting}
          >
            {isDeleting ? 'Deleting...' : 'Delete'}
          </Button>
        </div>
      </CardFooter>
    </Card>
  );
};