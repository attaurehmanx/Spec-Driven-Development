'use client';

import React from 'react';
import { Task } from '../../../types';
import { Card, CardContent, CardHeader, CardTitle } from '../ui/card';
import { Button } from '../ui/button';

interface TaskDetailProps {
  task: Task;
  onToggleCompletion: () => void;
  onDelete: () => void;
  onBack: () => void;
}

export const TaskDetail: React.FC<TaskDetailProps> = ({
  task,
  onToggleCompletion,
  onDelete,
  onBack,
}) => {
  return (
    <Card>
      <CardHeader>
        <div className="flex justify-between items-start">
          <CardTitle className={task.completed ? 'line-through text-gray-500' : ''}>
            {task.title}
          </CardTitle>
          <Button
            variant={task.completed ? 'secondary' : 'default'}
            onClick={onToggleCompletion}
          >
            {task.completed ? 'Mark as Pending' : 'Mark as Complete'}
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        {task.description && (
          <div className="mb-6">
            <h3 className="text-sm font-medium text-gray-500 mb-1">Description</h3>
            <p className="text-gray-700">{task.description}</p>
          </div>
        )}

        <div className="grid grid-cols-2 gap-4 mb-6">
          <div>
            <h3 className="text-sm font-medium text-gray-500">Created</h3>
            <p className="text-gray-700">
              {new Date(task.created_at).toLocaleString()}
            </p>
          </div>
          <div>
            <h3 className="text-sm font-medium text-gray-500">Updated</h3>
            <p className="text-gray-700">
              {new Date(task.updated_at).toLocaleString()}
            </p>
          </div>
        </div>

        <div className="flex space-x-4">
          <Button
            variant="destructive"
            onClick={onDelete}
          >
            Delete Task
          </Button>
          <Button
            variant="secondary"
            onClick={onBack}
          >
            Back to Tasks
          </Button>
        </div>
      </CardContent>
    </Card>
  );
};