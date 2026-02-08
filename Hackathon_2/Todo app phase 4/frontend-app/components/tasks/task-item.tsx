'use client';

import React, { useState } from 'react';
import Link from 'next/link';
import { motion } from 'framer-motion';
import { Task } from '../../types';
import { Button } from '../ui/button';
import { Card, CardContent, CardFooter } from '../ui/card';
import { triggerConfetti } from '@/lib/confetti';

interface TaskItemProps {
  task: Task;
  onToggle?: (task: Task) => void;
  onDelete?: (taskId: string) => void;
}

// Priority-based gradient variants
const getPriorityGradient = (priority?: string, completed?: boolean) => {
  if (completed) {
    return 'bg-gradient-to-br from-success-500/20 via-success-400/10 to-transparent dark:from-success-600/30 dark:via-success-500/20 border-success-400/40 dark:border-success-500/50 shadow-glow-green';
  }

  // Default to medium priority if not specified
  const taskPriority = priority?.toLowerCase() || 'medium';

  if (taskPriority === 'high' || taskPriority === 'urgent') {
    return 'bg-gradient-to-br from-red-500/20 via-secondary-500/15 to-transparent dark:from-red-600/30 dark:via-secondary-600/25 border-red-400/40 dark:border-red-500/50 shadow-glow-pink';
  } else if (taskPriority === 'low') {
    return 'bg-gradient-to-br from-accent-500/20 via-primary-400/10 to-transparent dark:from-accent-600/30 dark:via-primary-500/20 border-accent-400/40 dark:border-accent-500/50 shadow-glow-cyan';
  } else {
    return 'bg-gradient-to-br from-primary-500/20 via-secondary-400/10 to-transparent dark:from-primary-600/30 dark:via-secondary-500/20 border-primary-400/40 dark:border-primary-500/50 shadow-glow';
  }
};

export const TaskItem: React.FC<TaskItemProps> = ({ task, onToggle, onDelete }) => {
  const [isDeleting, setIsDeleting] = useState(false);
  const [isHovered, setIsHovered] = useState(false);

  const handleToggle = () => {
    // Trigger confetti if task is being marked as completed
    if (!task.completed) {
      triggerConfetti();
    }
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

  const gradientClass = getPriorityGradient((task as any).priority, task.completed);

  return (
    <motion.div
      className={`relative overflow-hidden rounded-2xl ${gradientClass} border-2 backdrop-blur-sm transition-all duration-300`}
      initial={{ opacity: 0, scale: 0.95, y: 20 }}
      animate={{ opacity: 1, scale: 1, y: 0 }}
      exit={{ opacity: 0, scale: 0.9, x: -100 }}
      whileHover={{ scale: 1.03, y: -5 }}
      onHoverStart={() => setIsHovered(true)}
      onHoverEnd={() => setIsHovered(false)}
    >
      {/* Decorative gradient orb */}
      <motion.div
        className="absolute top-0 right-0 w-32 h-32 bg-white/10 rounded-full blur-2xl"
        animate={{
          scale: isHovered ? 1.5 : 1,
          opacity: isHovered ? 0.3 : 0.2,
        }}
        transition={{ duration: 0.3 }}
        style={{ transform: 'translate(30%, -30%)' }}
      />

      <div className="relative z-10 p-6">
        <div className="flex items-start gap-4">
          {/* Custom Animated Checkbox */}
          <motion.button
            onClick={handleToggle}
            className={`relative flex-shrink-0 w-7 h-7 rounded-lg border-2 transition-all duration-300 ${
              task.completed
                ? 'bg-success-500 border-success-500 shadow-lg shadow-success-500/50'
                : 'bg-white/50 dark:bg-slate-800/50 border-primary-400 dark:border-primary-500 hover:border-primary-500 dark:hover:border-primary-400'
            }`}
            whileHover={{ scale: 1.1, rotate: 5 }}
            whileTap={{ scale: 0.95 }}
          >
            <motion.svg
              xmlns="http://www.w3.org/2000/svg"
              className="w-full h-full text-white"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
              initial={{ pathLength: 0, opacity: 0 }}
              animate={{
                pathLength: task.completed ? 1 : 0,
                opacity: task.completed ? 1 : 0,
              }}
              transition={{ duration: 0.3, ease: 'easeOut' }}
            >
              <motion.path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={3}
                d="M5 13l4 4L19 7"
              />
            </motion.svg>
          </motion.button>

          {/* Task Content */}
          <div className="flex-1 min-w-0">
            <motion.h3
              className="text-xl font-bold mb-2 relative"
              animate={{
                opacity: task.completed ? 0.6 : 1,
              }}
              transition={{ duration: 0.3 }}
            >
              <span className={task.completed ? 'line-through' : ''}>
                {task.title}
              </span>
            </motion.h3>

            {task.description && (
              <motion.p
                className="text-sm text-foreground/70 dark:text-foreground/60 mb-3 line-clamp-2"
                animate={{
                  opacity: task.completed ? 0.5 : 0.8,
                }}
                transition={{ duration: 0.3 }}
              >
                {task.description}
              </motion.p>
            )}

            {/* Metadata */}
            <div className="flex flex-wrap items-center gap-3 text-xs text-muted-foreground/80">
              <span className="flex items-center gap-1">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-3.5 w-3.5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 7V3m8 4V3m-9 8h10M5 21h14a2 2 0 002-2V7a2 2 0 00-2-2H5a2 2 0 00-2 2v12a2 2 0 002 2z" />
                </svg>
                {new Date(task.created_at).toLocaleDateString()}
              </span>
              {task.updated_at !== task.created_at && (
                <span className="flex items-center gap-1">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-3.5 w-3.5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                  </svg>
                  {new Date(task.updated_at).toLocaleDateString()}
                </span>
              )}
            </div>
          </div>
        </div>

        {/* Footer Actions */}
        <div className="flex items-center justify-between mt-6 pt-4 border-t border-white/20 dark:border-white/10">
          <motion.span
            className={`inline-flex items-center px-4 py-1.5 rounded-full text-xs font-bold uppercase tracking-wider ${
              task.completed
                ? 'bg-success-500/30 text-success-700 dark:text-success-300 border border-success-500/50'
                : 'bg-primary-500/30 text-primary-700 dark:text-primary-300 border border-primary-500/50'
            }`}
            whileHover={{ scale: 1.05 }}
          >
            {task.completed ? '✓ Completed' : '○ Pending'}
          </motion.span>

          <div className="flex gap-2">
            <Link href={`/dashboard/tasks/${task.id}`}>
              <Button variant="outline" size="sm" className="font-bold">
                View
              </Button>
            </Link>
            <Button
              variant="destructive"
              size="sm"
              onClick={handleDelete}
              disabled={isDeleting}
              className="font-bold"
            >
              {isDeleting ? 'Deleting...' : 'Delete'}
            </Button>
          </div>
        </div>
      </div>
    </motion.div>
  );
};