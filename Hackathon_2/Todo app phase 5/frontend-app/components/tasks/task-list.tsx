'use client';

import React, { useState } from 'react';
import { motion, AnimatePresence, Reorder } from 'framer-motion';
import { Task } from '../../types';
import { TaskItem } from './task-item';

interface TaskListProps {
  tasks: Task[];
  loading?: boolean;
  error?: string | null;
  onTaskToggle?: (task: Task) => void;
  onTaskDelete?: (taskId: string) => void;
  onTaskReorder?: (tasks: Task[]) => void;
  emptyState?: React.ReactNode;
  enableDragDrop?: boolean;
}

// Check for reduced motion preference
const useReducedMotion = () => {
  const [prefersReducedMotion, setPrefersReducedMotion] = React.useState(false);

  React.useEffect(() => {
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    setPrefersReducedMotion(mediaQuery.matches);

    const handleChange = () => {
      setPrefersReducedMotion(mediaQuery.matches);
    };

    mediaQuery.addEventListener('change', handleChange);
    return () => mediaQuery.removeEventListener('change', handleChange);
  }, []);

  return prefersReducedMotion;
};

// Animation variants
const containerVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
    },
  },
};

const itemVariants = {
  hidden: { opacity: 0, y: 20 },
  visible: {
    opacity: 1,
    y: 0,
  },
  exit: {
    opacity: 0,
    x: -100,
  },
};

const itemTransition = {
  duration: 0.4,
  ease: [0.4, 0, 0.2, 1] as const,
};

const TaskList: React.FC<TaskListProps> = ({
  tasks,
  loading = false,
  error,
  onTaskToggle,
  onTaskDelete,
  onTaskReorder,
  emptyState,
  enableDragDrop = false,
}) => {
  const prefersReducedMotion = useReducedMotion();
  const [orderedTasks, setOrderedTasks] = useState(tasks);

  React.useEffect(() => {
    setOrderedTasks(tasks);
  }, [tasks]);

  const handleReorder = (newOrder: Task[]) => {
    setOrderedTasks(newOrder);
    onTaskReorder?.(newOrder);
  };

  if (loading) {
    return (
      <div className="flex justify-center items-center py-12">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-primary"></div>
      </div>
    );
  }

  if (error) {
    return (
      <motion.div
        initial={{ opacity: 0, y: -10 }}
        animate={{ opacity: 1, y: 0 }}
        className="bg-destructive/10 dark:bg-destructive/20 border-l-4 border-destructive p-4 rounded-lg"
      >
        <div className="flex">
          <div className="flex-shrink-0">
            <svg
              className="h-5 w-5 text-destructive"
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 20 20"
              fill="currentColor"
              aria-hidden="true"
            >
              <path
                fillRule="evenodd"
                d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z"
                clipRule="evenodd"
              />
            </svg>
          </div>
          <div className="ml-3">
            <p className="text-sm text-destructive">{error}</p>
          </div>
        </div>
      </motion.div>
    );
  }

  if (tasks.length === 0) {
    return emptyState || (
      <motion.div
        initial={{ opacity: 0, scale: 0.95 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ duration: 0.3 }}
        className="text-center py-12"
      >
        <svg
          className="mx-auto h-12 w-12 text-muted-foreground"
          fill="none"
          viewBox="0 0 24 24"
          stroke="currentColor"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2"
          />
        </svg>
        <h3 className="mt-2 text-sm font-medium text-foreground">No tasks</h3>
        <p className="mt-1 text-sm text-muted-foreground">Get started by creating a new task.</p>
      </motion.div>
    );
  }

  // Render with drag-and-drop if enabled
  if (enableDragDrop && !prefersReducedMotion) {
    return (
      <Reorder.Group
        axis="y"
        values={orderedTasks}
        onReorder={handleReorder}
        className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6"
      >
        <AnimatePresence mode="popLayout">
          {orderedTasks.map((task) => (
            <Reorder.Item
              key={task.id}
              value={task}
              className="cursor-grab active:cursor-grabbing"
              whileDrag={{
                scale: 1.05,
                boxShadow: '0 20px 40px rgba(139, 92, 246, 0.3)',
                zIndex: 50,
              }}
              transition={{ type: 'spring', stiffness: 300, damping: 30 }}
            >
              <TaskItem
                task={task}
                onToggle={onTaskToggle}
                onDelete={onTaskDelete}
              />
            </Reorder.Item>
          ))}
        </AnimatePresence>
      </Reorder.Group>
    );
  }

  // Default render without drag-and-drop
  return (
    <motion.div
      variants={prefersReducedMotion ? undefined : containerVariants}
      initial="hidden"
      animate="visible"
      className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6"
    >
      <AnimatePresence mode="popLayout">
        {orderedTasks.map((task) => (
          <motion.div
            key={task.id}
            variants={prefersReducedMotion ? undefined : itemVariants}
            layout={!prefersReducedMotion}
            initial="hidden"
            animate="visible"
            exit="exit"
            transition={prefersReducedMotion ? undefined : itemTransition}
          >
            <TaskItem
              task={task}
              onToggle={onTaskToggle}
              onDelete={onTaskDelete}
            />
          </motion.div>
        ))}
      </AnimatePresence>
    </motion.div>
  );
};

export { TaskList };