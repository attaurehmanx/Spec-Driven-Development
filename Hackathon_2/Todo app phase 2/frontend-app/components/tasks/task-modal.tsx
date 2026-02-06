'use client';

import { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { X, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Task } from '@/types';

interface TaskModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSubmit: (data: { title: string; description: string; completed: boolean }) => Promise<void>;
  task?: Task | null;
  mode: 'create' | 'edit';
}

export function TaskModal({ isOpen, onClose, onSubmit, task, mode }: TaskModalProps) {
  const [title, setTitle] = useState('');
  const [description, setDescription] = useState('');
  const [completed, setCompleted] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [errors, setErrors] = useState<{ title?: string; description?: string }>({});

  useEffect(() => {
    if (task && mode === 'edit') {
      setTitle(task.title);
      setDescription(task.description || '');
      setCompleted(task.completed);
    } else {
      setTitle('');
      setDescription('');
      setCompleted(false);
    }
    setErrors({});
  }, [task, mode, isOpen]);

  const validate = () => {
    const newErrors: { title?: string; description?: string } = {};

    if (!title.trim()) {
      newErrors.title = 'Title is required';
    } else if (title.length > 200) {
      newErrors.title = 'Title must be less than 200 characters';
    }

    if (description && description.length > 1000) {
      newErrors.description = 'Description must be less than 1000 characters';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validate()) {
      return;
    }

    setIsSubmitting(true);
    try {
      await onSubmit({
        title: title.trim(),
        description: description.trim(),
        completed,
      });
      onClose();
    } catch (error) {
      console.error('Error submitting task:', error);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleClose = () => {
    if (!isSubmitting) {
      onClose();
    }
  };

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Overlay */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
            className="fixed inset-0 bg-black/50 backdrop-blur-sm z-50"
            onClick={handleClose}
          />

          {/* Modal */}
          <div className="fixed inset-0 z-50 flex items-center justify-center p-4">
            <motion.div
              initial={{ opacity: 0, scale: 0.95, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.95, y: 20 }}
              transition={{ duration: 0.3, ease: 'easeOut' }}
              className="relative w-full max-w-2xl"
              onClick={(e) => e.stopPropagation()}
            >
              {/* Gradient border wrapper */}
              <div className="absolute inset-0 bg-gradient-to-r from-purple-500 via-pink-500 to-cyan-500 rounded-3xl blur-sm opacity-75"></div>

              {/* Modal content */}
              <div className="relative glass-strong rounded-3xl border-2 border-white/30 dark:border-white/20 shadow-2xl overflow-hidden">
                {/* Header */}
                <div className="relative px-8 py-6 border-b border-gray-200 dark:border-gray-700">
                  <div className="absolute inset-0 bg-gradient-to-r from-purple-500/10 via-pink-500/10 to-cyan-500/10"></div>
                  <div className="relative flex items-center justify-between">
                    <h2 className="text-3xl font-extrabold bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent">
                      {mode === 'create' ? 'Create New Task' : 'Edit Task'}
                    </h2>
                    <button
                      onClick={handleClose}
                      disabled={isSubmitting}
                      className="p-2 rounded-xl hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors disabled:opacity-50"
                    >
                      <X className="w-6 h-6 text-gray-500 dark:text-gray-400" />
                    </button>
                  </div>
                </div>

                {/* Form */}
                <form onSubmit={handleSubmit} className="p-8 space-y-6">
                  {/* Title Input */}
                  <div>
                    <label htmlFor="title" className="block text-sm font-bold text-gray-700 dark:text-gray-300 mb-2">
                      Task Title *
                    </label>
                    <input
                      id="title"
                      type="text"
                      value={title}
                      onChange={(e) => setTitle(e.target.value)}
                      placeholder="Enter task title..."
                      disabled={isSubmitting}
                      className={`w-full px-4 py-3 rounded-xl border-2 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed ${
                        errors.title
                          ? 'border-red-500 focus:border-red-500'
                          : 'border-gray-300 dark:border-gray-600 focus:border-purple-500'
                      }`}
                    />
                    {errors.title && (
                      <motion.p
                        initial={{ opacity: 0, y: -10 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="mt-2 text-sm text-red-600 dark:text-red-400"
                      >
                        {errors.title}
                      </motion.p>
                    )}
                  </div>

                  {/* Description Input */}
                  <div>
                    <label htmlFor="description" className="block text-sm font-bold text-gray-700 dark:text-gray-300 mb-2">
                      Description
                    </label>
                    <textarea
                      id="description"
                      value={description}
                      onChange={(e) => setDescription(e.target.value)}
                      placeholder="Enter task description..."
                      rows={4}
                      disabled={isSubmitting}
                      className={`w-full px-4 py-3 rounded-xl border-2 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 transition-all resize-none disabled:opacity-50 disabled:cursor-not-allowed ${
                        errors.description
                          ? 'border-red-500 focus:border-red-500'
                          : 'border-gray-300 dark:border-gray-600 focus:border-purple-500'
                      }`}
                    />
                    {errors.description && (
                      <motion.p
                        initial={{ opacity: 0, y: -10 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="mt-2 text-sm text-red-600 dark:text-red-400"
                      >
                        {errors.description}
                      </motion.p>
                    )}
                  </div>

                  {/* Completed Checkbox (only in edit mode) */}
                  {mode === 'edit' && (
                    <div className="flex items-center gap-3">
                      <input
                        id="completed"
                        type="checkbox"
                        checked={completed}
                        onChange={(e) => setCompleted(e.target.checked)}
                        disabled={isSubmitting}
                        className="w-5 h-5 rounded border-2 border-gray-300 dark:border-gray-600 text-purple-600 focus:ring-2 focus:ring-purple-500 disabled:opacity-50"
                      />
                      <label htmlFor="completed" className="text-sm font-medium text-gray-700 dark:text-gray-300">
                        Mark as completed
                      </label>
                    </div>
                  )}

                  {/* Action Buttons */}
                  <div className="flex gap-3 pt-4">
                    <Button
                      type="button"
                      variant="outline"
                      onClick={handleClose}
                      disabled={isSubmitting}
                      className="flex-1"
                    >
                      Cancel
                    </Button>
                    <Button
                      type="submit"
                      disabled={isSubmitting}
                      className="flex-1"
                    >
                      {isSubmitting ? (
                        <>
                          <Loader2 className="w-4 h-4 animate-spin" />
                          {mode === 'create' ? 'Creating...' : 'Saving...'}
                        </>
                      ) : (
                        mode === 'create' ? 'Create Task' : 'Save Changes'
                      )}
                    </Button>
                  </div>
                </form>
              </div>
            </motion.div>
          </div>
        </>
      )}
    </AnimatePresence>
  );
}
