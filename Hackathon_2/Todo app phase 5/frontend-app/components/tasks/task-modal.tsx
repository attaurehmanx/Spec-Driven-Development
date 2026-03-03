'use client';

import { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { X, Loader2, Calendar } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Task, PriorityLevel, RecurringPattern } from '@/types';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';

interface TaskModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSubmit: (data: {
    title: string;
    description: string;
    completed: boolean;
    priority?: PriorityLevel;
    tags?: string[];
    recurring?: RecurringPattern;
    due_date?: string;
  }) => Promise<void>;
  task?: Task | null;
  mode: 'create' | 'edit';
}

export function TaskModal({ isOpen, onClose, onSubmit, task, mode }: TaskModalProps) {
  const [title, setTitle] = useState('');
  const [description, setDescription] = useState('');
  const [completed, setCompleted] = useState(false);
  const [priority, setPriority] = useState<PriorityLevel>('medium');
  const [tags, setTags] = useState<string[]>([]);
  const [tagInput, setTagInput] = useState('');
  const [recurring, setRecurring] = useState<RecurringPattern>('none');
  const [dueDate, setDueDate] = useState<Date | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [errors, setErrors] = useState<{ title?: string; description?: string; tags?: string }>({});

  useEffect(() => {
    if (task && mode === 'edit') {
      setTitle(task.title);
      setDescription(task.description || '');
      setCompleted(task.completed);
      setPriority(task.priority || 'medium');
      setTags(task.tags || []);
      setRecurring(task.recurring || 'none');
      setDueDate(task.due_date ? new Date(task.due_date) : null);
    } else {
      setTitle('');
      setDescription('');
      setCompleted(false);
      setPriority('medium');
      setTags([]);
      setRecurring('none');
      setDueDate(null);
    }
    setTagInput('');
    setErrors({});
  }, [task, mode, isOpen]);

  const validate = () => {
    const newErrors: { title?: string; description?: string; tags?: string } = {};

    if (!title.trim()) {
      newErrors.title = 'Title is required';
    } else if (title.length > 200) {
      newErrors.title = 'Title must be less than 200 characters';
    }

    if (description && description.length > 1000) {
      newErrors.description = 'Description must be less than 1000 characters';
    }

    if (tags.length > 20) {
      newErrors.tags = 'Maximum 20 tags allowed';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleAddTag = () => {
    const trimmedTag = tagInput.trim();

    // Validate tag format (alphanumeric and hyphens only, 1-50 chars)
    const tagRegex = /^[a-zA-Z0-9-]{1,50}$/;

    if (!trimmedTag) return;

    if (!tagRegex.test(trimmedTag)) {
      setErrors({ ...errors, tags: 'Tags must be 1-50 characters, alphanumeric and hyphens only' });
      return;
    }

    if (tags.includes(trimmedTag)) {
      setErrors({ ...errors, tags: 'Tag already added' });
      return;
    }

    if (tags.length >= 20) {
      setErrors({ ...errors, tags: 'Maximum 20 tags allowed' });
      return;
    }

    setTags([...tags, trimmedTag]);
    setTagInput('');
    setErrors({ ...errors, tags: undefined });
  };

  const handleRemoveTag = (tagToRemove: string) => {
    setTags(tags.filter(tag => tag !== tagToRemove));
  };

  const handleTagInputKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleAddTag();
    }
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
        priority,
        tags,
        recurring,
        due_date: dueDate ? dueDate.toISOString() : undefined,
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

  const getPriorityColor = (level: PriorityLevel) => {
    switch (level) {
      case 'high':
        return 'from-red-500 to-orange-500';
      case 'medium':
        return 'from-yellow-500 to-amber-500';
      case 'low':
        return 'from-green-500 to-emerald-500';
      default:
        return 'from-gray-500 to-slate-500';
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
          <div className="fixed inset-0 z-50 flex items-center justify-center p-3 sm:p-4">
            <motion.div
              initial={{ opacity: 0, scale: 0.95, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.95, y: 20 }}
              transition={{ duration: 0.3, ease: 'easeOut' }}
              className="relative w-full max-w-sm sm:max-w-md md:max-w-2xl max-h-[90vh] overflow-y-auto"
              onClick={(e) => e.stopPropagation()}
            >
              {/* Gradient border wrapper */}
              <div className="absolute inset-0 bg-gradient-to-r from-purple-500 via-pink-500 to-cyan-500 rounded-2xl sm:rounded-3xl blur-sm opacity-75"></div>

              {/* Modal content */}
              <div className="relative glass-strong rounded-2xl sm:rounded-3xl border-2 border-white/30 dark:border-white/20 shadow-2xl overflow-hidden">
                {/* Header */}
                <div className="relative px-4 sm:px-6 py-3 sm:py-4 border-b border-gray-200 dark:border-gray-700">
                  <div className="absolute inset-0 bg-gradient-to-r from-purple-500/10 via-pink-500/10 to-cyan-500/10"></div>
                  <div className="relative flex items-center justify-between">
                    <h2 className="text-lg sm:text-xl md:text-2xl font-extrabold bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent">
                      {mode === 'create' ? 'Create New Task' : 'Edit Task'}
                    </h2>
                    <button
                      onClick={handleClose}
                      disabled={isSubmitting}
                      className="p-1.5 sm:p-2 rounded-lg sm:rounded-xl hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors disabled:opacity-50"
                    >
                      <X className="w-4 h-4 sm:w-5 sm:h-5 text-gray-500 dark:text-gray-400" />
                    </button>
                  </div>
                </div>

                {/* Form */}
                <form onSubmit={handleSubmit} className="p-4 sm:p-6 space-y-3 sm:space-y-4">
                  {/* Title Input */}
                  <div>
                    <label htmlFor="title" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Task Title *
                    </label>
                    <input
                      id="title"
                      type="text"
                      value={title}
                      onChange={(e) => setTitle(e.target.value)}
                      placeholder="Enter task title..."
                      disabled={isSubmitting}
                      className={`w-full px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl border-2 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed text-sm sm:text-base ${
                        errors.title
                          ? 'border-red-500 focus:border-red-500'
                          : 'border-gray-300 dark:border-gray-600 focus:border-purple-500'
                      }`}
                    />
                    {errors.title && (
                      <motion.p
                        initial={{ opacity: 0, y: -10 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="mt-1.5 text-xs sm:text-sm text-red-600 dark:text-red-400"
                      >
                        {errors.title}
                      </motion.p>
                    )}
                  </div>

                  {/* Description Input */}
                  <div>
                    <label htmlFor="description" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Description
                    </label>
                    <textarea
                      id="description"
                      value={description}
                      onChange={(e) => setDescription(e.target.value)}
                      placeholder="Enter task description..."
                      rows={3}
                      disabled={isSubmitting}
                      className={`w-full px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl border-2 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 transition-all resize-none disabled:opacity-50 disabled:cursor-not-allowed text-sm sm:text-base ${
                        errors.description
                          ? 'border-red-500 focus:border-red-500'
                          : 'border-gray-300 dark:border-gray-600 focus:border-purple-500'
                      }`}
                    />
                    {errors.description && (
                      <motion.p
                        initial={{ opacity: 0, y: -10 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="mt-1.5 text-xs sm:text-sm text-red-600 dark:text-red-400"
                      >
                        {errors.description}
                      </motion.p>
                    )}
                  </div>

                  {/* Tags Input */}
                  <div>
                    <label htmlFor="tags" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Tags
                    </label>
                    <div className="flex flex-col sm:flex-row gap-2">
                      <input
                        id="tags"
                        type="text"
                        value={tagInput}
                        onChange={(e) => setTagInput(e.target.value)}
                        onKeyDown={handleTagInputKeyDown}
                        placeholder="Add tags (press Enter)..."
                        disabled={isSubmitting}
                        className="flex-1 px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl border-2 border-gray-300 dark:border-gray-600 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed text-sm sm:text-base"
                      />
                      <button
                        type="button"
                        onClick={handleAddTag}
                        disabled={isSubmitting || !tagInput.trim()}
                        className="px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl bg-purple-600 hover:bg-purple-500 text-white font-bold transition-all disabled:opacity-50 disabled:cursor-not-allowed text-sm sm:text-base whitespace-nowrap"
                      >
                        Add
                      </button>
                    </div>
                    {errors.tags && (
                      <motion.p
                        initial={{ opacity: 0, y: -10 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="mt-1.5 text-xs sm:text-sm text-red-600 dark:text-red-400"
                      >
                        {errors.tags}
                      </motion.p>
                    )}
                    {/* Display current tags */}
                    {tags.length > 0 && (
                      <div className="mt-2 flex flex-wrap gap-1.5 sm:gap-2">
                        {tags.map((tag) => (
                          <span
                            key={tag}
                            className="inline-flex items-center gap-1 sm:gap-1.5 px-2 sm:px-3 py-1 rounded-full text-xs font-bold bg-gradient-to-r from-cyan-500 to-blue-500 text-white shadow-sm"
                          >
                            {tag}
                            <button
                              type="button"
                              onClick={() => handleRemoveTag(tag)}
                              disabled={isSubmitting}
                              className="hover:bg-white/20 rounded-full p-0.5 transition-colors disabled:opacity-50"
                            >
                              <svg className="w-2.5 h-2.5 sm:w-3 sm:h-3" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                              </svg>
                            </button>
                          </span>
                        ))}
                      </div>
                    )}
                    <p className="mt-1.5 text-xs text-gray-500 dark:text-gray-400">
                      {tags.length}/20 tags • Alphanumeric and hyphens only
                    </p>
                  </div>

                  {/* Recurring Dropdown */}
                  <div>
                    <label htmlFor="recurring" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Recurring Pattern
                    </label>
                    <select
                      id="recurring"
                      value={recurring}
                      onChange={(e) => setRecurring(e.target.value as RecurringPattern)}
                      disabled={isSubmitting}
                      className="w-full px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl border-2 border-gray-300 dark:border-gray-600 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed appearance-none cursor-pointer text-sm sm:text-base"
                    >
                      <option value="none">🚫 No Recurrence</option>
                      <option value="daily">📅 Daily</option>
                      <option value="weekly">📆 Weekly</option>
                      <option value="monthly">🗓️ Monthly</option>
                    </select>
                    <p className="mt-1.5 text-xs text-gray-500 dark:text-gray-400">
                      Recurring tasks automatically create a new instance when completed
                    </p>
                  </div>

                  {/* Due Date Picker */}
                  <div>
                    <label htmlFor="due-date" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Due Date
                    </label>
                    <div className="relative">
                      <DatePicker
                        selected={dueDate}
                        onChange={(date) => setDueDate(date)}
                        showTimeSelect
                        timeFormat="HH:mm"
                        timeIntervals={15}
                        dateFormat="MMMM d, yyyy h:mm aa"
                        placeholderText="Select due date and time..."
                        disabled={isSubmitting}
                        minDate={new Date()}
                        className="w-full px-3 sm:px-4 py-2 sm:py-2.5 pl-9 sm:pl-10 rounded-lg sm:rounded-xl border-2 border-gray-300 dark:border-gray-600 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed text-sm sm:text-base"
                        wrapperClassName="w-full"
                      />
                      <Calendar className="absolute left-2.5 sm:left-3 top-1/2 -translate-y-1/2 w-3.5 h-3.5 sm:w-4 sm:h-4 text-gray-400 pointer-events-none" />
                      {dueDate && (
                        <button
                          type="button"
                          onClick={() => setDueDate(null)}
                          className="absolute right-2 sm:right-3 top-1/2 -translate-y-1/2 p-1 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors"
                        >
                          <X className="w-3.5 h-3.5 sm:w-4 sm:h-4 text-gray-400" />
                        </button>
                      )}
                    </div>
                    <p className="mt-1.5 text-xs text-gray-500 dark:text-gray-400">
                      Optional: Set a due date to receive reminders
                    </p>
                  </div>

                  {/* Priority Dropdown */}
                  <div>
                    <label htmlFor="priority" className="block text-xs sm:text-sm font-bold text-gray-700 dark:text-gray-300 mb-1.5">
                      Priority
                    </label>
                    <div className="relative">
                      <select
                        id="priority"
                        value={priority}
                        onChange={(e) => setPriority(e.target.value as PriorityLevel)}
                        disabled={isSubmitting}
                        className="w-full px-3 sm:px-4 py-2 sm:py-2.5 rounded-lg sm:rounded-xl border-2 border-gray-300 dark:border-gray-600 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-purple-500 transition-all disabled:opacity-50 disabled:cursor-not-allowed appearance-none cursor-pointer text-sm sm:text-base"
                      >
                        <option value="high">🔴 High Priority</option>
                        <option value="medium">🟡 Medium Priority</option>
                        <option value="low">🟢 Low Priority</option>
                      </select>
                      <div className="absolute right-3 sm:right-4 top-1/2 -translate-y-1/2 pointer-events-none">
                        <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                        </svg>
                      </div>
                    </div>
                    {/* Priority Preview */}
                    <div className="mt-2 flex items-center gap-2">
                      <span className="text-xs text-gray-500 dark:text-gray-400">Preview:</span>
                      <span className={`inline-flex items-center px-2 sm:px-2.5 py-1 rounded-full text-xs font-bold bg-gradient-to-r ${getPriorityColor(priority)} text-white shadow-sm`}>
                        {priority.charAt(0).toUpperCase() + priority.slice(1)} Priority
                      </span>
                    </div>
                  </div>

                  {/* Completed Checkbox (only in edit mode) */}
                  {mode === 'edit' && (
                    <div className="flex items-center gap-2 sm:gap-2.5">
                      <input
                        id="completed"
                        type="checkbox"
                        checked={completed}
                        onChange={(e) => setCompleted(e.target.checked)}
                        disabled={isSubmitting}
                        className="w-3.5 h-3.5 sm:w-4 sm:h-4 rounded border-2 border-gray-300 dark:border-gray-600 text-purple-600 focus:ring-2 focus:ring-purple-500 disabled:opacity-50"
                      />
                      <label htmlFor="completed" className="text-xs sm:text-sm font-medium text-gray-700 dark:text-gray-300">
                        Mark as completed
                      </label>
                    </div>
                  )}

                  {/* Action Buttons */}
                  <div className="flex flex-col sm:flex-row gap-2 sm:gap-2.5 pt-2">
                    <Button
                      type="button"
                      variant="outline"
                      onClick={handleClose}
                      disabled={isSubmitting}
                      className="w-full sm:flex-1 py-2 sm:py-2.5 hover:bg-purple-100 dark:hover:bg-purple-500 hover:text-purple-700 dark:hover:text-white text-sm sm:text-base"
                    >
                      Cancel
                    </Button>
                    <Button
                      type="submit"
                      variant="default"
                      disabled={isSubmitting}
                      className="w-full sm:flex-1 py-2 sm:py-2.5 text-sm sm:text-base"
                      style={{
                        background: 'linear-gradient(to right, #9333ea, #a855f7)',
                        color: 'white'
                      }}
                    >
                      {isSubmitting ? (
                        <>
                          <Loader2 className="w-3.5 h-3.5 sm:w-4 sm:h-4 animate-spin" />
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
