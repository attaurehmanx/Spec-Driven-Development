'use client';

import { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { Filter, X } from 'lucide-react';
import { PriorityLevel } from '@/types';

interface TaskFiltersProps {
  onFilterChange: (filters: {
    priority: PriorityLevel[];
    status: 'all' | 'completed' | 'pending';
    tags: string[];
  }) => void;
  taskCount?: number;
  availableTags?: string[];
}

export function TaskFilters({ onFilterChange, taskCount = 0, availableTags = [] }: TaskFiltersProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedPriorities, setSelectedPriorities] = useState<PriorityLevel[]>([]);
  const [selectedStatus, setSelectedStatus] = useState<'all' | 'completed' | 'pending'>('all');
  const [selectedTags, setSelectedTags] = useState<string[]>([]);

  const handlePriorityToggle = (priority: PriorityLevel) => {
    const newPriorities = selectedPriorities.includes(priority)
      ? selectedPriorities.filter(p => p !== priority)
      : [...selectedPriorities, priority];

    setSelectedPriorities(newPriorities);
    onFilterChange({
      priority: newPriorities,
      status: selectedStatus,
      tags: selectedTags,
    });
  };

  const handleStatusChange = (status: 'all' | 'completed' | 'pending') => {
    setSelectedStatus(status);
    onFilterChange({
      priority: selectedPriorities,
      status,
      tags: selectedTags,
    });
  };

  const handleTagToggle = (tag: string) => {
    const newTags = selectedTags.includes(tag)
      ? selectedTags.filter(t => t !== tag)
      : [...selectedTags, tag];

    setSelectedTags(newTags);
    onFilterChange({
      priority: selectedPriorities,
      status: selectedStatus,
      tags: newTags,
    });
  };

  const handleClearFilters = () => {
    setSelectedPriorities([]);
    setSelectedStatus('all');
    setSelectedTags([]);
    onFilterChange({
      priority: [],
      status: 'all',
      tags: [],
    });
  };

  const hasActiveFilters = selectedPriorities.length > 0 || selectedStatus !== 'all' || selectedTags.length > 0;
  const activeFilterCount = selectedPriorities.length + (selectedStatus !== 'all' ? 1 : 0) + selectedTags.length;

  const getPriorityColor = (priority: PriorityLevel) => {
    switch (priority) {
      case 'high':
        return 'from-red-500 to-orange-500';
      case 'medium':
        return 'from-yellow-500 to-amber-500';
      case 'low':
        return 'from-green-500 to-emerald-500';
    }
  };

  const getPriorityIcon = (priority: PriorityLevel) => {
    switch (priority) {
      case 'high':
        return '🔴';
      case 'medium':
        return '🟡';
      case 'low':
        return '🟢';
    }
  };

  return (
    <div className="relative">
      {/* Filter Toggle Button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`relative inline-flex items-center gap-2 px-4 py-2.5 rounded-xl font-bold transition-all duration-300 ${
          hasActiveFilters
            ? 'bg-gradient-to-r from-purple-600 to-pink-600 text-white shadow-lg shadow-purple-500/50'
            : 'glass border-2 border-gray-300 dark:border-gray-600 text-gray-700 dark:text-gray-300 hover:border-purple-500 dark:hover:border-purple-400'
        }`}
      >
        <Filter className="w-4 h-4" />
        <span>Filters</span>
        {activeFilterCount > 0 && (
          <span className="absolute -top-2 -right-2 w-6 h-6 bg-pink-500 text-white text-xs font-bold rounded-full flex items-center justify-center shadow-lg">
            {activeFilterCount}
          </span>
        )}
      </button>

      {/* Filter Panel */}
      <AnimatePresence>
        {isOpen && (
          <>
            {/* Backdrop */}
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              className="fixed inset-0 bg-black/20 backdrop-blur-sm z-40"
              onClick={() => setIsOpen(false)}
            />

            {/* Filter Panel */}
            <motion.div
              initial={{ opacity: 0, y: -10, scale: 0.95 }}
              animate={{ opacity: 1, y: 0, scale: 1 }}
              exit={{ opacity: 0, y: -10, scale: 0.95 }}
              transition={{ duration: 0.2 }}
              className="absolute top-full mt-2 right-0 w-80 glass-strong rounded-2xl border-2 border-white/30 dark:border-white/20 shadow-2xl z-50 overflow-hidden"
            >
              {/* Header */}
              <div className="relative px-4 py-3 border-b border-gray-200 dark:border-gray-700">
                <div className="absolute inset-0 bg-gradient-to-r from-purple-500/10 via-pink-500/10 to-cyan-500/10"></div>
                <div className="relative flex items-center justify-between">
                  <h3 className="text-lg font-extrabold bg-gradient-to-r from-purple-600 to-pink-600 dark:from-purple-400 dark:to-pink-400 bg-clip-text text-transparent">
                    Filter Tasks
                  </h3>
                  <button
                    onClick={() => setIsOpen(false)}
                    className="p-1.5 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors"
                  >
                    <X className="w-4 h-4 text-gray-500 dark:text-gray-400" />
                  </button>
                </div>
              </div>

              {/* Filter Content */}
              <div className="p-4 space-y-4">
                {/* Priority Filter */}
                <div>
                  <label className="block text-sm font-bold text-gray-700 dark:text-gray-300 mb-2">
                    Priority Level
                  </label>
                  <div className="space-y-2">
                    {(['high', 'medium', 'low'] as PriorityLevel[]).map((priority) => (
                      <button
                        key={priority}
                        onClick={() => handlePriorityToggle(priority)}
                        className={`w-full flex items-center gap-3 px-3 py-2.5 rounded-xl border-2 transition-all duration-200 ${
                          selectedPriorities.includes(priority)
                            ? 'border-purple-500 bg-purple-50 dark:bg-purple-900/20'
                            : 'border-gray-300 dark:border-gray-600 hover:border-purple-300 dark:hover:border-purple-500'
                        }`}
                      >
                        {/* Checkbox */}
                        <div className={`w-5 h-5 rounded border-2 flex items-center justify-center transition-all ${
                          selectedPriorities.includes(priority)
                            ? 'bg-purple-600 border-purple-600'
                            : 'border-gray-300 dark:border-gray-600'
                        }`}>
                          {selectedPriorities.includes(priority) && (
                            <svg className="w-3 h-3 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M5 13l4 4L19 7" />
                            </svg>
                          )}
                        </div>

                        {/* Priority Badge */}
                        <span className={`flex-1 text-left inline-flex items-center gap-2 px-2.5 py-1 rounded-full text-xs font-bold bg-gradient-to-r ${getPriorityColor(priority)} text-white shadow-sm`}>
                          <span>{getPriorityIcon(priority)}</span>
                          <span>{priority.charAt(0).toUpperCase() + priority.slice(1)} Priority</span>
                        </span>
                      </button>
                    ))}
                  </div>
                </div>

                {/* Tags Filter */}
                {availableTags.length > 0 && (
                  <div>
                    <label className="block text-sm font-bold text-gray-700 dark:text-gray-300 mb-2">
                      Tags
                    </label>
                    <div className="max-h-48 overflow-y-auto space-y-2">
                      {availableTags.map((tag) => (
                        <button
                          key={tag}
                          onClick={() => handleTagToggle(tag)}
                          className={`w-full flex items-center gap-3 px-3 py-2.5 rounded-xl border-2 transition-all duration-200 ${
                            selectedTags.includes(tag)
                              ? 'border-cyan-500 bg-cyan-50 dark:bg-cyan-900/20'
                              : 'border-gray-300 dark:border-gray-600 hover:border-cyan-300 dark:hover:border-cyan-500'
                          }`}
                        >
                          {/* Checkbox */}
                          <div className={`w-5 h-5 rounded border-2 flex items-center justify-center transition-all ${
                            selectedTags.includes(tag)
                              ? 'bg-cyan-600 border-cyan-600'
                              : 'border-gray-300 dark:border-gray-600'
                          }`}>
                            {selectedTags.includes(tag) && (
                              <svg className="w-3 h-3 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M5 13l4 4L19 7" />
                              </svg>
                            )}
                          </div>

                          {/* Tag Badge */}
                          <span className="flex-1 text-left inline-flex items-center px-2.5 py-1 rounded-full text-xs font-bold bg-gradient-to-r from-cyan-500 to-blue-500 text-white shadow-sm">
                            {tag}
                          </span>
                        </button>
                      ))}
                    </div>
                  </div>
                )}

                {/* Status Filter */}
                <div>
                  <label className="block text-sm font-bold text-gray-700 dark:text-gray-300 mb-2">
                    Status
                  </label>
                  <div className="flex gap-2">
                    {(['all', 'pending', 'completed'] as const).map((status) => (
                      <button
                        key={status}
                        onClick={() => handleStatusChange(status)}
                        className={`flex-1 px-3 py-2 text-sm font-bold rounded-xl transition-all duration-200 ${
                          selectedStatus === status
                            ? 'bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50'
                            : 'glass border-2 border-gray-300 dark:border-gray-600 text-gray-600 dark:text-gray-400 hover:border-purple-300 dark:hover:border-purple-500'
                        }`}
                      >
                        {status.charAt(0).toUpperCase() + status.slice(1)}
                      </button>
                    ))}
                  </div>
                </div>

                {/* Task Count */}
                {taskCount > 0 && (
                  <div className="pt-2 border-t border-gray-200 dark:border-gray-700">
                    <p className="text-sm text-gray-600 dark:text-gray-400 text-center">
                      Showing <span className="font-bold text-purple-600 dark:text-purple-400">{taskCount}</span> task{taskCount !== 1 ? 's' : ''}
                    </p>
                  </div>
                )}

                {/* Clear Filters Button */}
                {hasActiveFilters && (
                  <button
                    onClick={handleClearFilters}
                    className="w-full px-4 py-2.5 rounded-xl border-2 border-gray-300 dark:border-gray-600 bg-white/50 dark:bg-slate-800/50 hover:bg-red-50 dark:hover:bg-red-900/20 hover:border-red-500 dark:hover:border-red-400 text-red-600 dark:text-red-400 font-bold transition-all duration-200"
                  >
                    Clear All Filters
                  </button>
                )}
              </div>
            </motion.div>
          </>
        )}
      </AnimatePresence>
    </div>
  );
}
