'use client';

import { useState, useEffect, useCallback } from 'react';
import { Search, X } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

interface TaskSearchProps {
  onSearchChange: (query: string) => void;
  placeholder?: string;
  debounceMs?: number;
}

export function TaskSearch({
  onSearchChange,
  placeholder = 'Search tasks...',
  debounceMs = 300
}: TaskSearchProps) {
  const [searchQuery, setSearchQuery] = useState('');
  const [isFocused, setIsFocused] = useState(false);

  // Debounced search effect
  useEffect(() => {
    const timer = setTimeout(() => {
      onSearchChange(searchQuery);
    }, debounceMs);

    return () => clearTimeout(timer);
  }, [searchQuery, debounceMs, onSearchChange]);

  const handleClear = () => {
    setSearchQuery('');
    onSearchChange('');
  };

  return (
    <div className="relative">
      <div
        className={`relative flex items-center gap-2 px-4 py-2.5 rounded-xl border-2 transition-all duration-300 ${
          isFocused
            ? 'border-purple-500 shadow-lg shadow-purple-500/20'
            : 'border-gray-300 dark:border-gray-600 hover:border-purple-300 dark:hover:border-purple-500'
        } glass`}
      >
        {/* Search Icon */}
        <Search className={`w-4 h-4 transition-colors ${
          isFocused ? 'text-purple-600 dark:text-purple-400' : 'text-gray-400 dark:text-gray-500'
        }`} />

        {/* Search Input */}
        <input
          type="text"
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          onFocus={() => setIsFocused(true)}
          onBlur={() => setIsFocused(false)}
          placeholder={placeholder}
          className="flex-1 bg-transparent text-gray-900 dark:text-gray-100 placeholder:text-gray-400 dark:placeholder:text-gray-500 focus:outline-none"
        />

        {/* Clear Button */}
        <AnimatePresence>
          {searchQuery && (
            <motion.button
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.8 }}
              transition={{ duration: 0.15 }}
              onClick={handleClear}
              className="p-1 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors"
              type="button"
            >
              <X className="w-4 h-4 text-gray-400 dark:text-gray-500" />
            </motion.button>
          )}
        </AnimatePresence>
      </div>

      {/* Search indicator */}
      {searchQuery && (
        <motion.div
          initial={{ opacity: 0, y: -5 }}
          animate={{ opacity: 1, y: 0 }}
          className="absolute top-full mt-2 left-0 right-0 text-xs text-gray-500 dark:text-gray-400 text-center"
        >
          Searching for "{searchQuery}"...
        </motion.div>
      )}
    </div>
  );
}
