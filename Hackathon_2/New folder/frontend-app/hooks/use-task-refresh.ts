'use client';

import { useEffect, useCallback } from 'react';

/**
 * Custom hook for listening to task update events
 * Automatically refreshes tasks when the AI modifies data
 */
export function useTaskRefresh(onRefresh: () => void) {
  const handleTasksUpdated = useCallback(() => {
    console.log('Tasks updated event received, refreshing...');
    onRefresh();
  }, [onRefresh]);

  useEffect(() => {
    // Listen for tasks-updated event
    window.addEventListener('tasks-updated', handleTasksUpdated);

    // Cleanup listener on unmount
    return () => {
      window.removeEventListener('tasks-updated', handleTasksUpdated);
    };
  }, [handleTasksUpdated]);
}

export default useTaskRefresh;
