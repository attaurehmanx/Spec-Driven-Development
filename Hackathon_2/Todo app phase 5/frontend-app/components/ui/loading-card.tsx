'use client';

import { motion } from 'framer-motion';
import { Skeleton, SkeletonText } from './skeleton';
import { cn } from '@/lib/utils';

interface LoadingCardProps {
  className?: string;
  variant?: 'task' | 'profile' | 'stats';
}

/**
 * Loading card for task items
 */
export function TaskLoadingCard({ className }: { className?: string }) {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      className={cn(
        'p-6 rounded-2xl border-2 border-gray-200 dark:border-gray-800',
        'bg-white/80 dark:bg-slate-900/80 backdrop-blur-xl',
        'shadow-lg',
        className
      )}
    >
      <div className="flex items-start justify-between mb-4">
        <div className="flex-1 space-y-3">
          <Skeleton className="h-6 w-3/4" />
          <Skeleton className="h-4 w-1/2" />
        </div>
        <Skeleton className="h-8 w-20 rounded-full" />
      </div>
      <SkeletonText lines={2} />
      <div className="flex gap-2 mt-4">
        <Skeleton className="h-9 w-28" />
        <Skeleton className="h-9 w-28" />
      </div>
    </motion.div>
  );
}

/**
 * Loading card for stats
 */
export function StatsLoadingCard({ className }: { className?: string }) {
  return (
    <motion.div
      initial={{ opacity: 0, scale: 0.95 }}
      animate={{ opacity: 1, scale: 1 }}
      className={cn(
        'p-8 rounded-2xl',
        'bg-gradient-to-br from-purple-500 to-pink-500',
        'shadow-2xl shadow-purple-500/40',
        className
      )}
    >
      <div className="flex items-center justify-between mb-4">
        <Skeleton className="w-14 h-14 rounded-xl bg-white/20" animate={false} />
      </div>
      <Skeleton className="h-4 w-24 mb-2 bg-white/30" animate={false} />
      <Skeleton className="h-12 w-20 bg-white/40" animate={false} />
    </motion.div>
  );
}

/**
 * Loading card for profile sections
 */
export function ProfileLoadingCard({ className }: { className?: string }) {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      className={cn(
        'p-8 rounded-3xl',
        'bg-white/80 dark:bg-slate-900/80 backdrop-blur-xl',
        'border-2 border-white/30 dark:border-white/20',
        'shadow-2xl',
        className
      )}
    >
      <div className="flex items-center gap-6 mb-6">
        <Skeleton className="w-24 h-24 rounded-2xl" />
        <div className="flex-1 space-y-3">
          <Skeleton className="h-8 w-48" />
          <Skeleton className="h-5 w-64" />
        </div>
      </div>
      <div className="grid grid-cols-3 gap-4 mb-6">
        {[1, 2, 3].map((i) => (
          <div key={i} className="space-y-2">
            <Skeleton className="h-4 w-20" />
            <Skeleton className="h-8 w-16" />
          </div>
        ))}
      </div>
      <SkeletonText lines={4} />
    </motion.div>
  );
}

/**
 * Generic loading card with customizable content
 */
export default function LoadingCard({ className, variant = 'task' }: LoadingCardProps) {
  switch (variant) {
    case 'task':
      return <TaskLoadingCard className={className} />;
    case 'stats':
      return <StatsLoadingCard className={className} />;
    case 'profile':
      return <ProfileLoadingCard className={className} />;
    default:
      return <TaskLoadingCard className={className} />;
  }
}
