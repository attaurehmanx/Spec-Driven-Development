'use client';

import { motion } from 'framer-motion';
import { cn } from '@/lib/utils';

interface SpinnerProps {
  size?: 'sm' | 'md' | 'lg' | 'xl';
  className?: string;
  label?: string;
}

const sizeClasses = {
  sm: 'w-4 h-4 border-2',
  md: 'w-8 h-8 border-3',
  lg: 'w-12 h-12 border-4',
  xl: 'w-16 h-16 border-4',
};

export default function Spinner({ size = 'md', className, label }: SpinnerProps) {
  return (
    <div className="flex flex-col items-center justify-center gap-3">
      <motion.div
        className={cn(
          'rounded-full border-transparent border-t-purple-500 border-r-pink-500 border-b-cyan-500 border-l-transparent',
          sizeClasses[size],
          className
        )}
        animate={{ rotate: 360 }}
        transition={{
          duration: 1,
          repeat: Infinity,
          ease: 'linear',
        }}
        style={{
          background: 'conic-gradient(from 0deg, transparent, #8B5CF6, #EC4899, #06B6D4, transparent)',
          WebkitMaskImage: 'radial-gradient(circle, transparent 40%, black 40%)',
          maskImage: 'radial-gradient(circle, transparent 40%, black 40%)',
        }}
      />
      {label && (
        <motion.p
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.2 }}
          className="text-sm font-medium text-gray-600 dark:text-gray-400"
        >
          {label}
        </motion.p>
      )}
    </div>
  );
}

/**
 * Gradient spinner with glow effect
 */
export function GradientSpinner({ size = 'md', className }: Omit<SpinnerProps, 'label'>) {
  return (
    <motion.div
      className={cn(
        'rounded-full bg-gradient-to-r from-purple-500 via-pink-500 to-cyan-500',
        'shadow-lg shadow-purple-500/50',
        sizeClasses[size],
        className
      )}
      animate={{ rotate: 360 }}
      transition={{
        duration: 1.5,
        repeat: Infinity,
        ease: 'linear',
      }}
      style={{
        WebkitMaskImage: 'radial-gradient(circle, transparent 35%, black 35%)',
        maskImage: 'radial-gradient(circle, transparent 35%, black 35%)',
      }}
    />
  );
}

/**
 * Pulsing dot spinner
 */
export function DotSpinner({ className }: { className?: string }) {
  return (
    <div className={cn('flex items-center gap-2', className)}>
      {[0, 1, 2].map((i) => (
        <motion.div
          key={i}
          className="w-3 h-3 rounded-full bg-gradient-to-r from-purple-500 to-pink-500"
          animate={{
            scale: [1, 1.5, 1],
            opacity: [0.5, 1, 0.5],
          }}
          transition={{
            duration: 1,
            repeat: Infinity,
            delay: i * 0.2,
          }}
        />
      ))}
    </div>
  );
}
