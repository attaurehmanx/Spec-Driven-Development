'use client';

import { motion } from 'framer-motion';

interface EmptyStateIllustrationProps {
  className?: string;
}

export function EmptyStateIllustration({ className = '' }: EmptyStateIllustrationProps) {
  return (
    <motion.div
      initial={{ opacity: 0, scale: 0.8 }}
      animate={{ opacity: 1, scale: 1 }}
      transition={{ duration: 0.5, ease: 'easeOut' }}
      className={`relative w-64 h-64 mx-auto ${className}`}
    >
      {/* Animated background circles */}
      <motion.div
        animate={{
          scale: [1, 1.2, 1],
          opacity: [0.3, 0.5, 0.3],
        }}
        transition={{
          duration: 3,
          repeat: Infinity,
          ease: 'easeInOut',
        }}
        className="absolute inset-0 rounded-full bg-gradient-to-br from-purple-500/30 to-pink-500/30 blur-2xl"
      />

      <motion.div
        animate={{
          scale: [1.2, 1, 1.2],
          opacity: [0.2, 0.4, 0.2],
        }}
        transition={{
          duration: 3,
          repeat: Infinity,
          ease: 'easeInOut',
          delay: 0.5,
        }}
        className="absolute inset-0 rounded-full bg-gradient-to-br from-cyan-500/30 to-blue-500/30 blur-2xl"
      />

      {/* Main illustration */}
      <svg
        viewBox="0 0 200 200"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        className="relative z-10"
      >
        {/* Clipboard */}
        <motion.g
          initial={{ y: 10, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ delay: 0.2, duration: 0.5 }}
        >
          <rect
            x="50"
            y="40"
            width="100"
            height="130"
            rx="8"
            className="fill-white dark:fill-slate-800 stroke-purple-500 dark:stroke-purple-400"
            strokeWidth="3"
          />
          <rect
            x="70"
            y="30"
            width="60"
            height="20"
            rx="6"
            className="fill-purple-500 dark:fill-purple-600"
          />
        </motion.g>

        {/* Checkboxes - animated */}
        <motion.g
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ delay: 0.4, duration: 0.5 }}
        >
          {/* Checkbox 1 */}
          <motion.rect
            x="65"
            y="70"
            width="20"
            height="20"
            rx="4"
            className="fill-gray-100 dark:fill-gray-700 stroke-gray-300 dark:stroke-gray-600"
            strokeWidth="2"
            animate={{ scale: [1, 1.1, 1] }}
            transition={{ delay: 1, duration: 0.3 }}
          />
          <motion.line
            x1="70"
            y1="80"
            x2="75"
            y2="85"
            className="stroke-gray-400 dark:stroke-gray-500"
            strokeWidth="2"
            strokeLinecap="round"
          />
          <motion.line
            x1="75"
            y1="85"
            x2="82"
            y2="73"
            className="stroke-gray-400 dark:stroke-gray-500"
            strokeWidth="2"
            strokeLinecap="round"
          />
          <line
            x1="95"
            y1="80"
            x2="130"
            y2="80"
            className="stroke-gray-300 dark:stroke-gray-600"
            strokeWidth="3"
            strokeLinecap="round"
          />

          {/* Checkbox 2 */}
          <motion.rect
            x="65"
            y="105"
            width="20"
            height="20"
            rx="4"
            className="fill-gray-100 dark:fill-gray-700 stroke-gray-300 dark:stroke-gray-600"
            strokeWidth="2"
            animate={{ scale: [1, 1.1, 1] }}
            transition={{ delay: 1.2, duration: 0.3 }}
          />
          <motion.line
            x1="70"
            y1="115"
            x2="75"
            y2="120"
            className="stroke-gray-400 dark:stroke-gray-500"
            strokeWidth="2"
            strokeLinecap="round"
          />
          <motion.line
            x1="75"
            y1="120"
            x2="82"
            y2="108"
            className="stroke-gray-400 dark:stroke-gray-500"
            strokeWidth="2"
            strokeLinecap="round"
          />
          <line
            x1="95"
            y1="115"
            x2="130"
            y2="115"
            className="stroke-gray-300 dark:stroke-gray-600"
            strokeWidth="3"
            strokeLinecap="round"
          />

          {/* Checkbox 3 - empty */}
          <motion.rect
            x="65"
            y="140"
            width="20"
            height="20"
            rx="4"
            className="fill-white dark:fill-slate-800 stroke-purple-500 dark:stroke-purple-400"
            strokeWidth="2"
            animate={{ scale: [1, 1.1, 1] }}
            transition={{ delay: 1.4, duration: 0.3 }}
          />
          <line
            x1="95"
            y1="150"
            x2="130"
            y2="150"
            className="stroke-gray-300 dark:stroke-gray-600"
            strokeWidth="3"
            strokeLinecap="round"
          />
        </motion.g>

        {/* Floating plus icon */}
        <motion.g
          initial={{ scale: 0, opacity: 0 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={{ delay: 0.8, duration: 0.5, type: 'spring' }}
        >
          <motion.circle
            cx="140"
            cy="150"
            r="20"
            className="fill-gradient-to-br from-pink-500 to-purple-500"
            style={{
              fill: 'url(#gradient)',
            }}
            animate={{
              y: [0, -5, 0],
            }}
            transition={{
              duration: 2,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
          <motion.line
            x1="140"
            y1="143"
            x2="140"
            y2="157"
            className="stroke-white"
            strokeWidth="3"
            strokeLinecap="round"
            animate={{
              y: [0, -5, 0],
            }}
            transition={{
              duration: 2,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
          <motion.line
            x1="133"
            y1="150"
            x2="147"
            y2="150"
            className="stroke-white"
            strokeWidth="3"
            strokeLinecap="round"
            animate={{
              y: [0, -5, 0],
            }}
            transition={{
              duration: 2,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
        </motion.g>

        {/* Gradient definition */}
        <defs>
          <linearGradient id="gradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#EC4899" />
            <stop offset="100%" stopColor="#8B5CF6" />
          </linearGradient>
        </defs>
      </svg>

      {/* Sparkles */}
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: [0, 1, 0] }}
        transition={{
          duration: 2,
          repeat: Infinity,
          delay: 1,
        }}
        className="absolute top-10 right-10 w-2 h-2 bg-yellow-400 rounded-full"
      />
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: [0, 1, 0] }}
        transition={{
          duration: 2,
          repeat: Infinity,
          delay: 1.5,
        }}
        className="absolute bottom-20 left-10 w-2 h-2 bg-cyan-400 rounded-full"
      />
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: [0, 1, 0] }}
        transition={{
          duration: 2,
          repeat: Infinity,
          delay: 2,
        }}
        className="absolute top-1/2 right-5 w-2 h-2 bg-pink-400 rounded-full"
      />
    </motion.div>
  );
}
