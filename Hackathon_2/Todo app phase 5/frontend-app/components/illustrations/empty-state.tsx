'use client';

import { motion } from 'framer-motion';

export default function EmptyStateIllustration() {
  return (
    <svg
      className="w-64 h-64 mx-auto"
      viewBox="0 0 200 200"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Background circle */}
      <motion.circle
        cx="100"
        cy="100"
        r="80"
        fill="url(#emptyGradient)"
        opacity="0.1"
        initial={{ scale: 0 }}
        animate={{ scale: 1 }}
        transition={{ duration: 0.5, type: 'spring' }}
      />

      {/* Clipboard */}
      <motion.g
        initial={{ y: 20, opacity: 0 }}
        animate={{ y: 0, opacity: 1 }}
        transition={{ delay: 0.2, duration: 0.5 }}
      >
        {/* Clipboard board */}
        <rect
          x="60"
          y="50"
          width="80"
          height="100"
          rx="8"
          fill="url(#clipboardGradient)"
          stroke="url(#strokeGradient)"
          strokeWidth="3"
        />

        {/* Clipboard clip */}
        <rect
          x="85"
          y="45"
          width="30"
          height="12"
          rx="6"
          fill="url(#clipGradient)"
        />

        {/* Empty lines */}
        <motion.line
          x1="75"
          y1="75"
          x2="125"
          y2="75"
          stroke="#E5E7EB"
          strokeWidth="3"
          strokeLinecap="round"
          initial={{ pathLength: 0 }}
          animate={{ pathLength: 1 }}
          transition={{ delay: 0.5, duration: 0.5 }}
        />
        <motion.line
          x1="75"
          y1="90"
          x2="125"
          y2="90"
          stroke="#E5E7EB"
          strokeWidth="3"
          strokeLinecap="round"
          initial={{ pathLength: 0 }}
          animate={{ pathLength: 1 }}
          transition={{ delay: 0.6, duration: 0.5 }}
        />
        <motion.line
          x1="75"
          y1="105"
          x2="110"
          y2="105"
          stroke="#E5E7EB"
          strokeWidth="3"
          strokeLinecap="round"
          initial={{ pathLength: 0 }}
          animate={{ pathLength: 1 }}
          transition={{ delay: 0.7, duration: 0.5 }}
        />
      </motion.g>

      {/* Floating plus icon */}
      <motion.g
        initial={{ scale: 0, rotate: -180 }}
        animate={{ scale: 1, rotate: 0 }}
        transition={{ delay: 0.8, type: 'spring', stiffness: 200 }}
      >
        <circle cx="140" cy="60" r="15" fill="url(#plusGradient)" />
        <line x1="140" y1="53" x2="140" y2="67" stroke="white" strokeWidth="3" strokeLinecap="round" />
        <line x1="133" y1="60" x2="147" y2="60" stroke="white" strokeWidth="3" strokeLinecap="round" />
      </motion.g>

      {/* Floating sparkles */}
      <motion.circle
        cx="70"
        cy="70"
        r="3"
        fill="#8B5CF6"
        animate={{
          scale: [1, 1.5, 1],
          opacity: [0.5, 1, 0.5],
        }}
        transition={{
          duration: 2,
          repeat: Infinity,
          ease: 'easeInOut',
        }}
      />
      <motion.circle
        cx="130"
        cy="140"
        r="2"
        fill="#EC4899"
        animate={{
          scale: [1, 1.5, 1],
          opacity: [0.5, 1, 0.5],
        }}
        transition={{
          duration: 2,
          repeat: Infinity,
          ease: 'easeInOut',
          delay: 0.5,
        }}
      />

      {/* Gradients */}
      <defs>
        <linearGradient id="emptyGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#8B5CF6" />
          <stop offset="100%" stopColor="#EC4899" />
        </linearGradient>
        <linearGradient id="clipboardGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#FFFFFF" />
          <stop offset="100%" stopColor="#F9FAFB" />
        </linearGradient>
        <linearGradient id="strokeGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#8B5CF6" />
          <stop offset="50%" stopColor="#EC4899" />
          <stop offset="100%" stopColor="#06B6D4" />
        </linearGradient>
        <linearGradient id="clipGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#8B5CF6" />
          <stop offset="100%" stopColor="#EC4899" />
        </linearGradient>
        <linearGradient id="plusGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#84CC16" />
          <stop offset="100%" stopColor="#10B981" />
        </linearGradient>
      </defs>
    </svg>
  );
}
