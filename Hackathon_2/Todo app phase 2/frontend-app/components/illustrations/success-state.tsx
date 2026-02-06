'use client';

import { motion } from 'framer-motion';

export default function SuccessStateIllustration() {
  return (
    <svg
      className="w-64 h-64 mx-auto"
      viewBox="0 0 200 200"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Background circle with pulse */}
      <motion.circle
        cx="100"
        cy="100"
        r="80"
        fill="url(#successGradient)"
        opacity="0.2"
        animate={{
          scale: [1, 1.1, 1],
          opacity: [0.2, 0.3, 0.2],
        }}
        transition={{
          duration: 2,
          repeat: Infinity,
          ease: 'easeInOut',
        }}
      />

      {/* Trophy base */}
      <motion.g
        initial={{ y: 20, opacity: 0 }}
        animate={{ y: 0, opacity: 1 }}
        transition={{ delay: 0.2, duration: 0.5 }}
      >
        {/* Base platform */}
        <rect
          x="70"
          y="140"
          width="60"
          height="8"
          rx="4"
          fill="url(#baseGradient)"
        />
        <rect
          x="80"
          y="135"
          width="40"
          height="5"
          rx="2.5"
          fill="url(#baseGradient)"
        />

        {/* Trophy stem */}
        <rect
          x="92"
          y="115"
          width="16"
          height="20"
          rx="2"
          fill="url(#stemGradient)"
        />
      </motion.g>

      {/* Trophy cup */}
      <motion.g
        initial={{ scale: 0, y: -20 }}
        animate={{ scale: 1, y: 0 }}
        transition={{ delay: 0.4, type: 'spring', stiffness: 200 }}
      >
        {/* Cup body */}
        <path
          d="M 70 70 Q 70 50 100 50 Q 130 50 130 70 L 125 110 Q 125 120 100 120 Q 75 120 75 110 Z"
          fill="url(#cupGradient)"
          stroke="url(#cupStroke)"
          strokeWidth="3"
        />

        {/* Left handle */}
        <path
          d="M 70 65 Q 55 65 55 80 Q 55 95 70 95"
          stroke="url(#handleGradient)"
          strokeWidth="4"
          fill="none"
          strokeLinecap="round"
        />

        {/* Right handle */}
        <path
          d="M 130 65 Q 145 65 145 80 Q 145 95 130 95"
          stroke="url(#handleGradient)"
          strokeWidth="4"
          fill="none"
          strokeLinecap="round"
        />
      </motion.g>

      {/* Checkmark inside trophy */}
      <motion.path
        d="M 90 80 L 97 87 L 110 70"
        stroke="white"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
        fill="none"
        initial={{ pathLength: 0 }}
        animate={{ pathLength: 1 }}
        transition={{ delay: 0.8, duration: 0.5 }}
      />

      {/* Sparkles around trophy */}
      <motion.g
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ delay: 1 }}
      >
        {/* Top sparkle */}
        <motion.g
          animate={{
            scale: [1, 1.3, 1],
            rotate: [0, 180, 360],
          }}
          transition={{
            duration: 3,
            repeat: Infinity,
            ease: 'easeInOut',
          }}
        >
          <line x1="100" y1="35" x2="100" y2="45" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
          <line x1="95" y1="40" x2="105" y2="40" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
        </motion.g>

        {/* Left sparkle */}
        <motion.g
          animate={{
            scale: [1, 1.2, 1],
            rotate: [0, 180, 360],
          }}
          transition={{
            duration: 2.5,
            repeat: Infinity,
            ease: 'easeInOut',
            delay: 0.5,
          }}
        >
          <line x1="50" y1="70" x2="50" y2="78" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
          <line x1="46" y1="74" x2="54" y2="74" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
        </motion.g>

        {/* Right sparkle */}
        <motion.g
          animate={{
            scale: [1, 1.2, 1],
            rotate: [0, 180, 360],
          }}
          transition={{
            duration: 2.5,
            repeat: Infinity,
            ease: 'easeInOut',
            delay: 1,
          }}
        >
          <line x1="150" y1="70" x2="150" y2="78" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
          <line x1="146" y1="74" x2="154" y2="74" stroke="#FFD700" strokeWidth="2" strokeLinecap="round" />
        </motion.g>
      </motion.g>

      {/* Confetti particles */}
      {[...Array(8)].map((_, i) => (
        <motion.circle
          key={i}
          cx={80 + Math.random() * 40}
          cy={50 + Math.random() * 30}
          r="2"
          fill={['#8B5CF6', '#EC4899', '#06B6D4', '#84CC16'][i % 4]}
          initial={{ y: 0, opacity: 1 }}
          animate={{
            y: [0, 100],
            opacity: [1, 0],
          }}
          transition={{
            duration: 2,
            repeat: Infinity,
            delay: i * 0.2,
            ease: 'easeOut',
          }}
        />
      ))}

      {/* Gradients */}
      <defs>
        <linearGradient id="successGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#84CC16" />
          <stop offset="100%" stopColor="#10B981" />
        </linearGradient>
        <linearGradient id="cupGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#FFD700" />
          <stop offset="100%" stopColor="#FFA500" />
        </linearGradient>
        <linearGradient id="cupStroke" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#FFA500" />
          <stop offset="100%" stopColor="#FF8C00" />
        </linearGradient>
        <linearGradient id="handleGradient" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#FFD700" />
          <stop offset="100%" stopColor="#FFA500" />
        </linearGradient>
        <linearGradient id="stemGradient" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#FFA500" />
          <stop offset="100%" stopColor="#FF8C00" />
        </linearGradient>
        <linearGradient id="baseGradient" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#D1D5DB" />
          <stop offset="100%" stopColor="#9CA3AF" />
        </linearGradient>
      </defs>
    </svg>
  );
}
