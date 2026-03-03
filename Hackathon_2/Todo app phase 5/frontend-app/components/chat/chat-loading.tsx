'use client';

import React from 'react';
import { motion } from 'framer-motion';

/**
 * Typing indicator component
 * Shows animated dots while AI is generating a response
 */
export function ChatLoading() {
  const dotVariants = {
    initial: { y: 0 },
    animate: { y: -8 },
  };

  const dotTransition = {
    duration: 0.6,
    repeat: Infinity,
    repeatType: "reverse" as const,
    ease: [0.4, 0, 0.2, 1] as const, // easeInOut cubic bezier
  };

  return (
    <motion.div
      initial={{ opacity: 0, y: 10 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.3 }}
      className="flex w-full mb-4 justify-start"
    >
      <div className="max-w-[80%] rounded-xl px-4 py-3 bg-card/80 dark:bg-card/60 border border-border backdrop-blur-sm shadow-lg">
        <div className="flex items-center gap-3">
          <div className="flex gap-1.5">
            <motion.span
              variants={dotVariants}
              initial="initial"
              animate="animate"
              transition={{ ...dotTransition, delay: 0 }}
              className="w-2 h-2 bg-muted-foreground rounded-full"
            />
            <motion.span
              variants={dotVariants}
              initial="initial"
              animate="animate"
              transition={{ ...dotTransition, delay: 0.2 }}
              className="w-2 h-2 bg-muted-foreground rounded-full"
            />
            <motion.span
              variants={dotVariants}
              initial="initial"
              animate="animate"
              transition={{ ...dotTransition, delay: 0.4 }}
              className="w-2 h-2 bg-muted-foreground rounded-full"
            />
          </div>
          <span className="text-xs text-muted-foreground">AI is thinking...</span>
        </div>
      </div>
    </motion.div>
  );
}
