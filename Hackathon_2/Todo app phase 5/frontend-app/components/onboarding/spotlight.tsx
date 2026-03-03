'use client';

import { motion, AnimatePresence } from 'framer-motion';
import { useEffect, useState } from 'react';

interface SpotlightProps {
  target: string | null;
  isActive: boolean;
}

export default function Spotlight({ target, isActive }: SpotlightProps) {
  const [position, setPosition] = useState({ top: 0, left: 0, width: 0, height: 0 });

  useEffect(() => {
    if (!target || !isActive) return;

    const updatePosition = () => {
      const element = document.querySelector(target);
      if (element) {
        const rect = element.getBoundingClientRect();
        setPosition({
          top: rect.top + window.scrollY,
          left: rect.left + window.scrollX,
          width: rect.width,
          height: rect.height,
        });
      }
    };

    updatePosition();
    window.addEventListener('resize', updatePosition);
    window.addEventListener('scroll', updatePosition);

    return () => {
      window.removeEventListener('resize', updatePosition);
      window.removeEventListener('scroll', updatePosition);
    };
  }, [target, isActive]);

  if (!isActive) return null;

  return (
    <AnimatePresence>
      {target && (
        <>
          {/* Dark overlay */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/60 backdrop-blur-sm z-[9998]"
            style={{ pointerEvents: 'none' }}
          />

          {/* Spotlight cutout */}
          <motion.div
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            exit={{ opacity: 0, scale: 0.8 }}
            transition={{ type: 'spring', stiffness: 300, damping: 30 }}
            className="fixed z-[9999] pointer-events-none"
            style={{
              top: position.top - 8,
              left: position.left - 8,
              width: position.width + 16,
              height: position.height + 16,
              boxShadow: '0 0 0 9999px rgba(0, 0, 0, 0.6), 0 0 40px 10px rgba(139, 92, 246, 0.5)',
              borderRadius: '16px',
              border: '3px solid rgba(139, 92, 246, 0.8)',
            }}
          >
            {/* Animated pulse ring */}
            <motion.div
              className="absolute inset-0 rounded-2xl border-4 border-purple-500"
              animate={{
                scale: [1, 1.05, 1],
                opacity: [0.5, 0.8, 0.5],
              }}
              transition={{
                duration: 2,
                repeat: Infinity,
                ease: 'easeInOut',
              }}
            />
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}
