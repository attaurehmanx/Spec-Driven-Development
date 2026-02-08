'use client';

import { motion, AnimatePresence } from 'framer-motion';
import { X, ChevronRight, ChevronLeft, Sparkles } from 'lucide-react';
import { useOnboarding } from '@/hooks/use-onboarding';
import Spotlight from './spotlight';
import { useEffect, useState } from 'react';

interface TooltipPosition {
  top?: number;
  left?: number;
  bottom?: number;
  right?: number;
}

export default function OnboardingTour() {
  const {
    isActive,
    currentStep,
    currentStepData,
    totalSteps,
    nextStep,
    previousStep,
    skipTour,
  } = useOnboarding();

  const [tooltipPosition, setTooltipPosition] = useState<TooltipPosition>({});

  useEffect(() => {
    if (!currentStepData?.target || !isActive) return;

    const updateTooltipPosition = () => {
      const element = document.querySelector(currentStepData.target!);
      if (element) {
        const rect = element.getBoundingClientRect();
        const position = currentStepData.position || 'bottom';

        let newPosition: TooltipPosition = {};

        switch (position) {
          case 'top':
            newPosition = {
              bottom: window.innerHeight - rect.top + 20,
              left: rect.left + rect.width / 2,
            };
            break;
          case 'bottom':
            newPosition = {
              top: rect.bottom + 20,
              left: rect.left + rect.width / 2,
            };
            break;
          case 'left':
            newPosition = {
              top: rect.top + rect.height / 2,
              right: window.innerWidth - rect.left + 20,
            };
            break;
          case 'right':
            newPosition = {
              top: rect.top + rect.height / 2,
              left: rect.right + 20,
            };
            break;
        }

        setTooltipPosition(newPosition);
      }
    };

    updateTooltipPosition();
    window.addEventListener('resize', updateTooltipPosition);
    window.addEventListener('scroll', updateTooltipPosition);

    return () => {
      window.removeEventListener('resize', updateTooltipPosition);
      window.removeEventListener('scroll', updateTooltipPosition);
    };
  }, [currentStepData, isActive]);

  if (!isActive) return null;

  return (
    <>
      {/* Spotlight effect */}
      <Spotlight target={currentStepData?.target || null} isActive={isActive} />

      {/* Tooltip */}
      <AnimatePresence mode="wait">
        <motion.div
          key={currentStep}
          initial={{ opacity: 0, scale: 0.9, y: 20 }}
          animate={{ opacity: 1, scale: 1, y: 0 }}
          exit={{ opacity: 0, scale: 0.9, y: -20 }}
          transition={{ type: 'spring', stiffness: 300, damping: 30 }}
          className="fixed z-[10000] pointer-events-auto"
          style={{
            ...tooltipPosition,
            transform: currentStepData?.target ? 'translateX(-50%)' : 'none',
            maxWidth: '400px',
          }}
        >
          <div className="glass-strong rounded-2xl p-6 border-2 border-purple-500/50 shadow-2xl shadow-purple-500/50">
            {/* Header */}
            <div className="flex items-start justify-between mb-4">
              <div className="flex items-center gap-2">
                <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-purple-500 to-pink-500 flex items-center justify-center shadow-lg shadow-purple-500/50">
                  <Sparkles className="w-5 h-5 text-white" />
                </div>
                <div>
                  <h3 className="text-lg font-bold text-gray-900 dark:text-gray-100">
                    {currentStepData?.title}
                  </h3>
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    Step {currentStep + 1} of {totalSteps}
                  </p>
                </div>
              </div>
              <button
                onClick={skipTour}
                className="text-gray-400 hover:text-gray-600 dark:hover:text-gray-300 transition-colors"
              >
                <X className="w-5 h-5" />
              </button>
            </div>

            {/* Description */}
            <p className="text-gray-700 dark:text-gray-300 mb-6">
              {currentStepData?.description}
            </p>

            {/* Progress bar */}
            <div className="mb-4">
              <div className="h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                <motion.div
                  className="h-full bg-gradient-to-r from-purple-500 to-pink-500"
                  initial={{ width: 0 }}
                  animate={{ width: `${((currentStep + 1) / totalSteps) * 100}%` }}
                  transition={{ duration: 0.3 }}
                />
              </div>
            </div>

            {/* Navigation buttons */}
            <div className="flex items-center justify-between gap-3">
              <button
                onClick={previousStep}
                disabled={currentStep === 0}
                className={`flex items-center gap-2 px-4 py-2 rounded-xl font-medium transition-all ${
                  currentStep === 0
                    ? 'text-gray-400 cursor-not-allowed'
                    : 'text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-800'
                }`}
              >
                <ChevronLeft className="w-4 h-4" />
                Back
              </button>

              <div className="flex gap-2">
                <button
                  onClick={skipTour}
                  className="px-4 py-2 rounded-xl font-medium text-gray-600 dark:text-gray-400 hover:bg-gray-100 dark:hover:bg-gray-800 transition-all"
                >
                  Skip
                </button>
                <motion.button
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                  onClick={nextStep}
                  className="flex items-center gap-2 px-6 py-2 rounded-xl font-bold text-white bg-gradient-to-r from-purple-500 to-pink-500 hover:from-purple-400 hover:to-pink-400 shadow-lg shadow-purple-500/50 transition-all"
                >
                  {currentStep === totalSteps - 1 ? 'Finish' : 'Next'}
                  <ChevronRight className="w-4 h-4" />
                </motion.button>
              </div>
            </div>
          </div>
        </motion.div>
      </AnimatePresence>
    </>
  );
}
