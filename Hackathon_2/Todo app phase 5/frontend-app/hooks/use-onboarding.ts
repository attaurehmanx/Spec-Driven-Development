import { useState, useEffect } from 'react';

interface OnboardingStep {
  id: string;
  title: string;
  description: string;
  target?: string;
  position?: 'top' | 'bottom' | 'left' | 'right';
}

const ONBOARDING_STEPS: OnboardingStep[] = [
  {
    id: 'welcome',
    title: 'Welcome to Task Manager!',
    description: 'Let\'s take a quick tour to help you get started with managing your tasks efficiently.',
  },
  {
    id: 'create-task',
    title: 'Create Your First Task',
    description: 'Click here to create a new task. Add a title, description, and priority to stay organized.',
    target: '[data-tour="create-task"]',
    position: 'bottom',
  },
  {
    id: 'task-list',
    title: 'Your Task List',
    description: 'All your tasks appear here. You can filter by status, mark tasks as complete, or delete them.',
    target: '[data-tour="task-list"]',
    position: 'top',
  },
  {
    id: 'profile',
    title: 'View Your Profile',
    description: 'Check your productivity stats, manage settings, and track your progress here.',
    target: '[data-tour="profile"]',
    position: 'bottom',
  },
];

const ONBOARDING_KEY = 'task-manager-onboarding-completed';

export function useOnboarding() {
  const [isActive, setIsActive] = useState(false);
  const [currentStep, setCurrentStep] = useState(0);
  const [isCompleted, setIsCompleted] = useState(false);

  useEffect(() => {
    // Check if onboarding has been completed
    const completed = localStorage.getItem(ONBOARDING_KEY);
    if (completed === 'true') {
      setIsCompleted(true);
    }
  }, []);

  const startTour = () => {
    setIsActive(true);
    setCurrentStep(0);
  };

  const nextStep = () => {
    if (currentStep < ONBOARDING_STEPS.length - 1) {
      setCurrentStep(currentStep + 1);
    } else {
      completeTour();
    }
  };

  const previousStep = () => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  };

  const skipTour = () => {
    completeTour();
  };

  const completeTour = () => {
    setIsActive(false);
    setIsCompleted(true);
    localStorage.setItem(ONBOARDING_KEY, 'true');
  };

  const resetTour = () => {
    localStorage.removeItem(ONBOARDING_KEY);
    setIsCompleted(false);
    setCurrentStep(0);
  };

  return {
    isActive,
    currentStep,
    isCompleted,
    steps: ONBOARDING_STEPS,
    currentStepData: ONBOARDING_STEPS[currentStep],
    totalSteps: ONBOARDING_STEPS.length,
    startTour,
    nextStep,
    previousStep,
    skipTour,
    completeTour,
    resetTour,
  };
}
