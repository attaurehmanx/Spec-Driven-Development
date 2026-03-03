'use client';

import { motion, AnimatePresence } from 'framer-motion';
import { CheckCircle2, XCircle, AlertCircle, X } from 'lucide-react';
import { useEffect } from 'react';

export type ToastType = 'success' | 'error' | 'warning' | 'info';

export interface ToastProps {
  id: string;
  type: ToastType;
  message: string;
  duration?: number;
  onClose: (id: string) => void;
}

const toastConfig = {
  success: {
    icon: CheckCircle2,
    gradient: 'from-green-500 to-emerald-500',
    bgColor: 'bg-green-50 dark:bg-green-900/20',
    borderColor: 'border-green-500/50',
    textColor: 'text-green-800 dark:text-green-200',
  },
  error: {
    icon: XCircle,
    gradient: 'from-red-500 to-rose-500',
    bgColor: 'bg-red-50 dark:bg-red-900/20',
    borderColor: 'border-red-500/50',
    textColor: 'text-red-800 dark:text-red-200',
  },
  warning: {
    icon: AlertCircle,
    gradient: 'from-yellow-500 to-orange-500',
    bgColor: 'bg-yellow-50 dark:bg-yellow-900/20',
    borderColor: 'border-yellow-500/50',
    textColor: 'text-yellow-800 dark:text-yellow-200',
  },
  info: {
    icon: AlertCircle,
    gradient: 'from-blue-500 to-cyan-500',
    bgColor: 'bg-blue-50 dark:bg-blue-900/20',
    borderColor: 'border-blue-500/50',
    textColor: 'text-blue-800 dark:text-blue-200',
  },
};

export function Toast({ id, type, message, duration = 5000, onClose }: ToastProps) {
  const config = toastConfig[type];
  const Icon = config.icon;

  useEffect(() => {
    const timer = setTimeout(() => {
      onClose(id);
    }, duration);

    return () => clearTimeout(timer);
  }, [id, duration, onClose]);

  return (
    <motion.div
      initial={{ opacity: 0, y: -50, scale: 0.9 }}
      animate={{ opacity: 1, y: 0, scale: 1 }}
      exit={{ opacity: 0, x: 100, scale: 0.9 }}
      transition={{ duration: 0.3, ease: 'easeOut' }}
      className={`flex items-center gap-3 p-4 rounded-2xl ${config.bgColor} border-2 ${config.borderColor} shadow-2xl backdrop-blur-xl min-w-[320px] max-w-md`}
    >
      <div className={`w-10 h-10 rounded-xl bg-gradient-to-br ${config.gradient} flex items-center justify-center shadow-lg flex-shrink-0`}>
        <Icon className="w-5 h-5 text-white" />
      </div>
      <p className={`flex-1 font-medium text-sm ${config.textColor}`}>{message}</p>
      <button
        onClick={() => onClose(id)}
        className={`${config.textColor} hover:opacity-70 transition-opacity flex-shrink-0`}
      >
        <X className="w-5 h-5" />
      </button>
    </motion.div>
  );
}

export interface ToastContainerProps {
  toasts: Array<{
    id: string;
    type: ToastType;
    message: string;
    duration?: number;
  }>;
  onClose: (id: string) => void;
}

export function ToastContainer({ toasts, onClose }: ToastContainerProps) {
  return (
    <div className="fixed top-20 right-6 z-[100] flex flex-col gap-3">
      <AnimatePresence mode="popLayout">
        {toasts.map((toast) => (
          <Toast key={toast.id} {...toast} onClose={onClose} />
        ))}
      </AnimatePresence>
    </div>
  );
}
