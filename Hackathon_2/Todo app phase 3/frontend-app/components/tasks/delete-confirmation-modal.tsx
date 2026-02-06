'use client';

import { motion, AnimatePresence } from 'framer-motion';
import { AlertTriangle, X, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { useState } from 'react';

interface DeleteConfirmationModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: () => Promise<void>;
  taskTitle: string;
}

export function DeleteConfirmationModal({
  isOpen,
  onClose,
  onConfirm,
  taskTitle,
}: DeleteConfirmationModalProps) {
  const [isDeleting, setIsDeleting] = useState(false);

  const handleConfirm = async () => {
    setIsDeleting(true);
    try {
      await onConfirm();
      onClose();
    } catch (error) {
      console.error('Error deleting task:', error);
    } finally {
      setIsDeleting(false);
    }
  };

  const handleClose = () => {
    if (!isDeleting) {
      onClose();
    }
  };

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Overlay */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
            className="fixed inset-0 bg-black/50 backdrop-blur-sm z-50"
            onClick={handleClose}
          />

          {/* Modal */}
          <div className="fixed inset-0 z-50 flex items-center justify-center p-4">
            <motion.div
              initial={{ opacity: 0, scale: 0.95, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.95, y: 20 }}
              transition={{ duration: 0.3, ease: 'easeOut' }}
              className="relative w-full max-w-md"
              onClick={(e) => e.stopPropagation()}
            >
              {/* Gradient border wrapper */}
              <div className="absolute inset-0 bg-gradient-to-r from-red-500 via-rose-500 to-pink-500 rounded-3xl blur-sm opacity-75"></div>

              {/* Modal content */}
              <div className="relative glass-strong rounded-3xl border-2 border-white/30 dark:border-white/20 shadow-2xl overflow-hidden">
                {/* Header with warning icon */}
                <div className="relative px-8 py-6 border-b border-gray-200 dark:border-gray-700">
                  <div className="absolute inset-0 bg-gradient-to-r from-red-500/10 via-rose-500/10 to-pink-500/10"></div>
                  <div className="relative flex items-center justify-between">
                    <div className="flex items-center gap-3">
                      <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-red-500 to-rose-500 flex items-center justify-center shadow-lg shadow-red-500/50">
                        <AlertTriangle className="w-6 h-6 text-white" />
                      </div>
                      <h2 className="text-2xl font-extrabold text-gray-900 dark:text-gray-100">
                        Delete Task
                      </h2>
                    </div>
                    <button
                      onClick={handleClose}
                      disabled={isDeleting}
                      className="p-2 rounded-xl hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors disabled:opacity-50"
                    >
                      <X className="w-6 h-6 text-gray-500 dark:text-gray-400" />
                    </button>
                  </div>
                </div>

                {/* Content */}
                <div className="p-8 space-y-4">
                  <p className="text-gray-700 dark:text-gray-300 text-lg">
                    Are you sure you want to delete this task?
                  </p>
                  <div className="p-4 rounded-xl bg-red-50 dark:bg-red-900/20 border-2 border-red-200 dark:border-red-800">
                    <p className="font-bold text-red-900 dark:text-red-100 break-words">
                      "{taskTitle}"
                    </p>
                  </div>
                  <p className="text-sm text-gray-600 dark:text-gray-400">
                    This action cannot be undone. The task will be permanently removed from your list.
                  </p>
                </div>

                {/* Action Buttons */}
                <div className="px-8 pb-8 flex gap-3">
                  <Button
                    type="button"
                    variant="outline"
                    onClick={handleClose}
                    disabled={isDeleting}
                    className="flex-1"
                  >
                    Cancel
                  </Button>
                  <Button
                    type="button"
                    variant="destructive"
                    onClick={handleConfirm}
                    disabled={isDeleting}
                    className="flex-1"
                  >
                    {isDeleting ? (
                      <>
                        <Loader2 className="w-4 h-4 animate-spin" />
                        Deleting...
                      </>
                    ) : (
                      'Delete Task'
                    )}
                  </Button>
                </div>
              </div>
            </motion.div>
          </div>
        </>
      )}
    </AnimatePresence>
  );
}
