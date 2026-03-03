'use client';

import { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { Moon, Sun, Bell, Lock, User, LogOut, AlertTriangle, X } from 'lucide-react';
import { useTheme } from 'next-themes';
import { useRouter } from 'next/navigation';

export default function SettingsPanel() {
  const { theme, setTheme } = useTheme();
  const router = useRouter();
  const [showDeleteModal, setShowDeleteModal] = useState(false);
  const [deleteConfirmation, setDeleteConfirmation] = useState('');
  const [isDeleting, setIsDeleting] = useState(false);

  const handleLogout = () => {
    localStorage.removeItem('token');
    router.push('/auth/sign-in');
  };

  const handleDeleteAccount = async () => {
    if (deleteConfirmation !== 'DELETE') return;

    setIsDeleting(true);
    // Simulate API call
    setTimeout(() => {
      localStorage.removeItem('token');
      router.push('/auth/sign-in');
    }, 1500);
  };

  const settingsSections = [
    {
      title: 'Appearance',
      icon: theme === 'dark' ? Moon : Sun,
      items: [
        {
          label: 'Theme',
          description: 'Switch between light and dark mode',
          action: (
            <button
              onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}
              className="relative w-16 h-8 rounded-full bg-gradient-to-r from-purple-500 to-pink-500 shadow-lg transition-all hover:shadow-purple-500/50"
            >
              <motion.div
                className="absolute top-1 left-1 w-6 h-6 bg-white rounded-full shadow-md flex items-center justify-center"
                animate={{ x: theme === 'dark' ? 32 : 0 }}
                transition={{ type: 'spring', stiffness: 300, damping: 30 }}
              >
                {theme === 'dark' ? (
                  <Moon className="w-4 h-4 text-purple-600" />
                ) : (
                  <Sun className="w-4 h-4 text-yellow-600" />
                )}
              </motion.div>
            </button>
          ),
        },
      ],
    },
    {
      title: 'Notifications',
      icon: Bell,
      items: [
        {
          label: 'Task Reminders',
          description: 'Get notified about pending tasks',
          action: (
            <button className="relative w-16 h-8 rounded-full bg-gray-300 dark:bg-gray-700 shadow-inner">
              <div className="absolute top-1 left-1 w-6 h-6 bg-white rounded-full shadow-md"></div>
            </button>
          ),
        },
      ],
    },
    {
      title: 'Security',
      icon: Lock,
      items: [
        {
          label: 'Password',
          description: 'Change your password',
          action: (
            <button className="px-4 py-2 rounded-lg text-sm font-semibold text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors">
              Change
            </button>
          ),
        },
      ],
    },
  ];

  return (
    <>
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.7 }}
        className="space-y-6"
      >
        {/* Settings Sections */}
        {settingsSections.map((section, sectionIndex) => {
          const SectionIcon = section.icon;
          return (
            <motion.div
              key={section.title}
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.8 + sectionIndex * 0.1 }}
              className="bg-white dark:bg-slate-900 rounded-2xl p-6 border border-gray-200 dark:border-gray-800 shadow-sm"
            >
              <div className="flex items-center gap-3 mb-4">
                <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-purple-500 to-pink-500 flex items-center justify-center shadow-lg shadow-purple-500/30">
                  <SectionIcon className="w-5 h-5 text-white" />
                </div>
                <h3 className="text-xl font-bold text-gray-900 dark:text-gray-100">
                  {section.title}
                </h3>
              </div>

              <div className="space-y-3">
                {section.items.map((item, itemIndex) => (
                  <motion.div
                    key={item.label}
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    transition={{ delay: 0.9 + sectionIndex * 0.1 + itemIndex * 0.05 }}
                    className="flex items-center justify-between p-4 rounded-xl bg-gray-50 dark:bg-slate-800/50 hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors"
                  >
                    <div className="flex-1">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-1">
                        {item.label}
                      </h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        {item.description}
                      </p>
                    </div>
                    <div className="ml-4">
                      {item.action}
                    </div>
                  </motion.div>
                ))}
              </div>
            </motion.div>
          );
        })}

        {/* Sign Out Button - Neutral */}
        <motion.button
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.1 }}
          whileHover={{ scale: 1.02, y: -2 }}
          whileTap={{ scale: 0.98 }}
          onClick={handleLogout}
          className="w-full px-8 py-4 rounded-2xl font-bold text-gray-700 dark:text-gray-300 bg-white dark:bg-slate-900 border-2 border-gray-300 dark:border-gray-700 hover:border-gray-400 dark:hover:border-gray-600 shadow-sm hover:shadow-md transition-all flex items-center justify-center gap-2"
        >
          <LogOut className="w-5 h-5" />
          Sign Out
        </motion.button>

        {/* Danger Zone */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.2 }}
          className="mt-12 pt-8 border-t-2 border-red-200 dark:border-red-900/30"
        >
          <div className="bg-red-50 dark:bg-red-900/10 rounded-2xl p-6 border-2 border-red-200 dark:border-red-900/30">
            <div className="flex items-start gap-3 mb-4">
              <div className="w-10 h-10 rounded-xl bg-red-500 flex items-center justify-center shadow-lg shadow-red-500/30 flex-shrink-0">
                <AlertTriangle className="w-5 h-5 text-white" />
              </div>
              <div>
                <h3 className="text-xl font-bold text-red-900 dark:text-red-400 mb-2">
                  Danger Zone
                </h3>
                <p className="text-sm text-red-700 dark:text-red-400 leading-relaxed">
                  Once you delete your account, there is no going back. This action is <strong>irreversible</strong> and all your data will be <strong>permanently deleted</strong>.
                </p>
              </div>
            </div>

            <button
              onClick={() => setShowDeleteModal(true)}
              className="w-full mt-4 px-6 py-3 rounded-xl font-bold text-white bg-red-600 hover:bg-red-700 shadow-lg shadow-red-500/30 hover:shadow-red-500/50 transition-all flex items-center justify-center gap-2"
            >
              <AlertTriangle className="w-5 h-5" />
              Delete Account
            </button>
          </div>
        </motion.div>
      </motion.div>

      {/* Delete Confirmation Modal */}
      <AnimatePresence>
        {showDeleteModal && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4"
            onClick={() => !isDeleting && setShowDeleteModal(false)}
          >
            <motion.div
              initial={{ scale: 0.95, opacity: 0 }}
              animate={{ scale: 1, opacity: 1 }}
              exit={{ scale: 0.95, opacity: 0 }}
              onClick={(e) => e.stopPropagation()}
              className="bg-white dark:bg-slate-900 rounded-3xl p-8 max-w-md w-full shadow-2xl border-2 border-red-500"
            >
              {/* Close button */}
              <button
                onClick={() => !isDeleting && setShowDeleteModal(false)}
                className="absolute top-4 right-4 p-2 rounded-xl hover:bg-gray-100 dark:hover:bg-slate-800 transition-colors"
                disabled={isDeleting}
              >
                <X className="w-5 h-5 text-gray-500" />
              </button>

              {/* Warning Icon */}
              <div className="mx-auto w-16 h-16 rounded-2xl bg-red-100 dark:bg-red-900/30 flex items-center justify-center mb-4">
                <AlertTriangle className="w-8 h-8 text-red-600 dark:text-red-400" />
              </div>

              {/* Title */}
              <h2 className="text-2xl font-bold text-gray-900 dark:text-gray-100 text-center mb-2">
                Delete Account?
              </h2>

              {/* Warning Text */}
              <p className="text-gray-600 dark:text-gray-400 text-center mb-6">
                This action <strong className="text-red-600 dark:text-red-400">cannot be undone</strong>. All your tasks, data, and account information will be permanently deleted.
              </p>

              {/* Confirmation Input */}
              <div className="mb-6">
                <label className="block text-sm font-semibold text-gray-900 dark:text-gray-100 mb-2">
                  Type <span className="text-red-600 dark:text-red-400 font-mono">DELETE</span> to confirm:
                </label>
                <input
                  type="text"
                  value={deleteConfirmation}
                  onChange={(e) => setDeleteConfirmation(e.target.value)}
                  placeholder="DELETE"
                  disabled={isDeleting}
                  className="w-full px-4 py-3 rounded-xl border-2 border-gray-300 dark:border-gray-700 bg-white dark:bg-slate-800 text-gray-900 dark:text-gray-100 placeholder:text-gray-400 focus:outline-none focus:ring-2 focus:ring-red-500 focus:border-transparent transition-all font-mono"
                />
              </div>

              {/* Action Buttons */}
              <div className="flex gap-3">
                <button
                  onClick={() => setShowDeleteModal(false)}
                  disabled={isDeleting}
                  className="flex-1 px-6 py-3 rounded-xl font-bold text-gray-700 dark:text-gray-300 bg-gray-100 dark:bg-slate-800 hover:bg-gray-200 dark:hover:bg-slate-700 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  Cancel
                </button>
                <button
                  onClick={handleDeleteAccount}
                  disabled={deleteConfirmation !== 'DELETE' || isDeleting}
                  className="flex-1 px-6 py-3 rounded-xl font-bold text-white bg-red-600 hover:bg-red-700 disabled:bg-gray-400 disabled:cursor-not-allowed shadow-lg shadow-red-500/30 transition-all"
                >
                  {isDeleting ? 'Deleting...' : 'Delete Forever'}
                </button>
              </div>
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
}
