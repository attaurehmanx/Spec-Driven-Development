'use client';

import { useState } from 'react';
import { motion } from 'framer-motion';
import { User, Mail, Calendar, Shield, Edit2, Copy, Check } from 'lucide-react';

interface ProfileHeaderProps {
  user: {
    first_name?: string;
    last_name?: string;
    email?: string;
    created_at?: string;
  };
}

export default function ProfileHeader({ user }: ProfileHeaderProps) {
  const [isHoveringAvatar, setIsHoveringAvatar] = useState(false);
  const [emailCopied, setEmailCopied] = useState(false);

  const initials = `${user.first_name?.[0] || ''}${user.last_name?.[0] || ''}`.toUpperCase();
  const fullName = `${user.first_name || ''} ${user.last_name || ''}`.trim() || 'User';
  const joinedDate = user.created_at
    ? new Date(user.created_at).toLocaleDateString('en-US', { month: 'short', year: 'numeric' })
    : 'Recently';

  const handleCopyEmail = () => {
    if (user.email) {
      navigator.clipboard.writeText(user.email);
      setEmailCopied(true);
      setTimeout(() => setEmailCopied(false), 2000);
    }
  };

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ delay: 0.1 }}
      className="relative overflow-hidden rounded-2xl bg-white dark:bg-slate-900 border border-gray-200 dark:border-gray-800 shadow-lg"
    >
      {/* Subtle background gradient */}
      <div className="absolute top-0 right-0 w-96 h-96 bg-gradient-to-br from-purple-500/5 to-pink-500/5 rounded-full blur-3xl -mr-48 -mt-48"></div>

      <div className="relative z-10 p-6">
        <div className="flex items-start justify-between mb-6">
          {/* Left: Avatar + Info */}
          <div className="flex items-center gap-4">
            {/* Avatar with hover effect */}
            <motion.div
              initial={{ scale: 0.9, opacity: 0 }}
              animate={{ scale: 1, opacity: 1 }}
              transition={{ delay: 0.2, type: 'spring', stiffness: 200 }}
              className="relative cursor-pointer"
              onMouseEnter={() => setIsHoveringAvatar(true)}
              onMouseLeave={() => setIsHoveringAvatar(false)}
            >
              <div className="w-20 h-20 rounded-2xl bg-gradient-to-br from-purple-600 to-pink-600 flex items-center justify-center shadow-lg shadow-purple-500/30">
                <span className="text-3xl font-extrabold text-white">{initials}</span>
              </div>

              {/* Hover overlay */}
              {isHoveringAvatar && (
                <motion.div
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  className="absolute inset-0 bg-black/70 rounded-2xl flex flex-col items-center justify-center text-white text-xs font-semibold"
                >
                  <Edit2 className="w-4 h-4 mb-1" />
                  <span>Change</span>
                </motion.div>
              )}

              {/* Status indicator - softer green */}
              <motion.div
                initial={{ scale: 0 }}
                animate={{ scale: 1 }}
                transition={{ delay: 0.4, type: 'spring' }}
                className="absolute -bottom-1 -right-1 w-7 h-7 bg-gradient-to-br from-green-400 to-emerald-400 rounded-lg flex items-center justify-center shadow-md border-2 border-white dark:border-slate-900"
              >
                <div className="w-2 h-2 bg-white rounded-full"></div>
              </motion.div>
            </motion.div>

            {/* User Info */}
            <div>
              <motion.h2
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.3 }}
                className="text-2xl font-bold text-gray-900 dark:text-gray-100 mb-1"
              >
                {fullName}
              </motion.h2>

              <motion.div
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.4 }}
                className="flex items-center gap-2 text-sm text-gray-600 dark:text-gray-400 mb-1"
              >
                <Mail className="w-4 h-4" />
                <span>{user.email}</span>
                <button
                  onClick={handleCopyEmail}
                  className="p-1 hover:bg-gray-100 dark:hover:bg-slate-800 rounded transition-colors"
                  title="Copy email"
                >
                  {emailCopied ? (
                    <Check className="w-3 h-3 text-green-500" />
                  ) : (
                    <Copy className="w-3 h-3" />
                  )}
                </button>
              </motion.div>

              <motion.div
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.5 }}
                className="flex items-center gap-4"
              >
                <span className="inline-flex items-center gap-1.5 px-3 py-1 rounded-full bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 text-green-700 dark:text-green-400 text-xs font-semibold">
                  <div className="w-1.5 h-1.5 bg-green-500 rounded-full animate-pulse"></div>
                  Active Member
                </span>
                <span className="text-xs text-gray-600 dark:text-gray-400">
                  Joined {joinedDate}
                </span>
              </motion.div>
            </div>
          </div>

          {/* Right: Edit Profile Button */}
          <motion.button
            initial={{ opacity: 0, scale: 0.9 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.6 }}
            whileHover={{ scale: 1.05, y: -2 }}
            whileTap={{ scale: 0.95 }}
            className="px-4 py-2 rounded-xl font-semibold text-white bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-500 hover:to-pink-500 shadow-lg shadow-purple-500/30 transition-all flex items-center gap-2"
          >
            <Edit2 className="w-4 h-4" />
            Edit Profile
          </motion.button>
        </div>

        {/* Two Column Layout */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6 pt-6 border-t border-gray-200 dark:border-gray-800">
          {/* Left Column: Personal Info */}
          <motion.div
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.7 }}
          >
            <div className="mb-3 px-3 py-2 rounded-lg bg-gradient-to-r from-purple-600 to-pink-600">
              <h3 className="text-sm font-extrabold text-white uppercase tracking-wider">
                Personal Information
              </h3>
            </div>
            <div className="space-y-3">
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Full Name</label>
                <p className="text-sm text-gray-900 dark:text-gray-100 font-medium">{fullName}</p>
              </div>
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Email Address</label>
                <p className="text-sm text-gray-900 dark:text-gray-100 font-medium">{user.email}</p>
              </div>
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Phone</label>
                <p className="text-sm text-gray-500 dark:text-gray-400 italic">Not provided</p>
              </div>
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Address</label>
                <p className="text-sm text-gray-500 dark:text-gray-400 italic">Not provided</p>
              </div>
            </div>
          </motion.div>

          {/* Right Column: Account Info */}
          <motion.div
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.8 }}
          >
            <div className="mb-3 px-3 py-2 rounded-lg bg-gradient-to-r from-purple-600 to-pink-600">
              <h3 className="text-sm font-extrabold text-white uppercase tracking-wider">
                Account Information
              </h3>
            </div>
            <div className="space-y-3">
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Member Since</label>
                <p className="text-sm text-gray-900 dark:text-gray-100 font-medium">
                  {user.created_at ? new Date(user.created_at).toLocaleDateString('en-US', {
                    month: 'long',
                    day: 'numeric',
                    year: 'numeric'
                  }) : 'Recently'}
                </p>
              </div>
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Account Status</label>
                <p className="text-sm text-gray-900 dark:text-gray-100 font-medium flex items-center gap-2">
                  <Shield className="w-4 h-4 text-green-500" />
                  Active & Verified
                </p>
              </div>
              <div>
                <label className="text-xs text-gray-600 dark:text-gray-400 font-medium">Plan</label>
                <p className="text-sm text-gray-900 dark:text-gray-100 font-medium">Free Tier</p>
              </div>
            </div>
          </motion.div>
        </div>
      </div>
    </motion.div>
  );
}
