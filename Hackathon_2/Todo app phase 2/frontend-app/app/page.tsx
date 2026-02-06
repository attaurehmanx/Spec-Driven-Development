'use client';

import { motion } from 'framer-motion';
import Link from 'next/link';

export default function HomePage() {
  return (
    <div className="min-h-screen relative overflow-hidden">
      {/* Hero Section */}
      <main className="relative z-10 max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 pt-20 pb-32">
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, ease: 'easeOut' }}
          className="text-center"
        >
          {/* Badge */}
          <motion.div
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.2, duration: 0.4 }}
            className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-gradient-to-r from-purple-500/20 to-pink-500/20 border border-purple-500/40 dark:border-purple-500/40 mb-8"
          >
            <span className="relative flex h-2 w-2">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-purple-500 opacity-75"></span>
              <span className="relative inline-flex rounded-full h-2 w-2 bg-purple-600"></span>
            </span>
            <span className="text-sm font-bold text-purple-800 dark:text-purple-300">
              AI-Powered Task Management
            </span>
          </motion.div>

          {/* Main Heading */}
          <motion.h1
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.3, duration: 0.6 }}
            className="text-5xl sm:text-6xl md:text-7xl lg:text-8xl font-extrabold tracking-tight mb-8"
          >
            <span className="block bg-gradient-to-r from-purple-600 via-pink-600 to-cyan-600 dark:from-purple-400 dark:via-pink-400 dark:to-cyan-400 bg-clip-text text-transparent">
              TaskFlow
            </span>
            <span className="block text-3xl sm:text-4xl md:text-5xl lg:text-6xl mt-4 text-gray-900 dark:text-gray-100">
              Manage Tasks with Style
            </span>
          </motion.h1>

          {/* Description */}
          <motion.p
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.4, duration: 0.6 }}
            className="mt-6 max-w-3xl mx-auto text-xl sm:text-2xl text-gray-700 dark:text-gray-300 font-medium leading-relaxed"
          >
            A fully authenticated task management application with secure API communication and AI-powered assistance.
          </motion.p>

          {/* CTA Buttons */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.5, duration: 0.6 }}
            className="mt-12 flex flex-col sm:flex-row gap-4 justify-center items-center"
          >
            <Link href="/auth/sign-in">
              <motion.button
                whileHover={{ scale: 1.05, y: -2 }}
                whileTap={{ scale: 0.95 }}
                className="group relative overflow-hidden px-10 py-5 rounded-2xl bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-700 hover:to-pink-700 text-white font-bold text-lg shadow-2xl shadow-purple-500/50 hover:shadow-glow transition-all duration-300"
              >
                <span className="absolute inset-0 bg-gradient-to-r from-white/0 via-white/20 to-white/0 translate-x-[-200%] group-hover:translate-x-[200%] transition-transform duration-700"></span>
                <span className="relative z-10 flex items-center gap-2">
                  Sign In
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M13 7l5 5m0 0l-5 5m5-5H6" />
                  </svg>
                </span>
              </motion.button>
            </Link>

            <Link href="/auth/sign-up">
              <motion.button
                whileHover={{ scale: 1.05, y: -2 }}
                whileTap={{ scale: 0.95 }}
                className="px-10 py-5 rounded-2xl border-2 border-purple-600 bg-white dark:bg-transparent hover:bg-purple-600 text-purple-700 dark:text-purple-400 hover:text-white font-bold text-lg shadow-lg hover:shadow-xl hover:shadow-purple-500/40 transition-all duration-300"
              >
                Sign Up Free
              </motion.button>
            </Link>
          </motion.div>

          {/* Feature Cards */}
          <motion.div
            initial={{ opacity: 0, y: 40 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.7, duration: 0.6 }}
            className="mt-24 grid grid-cols-1 md:grid-cols-3 gap-8"
          >
            {/* Feature 1 */}
            <motion.div
              whileHover={{ scale: 1.05, y: -5 }}
              className="relative overflow-hidden rounded-2xl glass-strong p-8 border-2 border-purple-500/40 dark:border-purple-500/40 shadow-xl"
            >
              <div className="absolute top-0 right-0 w-32 h-32 bg-gradient-to-br from-purple-500/20 to-transparent rounded-full blur-2xl"></div>
              <div className="relative z-10">
                <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-purple-600 to-pink-600 flex items-center justify-center mb-4 shadow-lg shadow-purple-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-7 w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9 12l2 2 4-4m5.618-4.016A11.955 11.955 0 0112 2.944a11.955 11.955 0 01-8.618 3.04A12.02 12.02 0 003 9c0 5.591 3.824 10.29 9 11.622 5.176-1.332 9-6.03 9-11.622 0-1.042-.133-2.052-.382-3.016z" />
                  </svg>
                </div>
                <h3 className="text-xl font-bold mb-2 text-gray-900 dark:text-gray-100">Secure & Private</h3>
                <p className="text-gray-700 dark:text-gray-300">
                  Your data is encrypted and protected with industry-standard security.
                </p>
              </div>
            </motion.div>

            {/* Feature 2 */}
            <motion.div
              whileHover={{ scale: 1.05, y: -5 }}
              className="relative overflow-hidden rounded-2xl glass-strong p-8 border-2 border-pink-500/40 dark:border-pink-500/40 shadow-xl"
            >
              <div className="absolute top-0 right-0 w-32 h-32 bg-gradient-to-br from-pink-500/20 to-transparent rounded-full blur-2xl"></div>
              <div className="relative z-10">
                <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-pink-600 to-cyan-600 flex items-center justify-center mb-4 shadow-lg shadow-pink-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-7 w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M13 10V3L4 14h7v7l9-11h-7z" />
                  </svg>
                </div>
                <h3 className="text-xl font-bold mb-2 text-gray-900 dark:text-gray-100">Lightning Fast</h3>
                <p className="text-gray-700 dark:text-gray-300">
                  Built with modern tech for blazing fast performance and responsiveness.
                </p>
              </div>
            </motion.div>

            {/* Feature 3 */}
            <motion.div
              whileHover={{ scale: 1.05, y: -5 }}
              className="relative overflow-hidden rounded-2xl glass-strong p-8 border-2 border-cyan-500/40 dark:border-cyan-500/40 shadow-xl"
            >
              <div className="absolute top-0 right-0 w-32 h-32 bg-gradient-to-br from-cyan-500/20 to-transparent rounded-full blur-2xl"></div>
              <div className="relative z-10">
                <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-cyan-600 to-green-600 flex items-center justify-center mb-4 shadow-lg shadow-cyan-500/50">
                  <svg xmlns="http://www.w3.org/2000/svg" className="h-7 w-7 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                  </svg>
                </div>
                <h3 className="text-xl font-bold mb-2 text-gray-900 dark:text-gray-100">AI-Powered</h3>
                <p className="text-gray-700 dark:text-gray-300">
                  Get intelligent suggestions and automate your workflow with AI assistance.
                </p>
              </div>
            </motion.div>
          </motion.div>
        </motion.div>
      </main>
    </div>
  );
}