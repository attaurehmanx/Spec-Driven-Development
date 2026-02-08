'use client';

import { motion } from 'framer-motion';
import { CheckCircle2, Clock, ListTodo, TrendingUp, Target, Zap } from 'lucide-react';

interface StatsGridProps {
  stats: {
    total: number;
    completed: number;
    pending: number;
  };
  loading?: boolean;
}

const statCards = [
  {
    key: 'total',
    label: 'Total Tasks',
    icon: ListTodo,
    gradient: 'from-purple-500 to-pink-500',
    shadowColor: 'shadow-purple-500/40',
    glowClass: 'hover:shadow-glow',
  },
  {
    key: 'completed',
    label: 'Completed',
    icon: CheckCircle2,
    gradient: 'from-green-500 to-emerald-500',
    shadowColor: 'shadow-green-500/40',
    glowClass: 'hover:shadow-glow-green',
  },
  {
    key: 'pending',
    label: 'In Progress',
    icon: Clock,
    gradient: 'from-yellow-500 to-orange-500',
    shadowColor: 'shadow-yellow-500/40',
    glowClass: 'hover:shadow-glow-pink',
  },
];

export default function StatsGrid({ stats, loading = false }: StatsGridProps) {
  const completionRate = stats.total > 0 ? Math.round((stats.completed / stats.total) * 100) : 0;

  return (
    <div className="space-y-6">
      {/* Main Stats Grid */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.2 }}
        className="grid grid-cols-1 md:grid-cols-3 gap-6"
      >
        {statCards.map((card, index) => {
          const Icon = card.icon;
          const value = stats[card.key as keyof typeof stats];

          return (
            <motion.div
              key={card.key}
              initial={{ opacity: 0, scale: 0.9, y: 20 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              transition={{ delay: 0.3 + index * 0.1, type: 'spring', stiffness: 200 }}
              whileHover={{ scale: 1.03, y: -5 }}
              className={`relative overflow-hidden rounded-2xl bg-gradient-to-br ${card.gradient} p-8 shadow-2xl ${card.shadowColor} ${card.glowClass} transition-all duration-300`}
            >
              {/* Decorative orb */}
              <div className="absolute top-0 right-0 w-32 h-32 bg-white/10 rounded-full -mr-16 -mt-16"></div>

              <div className="relative z-10">
                <div className="flex items-center justify-between mb-4">
                  <div className="w-14 h-14 bg-white/20 backdrop-blur-sm rounded-xl flex items-center justify-center shadow-lg">
                    <Icon className="w-7 h-7 text-white" />
                  </div>
                </div>
                <p className="text-white/90 text-sm font-bold uppercase tracking-wider mb-2">
                  {card.label}
                </p>
                {loading ? (
                  <p className="text-5xl font-extrabold text-white animate-pulse">...</p>
                ) : (
                  <motion.p
                    initial={{ opacity: 0, scale: 0.5 }}
                    animate={{ opacity: 1, scale: 1 }}
                    transition={{ delay: 0.5 + index * 0.1, type: 'spring' }}
                    className="text-5xl font-extrabold text-white"
                  >
                    {value}
                  </motion.p>
                )}
              </div>
            </motion.div>
          );
        })}
      </motion.div>

      {/* Additional Stats */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.6 }}
        className="grid grid-cols-1 md:grid-cols-2 gap-6"
      >
        {/* Completion Rate */}
        <motion.div
          whileHover={{ scale: 1.02, y: -3 }}
          className="glass-strong rounded-2xl p-6 border-2 border-white/30 dark:border-white/20 shadow-xl"
        >
          <div className="flex items-center gap-4">
            <div className="w-16 h-16 rounded-xl bg-gradient-to-br from-cyan-500 to-blue-500 flex items-center justify-center shadow-lg shadow-cyan-500/50">
              <Target className="w-8 h-8 text-white" />
            </div>
            <div className="flex-1">
              <h3 className="text-sm font-medium text-gray-500 dark:text-gray-400 mb-1">
                Completion Rate
              </h3>
              <div className="flex items-baseline gap-2">
                <p className="text-3xl font-extrabold bg-gradient-to-r from-cyan-600 to-blue-600 dark:from-cyan-400 dark:to-blue-400 bg-clip-text text-transparent">
                  {completionRate}%
                </p>
                {completionRate > 50 && (
                  <TrendingUp className="w-5 h-5 text-green-500" />
                )}
              </div>
            </div>
          </div>
          {/* Progress bar */}
          <div className="mt-4 h-3 bg-gray-200 dark:bg-gray-800 rounded-full overflow-hidden">
            <motion.div
              initial={{ width: 0 }}
              animate={{ width: `${completionRate}%` }}
              transition={{ delay: 0.8, duration: 1, ease: 'easeOut' }}
              className="h-full bg-gradient-to-r from-cyan-500 to-blue-500 rounded-full"
            />
          </div>
        </motion.div>

        {/* Productivity Score */}
        <motion.div
          whileHover={{ scale: 1.02, y: -3 }}
          className="glass-strong rounded-2xl p-6 border-2 border-white/30 dark:border-white/20 shadow-xl"
        >
          <div className="flex items-center gap-4">
            <div className="w-16 h-16 rounded-xl bg-gradient-to-br from-pink-500 to-rose-500 flex items-center justify-center shadow-lg shadow-pink-500/50">
              <Zap className="w-8 h-8 text-white" />
            </div>
            <div className="flex-1">
              <h3 className="text-sm font-medium text-gray-500 dark:text-gray-400 mb-1">
                Productivity Score
              </h3>
              <div className="flex items-baseline gap-2">
                <p className="text-3xl font-extrabold bg-gradient-to-r from-pink-600 to-rose-600 dark:from-pink-400 dark:to-rose-400 bg-clip-text text-transparent">
                  {Math.min(100, stats.completed * 10)}
                </p>
                <span className="text-sm font-medium text-gray-500 dark:text-gray-400">
                  / 100
                </span>
              </div>
            </div>
          </div>
          {/* Progress bar */}
          <div className="mt-4 h-3 bg-gray-200 dark:bg-gray-800 rounded-full overflow-hidden">
            <motion.div
              initial={{ width: 0 }}
              animate={{ width: `${Math.min(100, stats.completed * 10)}%` }}
              transition={{ delay: 0.9, duration: 1, ease: 'easeOut' }}
              className="h-full bg-gradient-to-r from-pink-500 to-rose-500 rounded-full"
            />
          </div>
        </motion.div>
      </motion.div>
    </div>
  );
}
