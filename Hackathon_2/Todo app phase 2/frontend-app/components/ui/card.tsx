import React from 'react';
import { motion } from 'framer-motion';
import { cn } from '@/lib/utils';

interface CardProps {
  children: React.ReactNode;
  className?: string;
  title?: string;
  description?: string;
  variant?: 'default' | 'glass' | 'gradient-purple' | 'gradient-pink' | 'gradient-cyan' | 'gradient-green';
}

const Card: React.FC<CardProps> = ({
  children,
  className = '',
  title,
  description,
  variant = 'default'
}) => {
  const baseClasses = 'rounded-2xl border text-card-foreground shadow-2xl transition-all duration-300 hover:shadow-glow hover:-translate-y-1';

  const variantClasses = {
    default: 'bg-card border-border/50',
    glass: 'bg-white/90 dark:bg-slate-900/90 backdrop-blur-2xl border-white/30 dark:border-white/20',
    'gradient-purple': 'bg-gradient-to-br from-primary-500/20 via-primary-400/10 to-transparent dark:from-primary-600/30 dark:via-primary-500/20 border-primary-400/30 dark:border-primary-500/40 shadow-glow',
    'gradient-pink': 'bg-gradient-to-br from-secondary-500/20 via-secondary-400/10 to-transparent dark:from-secondary-600/30 dark:via-secondary-500/20 border-secondary-400/30 dark:border-secondary-500/40 shadow-glow-pink',
    'gradient-cyan': 'bg-gradient-to-br from-accent-500/20 via-accent-400/10 to-transparent dark:from-accent-600/30 dark:via-accent-500/20 border-accent-400/30 dark:border-accent-500/40 shadow-glow-cyan',
    'gradient-green': 'bg-gradient-to-br from-success-500/20 via-success-400/10 to-transparent dark:from-success-600/30 dark:via-success-500/20 border-success-400/30 dark:border-success-500/40 shadow-glow-green',
  };

  return (
    <motion.div
      className={cn(baseClasses, variantClasses[variant], className)}
      initial={{ opacity: 0, y: 20, scale: 0.95 }}
      animate={{ opacity: 1, y: 0, scale: 1 }}
      transition={{ duration: 0.4, ease: [0.4, 0, 0.2, 1] }}
      whileHover={{ scale: 1.02 }}
    >
      {(title || description) && (
        <div className="p-6 pb-0">
          {title && (
            <h3 className="text-xl font-bold leading-none tracking-tight text-foreground">
              {title}
            </h3>
          )}
          {description && (
            <p className="text-sm text-muted-foreground mt-2">
              {description}
            </p>
          )}
        </div>
      )}
      <div className={title || description ? 'p-6 pt-4' : 'p-6'}>
        {children}
      </div>
    </motion.div>
  );
};

interface CardHeaderProps {
  children: React.ReactNode;
  className?: string;
}

const CardHeader: React.FC<CardHeaderProps> = ({ children, className = '' }) => {
  return (
    <div className={cn('flex flex-col space-y-1.5 p-6', className)}>
      {children}
    </div>
  );
};

interface CardTitleProps {
  children: React.ReactNode;
  className?: string;
}

const CardTitle: React.FC<CardTitleProps> = ({ children, className = '' }) => {
  return (
    <h3 className={cn('font-semibold leading-none tracking-tight text-foreground', className)}>
      {children}
    </h3>
  );
};

interface CardDescriptionProps {
  children: React.ReactNode;
  className?: string;
}

const CardDescription: React.FC<CardDescriptionProps> = ({ children, className = '' }) => {
  return (
    <p className={cn('text-sm text-muted-foreground', className)}>
      {children}
    </p>
  );
};

interface CardContentProps {
  children: React.ReactNode;
  className?: string;
}

const CardContent: React.FC<CardContentProps> = ({ children, className = '' }) => {
  return (
    <div className={cn('p-6 pt-0', className)}>
      {children}
    </div>
  );
};

interface CardFooterProps {
  children: React.ReactNode;
  className?: string;
}

const CardFooter: React.FC<CardFooterProps> = ({ children, className = '' }) => {
  return (
    <div className={cn('flex items-center p-6 pt-0', className)}>
      {children}
    </div>
  );
};

export { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter };