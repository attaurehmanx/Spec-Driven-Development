import React, { ReactNode } from 'react';
import Link from 'next/link';
import { motion } from 'framer-motion';

interface ResponsiveLayoutProps {
  children: ReactNode;
  sidebar?: ReactNode;
  header?: ReactNode;
  footer?: ReactNode;
  title?: string;
  maxWidth?: 'sm' | 'md' | 'lg' | 'xl' | '2xl' | 'full';
}

const ResponsiveLayout: React.FC<ResponsiveLayoutProps> = ({
  children,
  sidebar,
  header,
  footer,
  title,
  maxWidth = '2xl',
}) => {
  const maxWidthClass = {
    sm: 'max-w-screen-sm',
    md: 'max-w-screen-md',
    lg: 'max-w-screen-lg',
    xl: 'max-w-screen-xl',
    '2xl': 'max-w-screen-2xl',
    full: 'max-w-full',
  }[maxWidth];

  return (
    <div className="flex flex-col min-h-screen">
      {header && <header>{header}</header>}

      <div className="flex flex-1">
        {sidebar && (
          <aside className="hidden md:block w-64 flex-shrink-0 glass-strong border-r-2 border-white/20 dark:border-white/10">
            {sidebar}
          </aside>
        )}

        <main className={`flex-1 ${sidebar ? 'md:ml-0' : ''}`}>
          <div className={`mx-auto w-full px-4 py-6 ${maxWidthClass}`}>
            {title && (
              <h1 className="text-3xl font-bold mb-6">{title}</h1>
            )}
            {children}
          </div>
        </main>
      </div>

      {footer && <footer>{footer}</footer>}
    </div>
  );
};

interface SidebarItemProps {
  children: ReactNode;
  href?: string;
  active?: boolean;
  onClick?: () => void;
  icon?: React.ReactNode;
}

const SidebarItem: React.FC<SidebarItemProps> = ({
  children,
  href,
  active = false,
  onClick,
  icon,
}) => {
  const baseClasses = "flex items-center gap-3 px-4 py-3 text-base font-bold rounded-xl transition-all duration-200";
  const activeClasses = active
    ? "bg-gradient-to-r from-purple-500 to-pink-500 text-white shadow-lg shadow-purple-500/50"
    : "text-gray-700 dark:text-gray-300 hover:bg-white/50 dark:hover:bg-slate-800/50 hover:scale-105";

  const classes = `${baseClasses} ${activeClasses}`;

  const content = (
    <>
      {icon && <span className="flex-shrink-0">{icon}</span>}
      <span>{children}</span>
      {active && (
        <motion.div
          layoutId="activeIndicator"
          className="ml-auto w-2 h-2 rounded-full bg-white"
          initial={false}
          transition={{ type: 'spring', stiffness: 300, damping: 30 }}
        />
      )}
    </>
  );

  if (href) {
    return (
      <Link href={href} className={classes}>
        {content}
      </Link>
    );
  }

  return (
    <button className={classes} onClick={onClick}>
      {content}
    </button>
  );
};

interface ContainerProps {
  children: ReactNode;
  className?: string;
}

const Container: React.FC<ContainerProps> = ({ children, className = '' }) => {
  return (
    <div className={`container mx-auto px-4 sm:px-6 lg:px-8 ${className}`}>
      {children}
    </div>
  );
};

export { ResponsiveLayout, SidebarItem, Container };