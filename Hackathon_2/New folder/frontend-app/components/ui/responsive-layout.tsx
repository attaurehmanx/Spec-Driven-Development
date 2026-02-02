import React, { ReactNode } from 'react';
import Link from 'next/link';

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
          <aside className="hidden md:block w-64 flex-shrink-0">
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
}

const SidebarItem: React.FC<SidebarItemProps> = ({
  children,
  href,
  active = false,
  onClick,
}) => {
  const baseClasses = "flex items-center px-4 py-3 text-base font-medium rounded-md";
  const activeClasses = active
    ? "bg-blue-100 text-blue-800"
    : "text-gray-600 hover:bg-gray-100";

  const classes = `${baseClasses} ${activeClasses}`;

  if (href) {
    return (
      <Link href={href} className={classes}>
        {children}
      </Link>
    );
  }

  return (
    <button className={classes} onClick={onClick}>
      {children}
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