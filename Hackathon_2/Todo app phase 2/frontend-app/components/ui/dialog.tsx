'use client';

import React from 'react';

interface DialogProps {
  children: React.ReactNode;
  open?: boolean;
  onOpenChange?: (open: boolean) => void;
}

interface DialogTriggerProps {
  children: React.ReactNode;
  asChild?: boolean;
}

interface DialogContentProps {
  children: React.ReactNode;
  className?: string;
}

interface DialogHeaderProps {
  children: React.ReactNode;
}

interface DialogTitleProps {
  children: React.ReactNode;
}

interface DialogDescriptionProps {
  children: React.ReactNode;
}

interface DialogFooterProps {
  children: React.ReactNode;
}

interface DialogCloseProps {
  children: React.ReactNode;
  onClick?: () => void;
}

const Dialog = ({ children, open, onOpenChange }: DialogProps) => {
  const childArray = React.Children.toArray(children);
  const trigger = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).name === 'DialogTrigger'
  );
  const content = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).name === 'DialogContent'
  );

  const [internalOpen, setInternalOpen] = React.useState(false);
  const isOpen = open ?? internalOpen;

  const toggleOpen = () => {
    if (onOpenChange) {
      onOpenChange(!isOpen);
    } else {
      setInternalOpen(!internalOpen);
    }
  };

  return (
    <div>
      {trigger && React.cloneElement(trigger as React.ReactElement, { onClick: toggleOpen })}
      {isOpen && content && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50">
          {React.cloneElement(content as React.ReactElement, { onClose: toggleOpen })}
        </div>
      )}
    </div>
  );
};

const DialogTrigger = ({ children, onClick }: DialogTriggerProps & { onClick?: () => void }) => {
  const child = React.Children.only(children);
  return React.cloneElement(child as React.ReactElement, { onClick });
};

interface DialogContentComponent extends React.FC<DialogContentProps> {
  displayName?: string;
}

const DialogContent: DialogContentComponent = ({ children, onClose, className }: DialogContentProps & { onClose?: () => void }) => {
  const childArray = React.Children.toArray(children);
  const header = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).displayName === 'DialogHeader'
  );
  const body = childArray.filter(child =>
    React.isValidElement(child) && (child.type as any).displayName !== 'DialogHeader' &&
    (child.type as any).displayName !== 'DialogFooter'
  );
  const footer = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).displayName === 'DialogFooter'
  );

  return (
    <div className={`bg-white rounded-lg shadow-xl w-full max-w-md mx-4 ${className}`}>
      {header}
      {body.length > 0 && (
        <div className="p-6">
          {body}
        </div>
      )}
      {footer}
      <button
        className="absolute top-4 right-4 text-gray-500 hover:text-gray-700"
        onClick={onClose}
      >
        ✕
      </button>
    </div>
  );
};

DialogContent.displayName = 'DialogContent';

const DialogHeader = ({ children }: DialogHeaderProps) => {
  const childArray = React.Children.toArray(children);
  const title = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).displayName === 'DialogTitle'
  );
  const description = childArray.find(child =>
    React.isValidElement(child) && (child.type as any).displayName === 'DialogDescription'
  );
  const others = childArray.filter(child =>
    React.isValidElement(child) &&
    (child.type as any).displayName !== 'DialogTitle' &&
    (child.type as any).displayName !== 'DialogDescription'
  );

  return (
    <div className="border-b border-gray-200 p-6 pt-6 pb-4">
      {title && <div className="mb-1">{title}</div>}
      {description && <div className="text-sm text-gray-500">{description}</div>}
      {others.length > 0 && <div>{others}</div>}
    </div>
  );
};

DialogHeader.displayName = 'DialogHeader';

const DialogTitle = ({ children }: DialogTitleProps) => {
  return <h3 className="text-lg font-semibold leading-none tracking-tight">{children}</h3>;
};

DialogTitle.displayName = 'DialogTitle';

const DialogDescription = ({ children }: DialogDescriptionProps) => {
  return <p className="text-sm text-muted-foreground">{children}</p>;
};

DialogDescription.displayName = 'DialogDescription';

const DialogFooter = ({ children }: DialogFooterProps) => {
  return <div className="flex flex-col-reverse sm:flex-row sm:justify-end sm:space-x-2 p-6 pt-4">{children}</div>;
};

DialogFooter.displayName = 'DialogFooter';

const DialogClose = ({ children, onClick }: DialogCloseProps) => {
  return React.cloneElement(children as React.ReactElement, { onClick });
};

DialogClose.displayName = 'DialogClose';

export {
  Dialog,
  DialogTrigger,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
  DialogFooter,
  DialogClose,
};