'use client';

import React from 'react';
import { Button } from '../ui/button';
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from '../ui/card';

interface LogoutConfirmationDialogProps {
  isOpen: boolean;
  onConfirm: () => void;
  onCancel: () => void;
}

const LogoutConfirmationDialog: React.FC<LogoutConfirmationDialogProps> = ({
  isOpen,
  onConfirm,
  onCancel,
}) => {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50">
      <Card className="w-full max-w-md mx-4">
        <CardHeader>
          <CardTitle>Confirm Logout</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-gray-600">
            Are you sure you want to log out? You will need to sign in again to access your tasks.
          </p>
        </CardContent>
        <CardFooter className="flex flex-col sm:flex-row gap-3">
          <Button
            variant="secondary"
            className="w-full sm:w-auto"
            onClick={onCancel}
          >
            Cancel
          </Button>
          <Button
            variant="destructive"
            className="w-full sm:w-auto"
            onClick={onConfirm}
          >
            Log Out
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
};

export default LogoutConfirmationDialog;