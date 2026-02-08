'use client';

import React, { useEffect, useState } from 'react';
import { Button } from '../ui/button';
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from '../ui/card';

interface SessionExpiryModalProps {
  isOpen: boolean;
  timeRemaining: number; // in seconds
  onContinueSession: () => void;
  onLogout: () => void;
}

const SessionExpiryModal: React.FC<SessionExpiryModalProps> = ({
  isOpen,
  timeRemaining,
  onContinueSession,
  onLogout,
}) => {
  const [timeLeft, setTimeLeft] = useState(timeRemaining);

  useEffect(() => {
    setTimeLeft(timeRemaining);
  }, [timeRemaining]);

  useEffect(() => {
    let timer: NodeJS.Timeout;
    if (isOpen && timeLeft > 0) {
      timer = setTimeout(() => {
        setTimeLeft(prev => prev - 1);
      }, 1000);
    } else if (timeLeft <= 0 && isOpen) {
      // Auto-logout when timer reaches 0
      onLogout();
    }

    return () => {
      if (timer) clearTimeout(timer);
    };
  }, [timeLeft, isOpen, onLogout]);

  // Format time as MM:SS
  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50">
      <Card className="w-full max-w-md mx-4">
        <CardHeader>
          <CardTitle>Session Expiring Soon</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-gray-600">
            Your session will expire in <span className="font-bold text-red-600">{formatTime(timeLeft)}</span>.
            Please choose to continue your session or log out.
          </p>
        </CardContent>
        <CardFooter className="flex flex-col sm:flex-row gap-3">
          <Button
            variant="secondary"
            className="w-full sm:w-auto"
            onClick={onLogout}
          >
            Log Out
          </Button>
          <Button
            className="w-full sm:w-auto"
            onClick={onContinueSession}
          >
            Continue Session
          </Button>
        </CardFooter>
      </Card>
    </div>
  );
};

export default SessionExpiryModal;