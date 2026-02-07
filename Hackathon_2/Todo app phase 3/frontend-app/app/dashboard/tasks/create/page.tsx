'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import ProtectedRoute from '../../../../components/auth/protected-route';

const TaskCreationPage: React.FC = () => {
  const router = useRouter();

  useEffect(() => {
    // Redirect to tasks page with create modal open
    router.push('/dashboard/tasks?create=true');
  }, [router]);

  return (
    <ProtectedRoute>
      <div className="flex items-center justify-center min-h-[50vh]">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-purple-600 mx-auto mb-4"></div>
          <p className="text-gray-600 dark:text-gray-400">Redirecting...</p>
        </div>
      </div>
    </ProtectedRoute>
  );
};

export default TaskCreationPage;