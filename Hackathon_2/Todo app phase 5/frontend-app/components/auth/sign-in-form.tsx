'use client';

import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
import Input from '@/components/ui/input';
import { Button } from '@/components/ui/button';
import ErrorMessage from '@/components/ui/error-message';
import useAuth from '@/hooks/use-auth';

interface SignInFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

const SignInForm: React.FC<SignInFormProps> = ({ onSuccess, onError }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });
  const [loading, setLoading] = useState(false);

  const { signIn, error } = useAuth();
  const router = useRouter();

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      await signIn(formData.email, formData.password);
      onSuccess?.();
    } catch (err) {
      onError?.((err as Error).message || 'Sign in failed');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      {error && <ErrorMessage message={error} />}

      <Input
        label="Email"
        type="email"
        name="email"
        value={formData.email}
        onChange={handleChange}
        required
        fullWidth
      />

      <Input
        label="Password"
        type="password"
        name="password"
        value={formData.password}
        onChange={handleChange}
        required
        fullWidth
      />

      <Button
        type="submit"
        className="w-full"
        isLoading={loading}
        disabled={loading}
      >
        Sign In
      </Button>
    </form>
  );
};

export default SignInForm;