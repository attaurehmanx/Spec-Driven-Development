'use client';

import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
import Input from '@/components/ui/input';
import { Button } from '@/components/ui/button';
import ErrorMessage from '@/components/ui/error-message';
import SuccessMessage from '@/components/ui/success-message';
import useAuth from '@/hooks/use-auth';

interface SignUpFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

const SignUpForm: React.FC<SignUpFormProps> = ({ onSuccess, onError }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
  });
  const [loading, setLoading] = useState(false);

  const { signUp, error } = useAuth();
  const router = useRouter();

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Basic validation
    if (formData.password !== formData.confirmPassword) {
      onError?.('Passwords do not match');
      return;
    }

    setLoading(true);

    try {
      await signUp(formData.email, formData.password, formData.firstName, formData.lastName);
      onSuccess?.();
    } catch (err) {
      onError?.((err as Error).message || 'Sign up failed');
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

      <Input
        label="First Name"
        type="text"
        name="firstName"
        value={formData.firstName}
        onChange={handleChange}
        required
        fullWidth
      />

      <Input
        label="Last Name"
        type="text"
        name="lastName"
        value={formData.lastName}
        onChange={handleChange}
        required
        fullWidth
      />

      <Input
        label="Confirm Password"
        type="password"
        name="confirmPassword"
        value={formData.confirmPassword}
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
        Sign Up
      </Button>
    </form>
  );
};

export default SignUpForm;