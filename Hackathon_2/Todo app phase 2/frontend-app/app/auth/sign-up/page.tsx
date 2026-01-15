'use client';

import React, { useState, useEffect } from 'react';
import Link from 'next/link';
import { useRouter } from 'next/navigation';
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from '../../../components/ui/card';
import Input from '../../../components/ui/input';
import { Button } from '../../../components/ui/button';
import ErrorMessage from '../../../components/ui/error-message';
import SuccessMessage from '../../../components/ui/success-message';
import useAuth from '../../../hooks/use-auth';

const SignUpPage: React.FC = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
  });
  const [loading, setLoading] = useState(false);

  const { signUp, error, isAuthenticated } = useAuth();
  const router = useRouter();

  // Redirect if already authenticated
  useEffect(() => {
    if (isAuthenticated) {
      router.push('/dashboard');
    }
  }, [isAuthenticated, router]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Basic validation
    if (formData.password !== formData.confirmPassword) {
      alert('Passwords do not match');
      return;
    }

    setLoading(true);

    try {
      await signUp(formData.email, formData.password, formData.firstName, formData.lastName);
      // Redirect to dashboard or sign-in page after successful signup
      router.push('/auth/sign-in');
    } catch (err) {
      console.error('Sign up failed:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-green-50 via-white to-teal-50 flex items-center justify-center p-4">
      <div className="absolute inset-0 bg-[url('data:image/svg+xml;charset=utf-8,%3Csvg%20width%3D%2260%22%20height%3D%2260%22%20viewBox%3D%220%200%2060%2060%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%3E%3Cg%20fill%3D%22none%22%20fill-rule%3D%22evenodd%22%3E%3Cg%20fill%3D%22%23d5d7e0%22%20fill-opacity%3D%220.4%22%3E%3Ccircle%20cx%3D%2230%22%20cy%3D%2230%22%20r%3D%222%22%2F%3E%3C%2Fg%3E%3C%2Fg%3E%3C%2Fsvg%3E')] opacity-20"></div>
      <div className="relative w-full max-w-md">
        <Card className="shadow-xl border-0 bg-white/80 backdrop-blur-sm">
          <CardHeader className="pb-4">
            <div className="mx-auto bg-gradient-to-r from-green-500 to-teal-600 p-2 rounded-full w-16 h-16 flex items-center justify-center mb-4">
              <svg xmlns="http://www.w3.org/2000/svg" className="h-8 w-8 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M18 9v3m0 0v3m0-3h3m-3 0h-3m-2-5a4 4 0 11-8 0 4 4 0 018 0zM3 20a6 6 0 0112 0v1H3v-1z" />
              </svg>
            </div>
            <CardTitle className="text-2xl font-bold text-center text-gray-800">Create Account</CardTitle>
            <CardDescription className="text-center text-gray-600 mt-1">
              Join us today and start managing your tasks
            </CardDescription>
          </CardHeader>
          <CardContent className="pt-2">
            <form onSubmit={handleSubmit} className="space-y-5">
              {error && <ErrorMessage message={error} />}

              <div className="grid grid-cols-2 gap-4">
                <div>
                  <Input
                    label="First Name"
                    type="text"
                    name="firstName"
                    value={formData.firstName}
                    onChange={handleChange}
                    required
                    fullWidth
                    placeholder="Enter your first name"
                  />
                </div>
                <div>
                  <Input
                    label="Last Name"
                    type="text"
                    name="lastName"
                    value={formData.lastName}
                    onChange={handleChange}
                    required
                    fullWidth
                    placeholder="Enter your last name"
                  />
                </div>
              </div>

              <div>
                <Input
                  label="Email Address"
                  type="email"
                  name="email"
                  value={formData.email}
                  onChange={handleChange}
                  required
                  fullWidth
                  placeholder="Enter your email"
                />
              </div>

              <div>
                <Input
                  label="Password"
                  type="password"
                  name="password"
                  value={formData.password}
                  onChange={handleChange}
                  required
                  fullWidth
                  placeholder="Create a strong password"
                />
              </div>

              <div>
                <Input
                  label="Confirm Password"
                  type="password"
                  name="confirmPassword"
                  value={formData.confirmPassword}
                  onChange={handleChange}
                  required
                  fullWidth
                  placeholder="Confirm your password"
                />
              </div>

              <div className="flex items-center">
                <input
                  id="terms"
                  type="checkbox"
                  className="h-4 w-4 text-green-600 focus:ring-green-500 border-gray-300 rounded"
                />
                <label htmlFor="terms" className="ml-2 block text-sm text-gray-700">
                  I agree to the <Link href="#" className="font-medium text-green-600 hover:text-green-500">Terms</Link> and <Link href="#" className="font-medium text-green-600 hover:text-green-500">Privacy Policy</Link>
                </label>
              </div>

              <Button
                type="submit"
                className="w-full py-3 text-base font-semibold"
                isLoading={loading}
                disabled={loading}
              >
                Create Account
              </Button>
            </form>
          </CardContent>
          <CardFooter className="border-t border-gray-100 pt-4 pb-6">
            <p className="text-center text-sm text-gray-600 w-full">
              Already have an account?{' '}
              <Link href="/auth/sign-in" className="font-semibold text-green-600 hover:text-green-700 transition-colors">
                Sign in to your account
              </Link>
            </p>
          </CardFooter>
        </Card>

        <div className="mt-8 text-center">
          <p className="text-xs text-gray-500">
            By creating an account, you agree to our{' '}
            <Link href="#" className="underline hover:text-gray-700">Terms</Link>
            {' '}and{' '}
            <Link href="#" className="underline hover:text-gray-700">Privacy Policy</Link>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SignUpPage;