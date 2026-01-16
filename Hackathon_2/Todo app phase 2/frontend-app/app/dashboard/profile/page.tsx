'use client';

import useAuth from '../../../hooks/use-auth';
import ProtectedRoute from '../../../components/auth/protected-route';
import { Card, CardHeader, CardTitle, CardContent, CardFooter } from '../../../components/ui/card';
import { useState, useRef } from 'react';
import { Button } from '../../../components/ui/button';
import Input from '../../../components/ui/input';
import Avatar from '../../../components/ui/avatar';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger
} from '../../../components/ui/dialog';
import { CameraIcon, PhoneIcon, MapPinIcon, CalendarIcon, UserIcon } from 'lucide-react';

const ProfilePage = () => {
  const { user, isLoading, updateUserProfile, uploadUserAvatar, deleteUserAccount, signOut } = useAuth();
  const [isEditing, setIsEditing] = useState(false);
  const [firstName, setFirstName] = useState(user?.first_name || '');
  const [lastName, setLastName] = useState(user?.last_name || '');
  const [phone, setPhone] = useState(user?.phone || '');
  const [address, setAddress] = useState(user?.address || '');
  const [errors, setErrors] = useState({
    firstName: '',
    lastName: '',
    phone: ''
  });
  const [isDeleteDialogOpen, setIsDeleteDialogOpen] = useState(false);
  const [isDeleting, setIsDeleting] = useState(false);
  const [avatarPreview, setAvatarPreview] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const validateForm = () => {
    let isValid = true;
    const newErrors = {
      firstName: '',
      lastName: '',
      phone: ''
    };

    if (!firstName.trim()) {
      newErrors.firstName = 'First name is required';
      isValid = false;
    }

    if (!lastName.trim()) {
      newErrors.lastName = 'Last name is required';
      isValid = false;
    }

    // Phone validation (simple pattern)
    if (phone && phone.trim() && !/^[\+]?[0-9][\d]{0,15}$/.test(phone.replace(/[\s\-\(\)]/g, ''))) {
      newErrors.phone = 'Please enter a valid phone number';
      isValid = false;
    }

    setErrors(newErrors);
    return isValid;
  };

  const handleUpdateProfile = async () => {
    if (!user) return;

    if (!validateForm()) {
      return;
    }

    try {
      await updateUserProfile(firstName, lastName, phone, address);
      setIsEditing(false);
    } catch (error) {
      console.error('Failed to update profile:', error);
    }
  };

  const handleDeleteAccount = async () => {
    setIsDeleting(true);
    try {
      await deleteUserAccount();
      setIsDeleteDialogOpen(false);
    } catch (error) {
      console.error('Failed to delete account:', error);
      setIsDeleting(false);
    }
  };

  const handleAvatarClick = () => {
    fileInputRef.current?.click();
  };

  const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file && user) {
      try {
        // Show preview immediately
        const reader = new FileReader();
        reader.onloadend = () => {
          setAvatarPreview(reader.result as string);
        };
        reader.readAsDataURL(file);

        // Upload the file to the server
        await uploadUserAvatar(user.id, file);
      } catch (error) {
        console.error('Failed to upload avatar:', error);
        // Revert to previous avatar if upload failed
        setAvatarPreview(user.avatar || null);
      }
    }
  };

  if (isLoading) {
    return (
      <ProtectedRoute>
        <div className="container mx-auto py-10">
          <div className="max-w-4xl mx-auto">
            <Card>
              <CardHeader className="text-center pb-4">
                <div className="flex justify-center mb-4">
                  <div className="w-24 h-24 rounded-full bg-gray-200 flex items-center justify-center animate-pulse"></div>
                </div>
                <CardTitle className="text-2xl font-bold">Profile Settings</CardTitle>
              </CardHeader>
              <CardContent className="space-y-6">
                <div className="flex items-center justify-center">
                  <div className="w-16 h-16 border-4 border-blue-500 border-t-transparent rounded-full animate-spin mx-auto"></div>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </ProtectedRoute>
    );
  }

  return (
    <ProtectedRoute>
      <div className="container mx-auto py-10">
        <div className="max-w-4xl mx-auto">
          <Card className="overflow-hidden">
            {/* Profile Header */}
            <div className="bg-gradient-to-r from-blue-500 to-indigo-600 p-6 text-white">
              <div className="flex items-center justify-between">
                <div className="flex items-center space-x-6">
                  <div
                    className="relative cursor-pointer group"
                    onClick={handleAvatarClick}
                  >
                    <Avatar
                      src={avatarPreview || (user?.avatar ? `${process.env.NEXT_PUBLIC_BACKEND_URL}${user.avatar}` : `https://ui-avatars.com/api/?name=${user?.first_name}+${user?.last_name}&background=fff&color=6366f1`)}
                      alt={`${user?.first_name} ${user?.last_name}`}
                      className="w-24 h-24 border-4 border-white shadow-lg"
                    />
                    {isEditing && (
                      <div className="absolute inset-0 bg-black bg-opacity-50 rounded-full flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity">
                        <CameraIcon className="w-6 h-6 text-white" />
                      </div>
                    )}
                    <input
                      type="file"
                      ref={fileInputRef}
                      className="hidden"
                      accept="image/*"
                      onChange={handleFileChange}
                    />
                  </div>
                  <div>
                    <h1 className="text-2xl font-bold">{user?.first_name} {user?.last_name}</h1>
                    <p className="text-blue-100">{user?.email}</p>
                    <div className="flex items-center mt-1">
                      <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-white bg-opacity-20 text-black">
                        Active Member
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            <CardContent className="pt-6">
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                {/* Personal Information */}
                <div className="space-y-4">
                  <h3 className="text-lg font-semibold text-gray-900 border-b pb-2">Personal Information</h3>

                  {/* Full Name */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500 flex items-center">
                      <UserIcon className="w-4 h-4 mr-1" />
                      Full Name
                    </label>
                    {isEditing ? (
                      <div className="space-y-2">
                        <div className="flex gap-2">
                          <div className="flex-1">
                            <Input
                              value={firstName}
                              onChange={(e) => setFirstName(e.target.value)}
                              className={`w-full ${errors.firstName ? 'border-red-500' : ''}`}
                              placeholder="First Name"
                            />
                            {errors.firstName && (
                              <p className="mt-1 text-sm text-red-600">{errors.firstName}</p>
                            )}
                          </div>
                          <div className="flex-1">
                            <Input
                              value={lastName}
                              onChange={(e) => setLastName(e.target.value)}
                              className={`w-full ${errors.lastName ? 'border-red-500' : ''}`}
                              placeholder="Last Name"
                            />
                            {errors.lastName && (
                              <p className="mt-1 text-sm text-red-600">{errors.lastName}</p>
                            )}
                          </div>
                        </div>
                      </div>
                    ) : (
                      <p className="text-sm text-gray-900">{user?.first_name} {user?.last_name}</p>
                    )}
                  </div>

                  {/* Email */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500">Email Address</label>
                    <p className="text-sm text-gray-900">{user?.email}</p>
                  </div>

                  {/* Phone */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500 flex items-center">
                      <PhoneIcon className="w-4 h-4 mr-1" />
                      Phone Number
                    </label>
                    {isEditing ? (
                      <div className="space-y-1">
                        <Input
                          value={phone}
                          onChange={(e) => setPhone(e.target.value)}
                          className={errors.phone ? 'border-red-500' : ''}
                          placeholder="(123) 456-7890"
                        />
                        {errors.phone && (
                          <p className="text-sm text-red-600">{errors.phone}</p>
                        )}
                      </div>
                    ) : (
                      <p className="text-sm text-gray-900">{user?.phone || 'Not provided'}</p>
                    )}
                  </div>

                  {/* Address */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500 flex items-center">
                      <MapPinIcon className="w-4 h-4 mr-1" />
                      Address
                    </label>
                    {isEditing ? (
                      <Input
                        value={address}
                        onChange={(e) => setAddress(e.target.value)}
                        placeholder="Street, City, State"
                      />
                    ) : (
                      <p className="text-sm text-gray-900">{user?.address || 'Not provided'}</p>
                    )}
                  </div>
                </div>

                {/* Account Information */}
                <div className="space-y-4">
                  <h3 className="text-lg font-semibold text-gray-900 border-b pb-2">Account Information</h3>

                  {/* Member Since */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500 flex items-center">
                      <CalendarIcon className="w-4 h-4 mr-1" />
                      Member Since
                    </label>
                    <p className="text-sm text-gray-900">{user?.created_at ? new Date(user.created_at).toLocaleDateString() : 'Today'}</p>
                  </div>

                  {/* Account Status */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500">Account Status</label>
                    <div className="flex items-center">
                      <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
                        Active
                      </span>
                    </div>
                  </div>

                  {/* Sign Out */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500">Account Actions</label>
                    <br />
                    <Button
                      variant="outline"
                      className=""
                      onClick={() => {
                        if (confirm('Are you sure you want to sign out?')) {
                          signOut();
                        }
                      }}
                    >
                      Sign Out
                    </Button>
                  </div>

                  {/* Profile Picture Upload */}
                  <div className="space-y-2">
                    <label className="text-sm font-medium text-gray-500">Profile Picture</label>
                    <div className="flex items-center space-x-4">
                      <Avatar
                        src={avatarPreview || (user?.avatar ? `${process.env.NEXT_PUBLIC_BACKEND_URL}${user.avatar}` : `https://ui-avatars.com/api/?name=${user?.first_name}+${user?.last_name}&background=fff&color=6366f1`)}
                        alt={`${user?.first_name} ${user?.last_name}`}
                        className="w-16 h-16"
                      />
                      {isEditing && (
                        <Button
                          type="button"
                          variant="outline"
                          size="sm"
                          onClick={handleAvatarClick}
                        >
                          Change Photo
                        </Button>
                      )}
                    </div>
                  </div>
                </div>
              </div>

              {/* Action Buttons */}
              <div className="pt-6 flex justify-end space-x-3">
                {isEditing ? (
                  <>
                    <Button
                      variant="outline"
                      onClick={() => {
                        setIsEditing(false);
                        setFirstName(user?.first_name || '');
                        setLastName(user?.last_name || '');
                        setPhone(user?.phone || '');
                        setAddress(user?.address || '');
                        setAvatarPreview(null);
                        setErrors({
                          firstName: '',
                          lastName: '',
                          phone: ''
                        });
                      }}
                    >
                      Cancel
                    </Button>
                    <Button onClick={handleUpdateProfile}>
                      Save Changes
                    </Button>
                  </>
                ) : (
                  <Button onClick={() => setIsEditing(true)}>
                    Edit Profile
                  </Button>
                )}
              </div>
            </CardContent>

            {/* Danger Zone */}
            <CardFooter className="bg-gray-50 border-t p-6">
              <div className="w-full">
                <h3 className="text-lg font-medium text-red-600 mb-2">Danger Zone</h3>
                <p className="text-sm text-gray-500 mb-4">
                  Once you delete your account, there is no going back. Please be certain.
                </p>
                <Dialog open={isDeleteDialogOpen} onOpenChange={setIsDeleteDialogOpen}>
                  <DialogTrigger asChild>
                    <Button variant="destructive">
                      Delete Account
                    </Button>
                  </DialogTrigger>
                  <DialogContent>
                    <DialogHeader>
                      <DialogTitle>Delete Account</DialogTitle>
                      <DialogDescription>
                        Are you sure you want to delete your account? This action cannot be undone.
                      </DialogDescription>
                    </DialogHeader>
                    <DialogFooter>
                      <Button
                        variant="outline"
                        onClick={() => setIsDeleteDialogOpen(false)}
                        disabled={isDeleting}
                      >
                        Cancel
                      </Button>
                      <Button
                        variant="destructive"
                        onClick={handleDeleteAccount}
                        disabled={isDeleting}
                      >
                        {isDeleting ? 'Deleting...' : 'Delete Account'}
                      </Button>
                    </DialogFooter>
                  </DialogContent>
                </Dialog>
              </div>
            </CardFooter>
          </Card>
        </div>
      </div>
    </ProtectedRoute>
  );
};

export default ProfilePage;