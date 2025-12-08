import { useState } from 'react';
import { useAuth } from './AuthContext';

interface UpdateUserProfileData {
  full_name?: string;
  email?: string;
}

export const useUserProfile = () => {
  const { user } = useAuth();
  const [isUpdating, setIsUpdating] = useState(false);
  const [updateError, setUpdateError] = useState<string | null>(null);

  const updateUserProfile = async (data: UpdateUserProfileData) => {
    setIsUpdating(true);
    setUpdateError(null);
    
    try {
      // Mock update for demo purposes
      await new Promise(resolve => setTimeout(resolve, 1000));
      
      // In a real implementation, you would make an API call here
      console.log('Updating user profile with:', data);
      
      // Simulate success
      return { success: true };
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Update failed';
      setUpdateError(errorMessage);
      throw error;
    } finally {
      setIsUpdating(false);
    }
  };

  return {
    updateUserProfile,
    isUpdating,
    updateError,
  };
};