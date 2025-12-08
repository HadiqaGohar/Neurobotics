/**
 * Simple test component for profile functionality
 */

import React from 'react';
import { useAuth } from '../../auth/AuthContext';
import { useUserProfile } from '../../auth/authHooks';

const ProfileTest: React.FC = () => {
  const { user, isAuthenticated } = useAuth();
  const { updateUserProfile, isUpdating, updateError } = useUserProfile();

  const handleTestUpdate = async () => {
    if (!user) return;
    
    try {
      await updateUserProfile({
        full_name: user.full_name + ' (Updated)'
      });
      alert('Profile updated successfully!');
    } catch (error) {
      console.error('Update failed:', error);
    }
  };

  if (!isAuthenticated) {
    return <div>Please log in to test profile functionality.</div>;
  }

  return (
    <div style={{ padding: '1rem', border: '1px solid #ccc', borderRadius: '8px', margin: '1rem' }}>
      <h3>Profile Test Component</h3>
      <div style={{ marginBottom: '1rem' }}>
        <p><strong>User ID:</strong> {user?.id}</p>
        <p><strong>Email:</strong> {user?.email}</p>
        <p><strong>Full Name:</strong> {user?.full_name || 'Not set'}</p>
        <p><strong>Active:</strong> {user?.is_active ? 'Yes' : 'No'}</p>
        <p><strong>Created:</strong> {user?.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}</p>
      </div>
      
      <button 
        onClick={handleTestUpdate}
        disabled={isUpdating}
        style={{
          padding: '0.5rem 1rem',
          background: isUpdating ? '#ccc' : '#007bff',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isUpdating ? 'not-allowed' : 'pointer'
        }}
      >
        {isUpdating ? 'Updating...' : 'Test Profile Update'}
      </button>
      
      {updateError && (
        <div style={{ color: 'red', marginTop: '0.5rem' }}>
          Error: {updateError}
        </div>
      )}
    </div>
  );
};

export default ProfileTest;