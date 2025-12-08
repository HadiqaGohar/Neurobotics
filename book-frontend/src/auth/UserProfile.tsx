import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { useUserProfile } from './authHooks';

interface UserProfileProps {
  onSuccess?: () => void;
}

const UserProfile: React.FC<UserProfileProps> = ({ onSuccess }) => {
  const { user } = useAuth();
  const { updateUserProfile, isUpdating, updateError } = useUserProfile();
  const [formData, setFormData] = useState({
    full_name: user?.full_name || '',
    email: user?.email || '',
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    try {
      await updateUserProfile(formData);
      onSuccess?.();
    } catch (error) {
      console.error('Profile update failed:', error);
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value,
    }));
  };

  if (!user) {
    return <div>Please log in to view your profile.</div>;
  }

  return (
    <div style={{ maxWidth: '600px', margin: '0 auto' }}>
      <h2>User Profile</h2>
      
      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
        <div>
          <label htmlFor="full_name" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Full Name
          </label>
          <input
            type="text"
            id="full_name"
            name="full_name"
            value={formData.full_name}
            onChange={handleChange}
            style={{
              width: '100%',
              padding: '0.75rem',
              border: '1px solid #d1d5db',
              borderRadius: '6px',
              fontSize: '1rem',
            }}
          />
        </div>

        <div>
          <label htmlFor="email" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
            Email
          </label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            style={{
              width: '100%',
              padding: '0.75rem',
              border: '1px solid #d1d5db',
              borderRadius: '6px',
              fontSize: '1rem',
            }}
          />
        </div>

        <div style={{ display: 'flex', gap: '1rem', marginTop: '1rem' }}>
          <button
            type="submit"
            disabled={isUpdating}
            style={{
              padding: '0.75rem 1.5rem',
              background: isUpdating ? '#9ca3af' : '#667eea',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              fontSize: '1rem',
              cursor: isUpdating ? 'not-allowed' : 'pointer',
              transition: 'background-color 0.2s',
            }}
          >
            {isUpdating ? 'Updating...' : 'Update Profile'}
          </button>
        </div>

        {updateError && (
          <div style={{
            padding: '0.75rem',
            background: '#fef2f2',
            border: '1px solid #fecaca',
            borderRadius: '6px',
            color: '#dc2626',
          }}>
            {updateError}
          </div>
        )}
      </form>

      <div style={{ marginTop: '2rem', padding: '1rem', background: '#f9fafb', borderRadius: '6px' }}>
        <h3>Account Information</h3>
        <div style={{ display: 'grid', gap: '0.5rem', fontSize: '0.875rem' }}>
          <div><strong>User ID:</strong> {user.id}</div>
          <div><strong>Status:</strong> {user.is_active ? 'Active' : 'Inactive'}</div>
          <div><strong>Member Since:</strong> {new Date(user.created_at).toLocaleDateString()}</div>
          {user.last_login && (
            <div><strong>Last Login:</strong> {new Date(user.last_login).toLocaleDateString()}</div>
          )}
        </div>
      </div>
    </div>
  );
};

export default UserProfile;