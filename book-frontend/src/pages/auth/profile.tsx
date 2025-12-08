import React from 'react';
import Layout from '@theme/Layout';
import UserProfile from '../../auth/UserProfile';
import ProtectedRoute from '../../auth/ProtectedRoute';
import ProfileTest from '../../components/ProfileTest';

export default function Profile(): JSX.Element {
  return (
    <Layout
      title="User Profile"
      description="Manage your account information and preferences">
      <ProtectedRoute
        requiredPermissions={['read']}
        loadingComponent={
          <div style={{ 
            display: 'flex', 
            justifyContent: 'center', 
            alignItems: 'center', 
            minHeight: '400px' 
          }}>
            <div>Loading profile...</div>
          </div>
        }
      >
        <div style={{ maxWidth: '800px', margin: '0 auto', padding: '2rem' }}>
          {/* Debug component - remove in production */}
          <ProfileTest />
          
          <UserProfile 
            onSuccess={() => {
              console.log('Profile updated successfully');
            }}
          />
        </div>
      </ProtectedRoute>
    </Layout>
  );
}