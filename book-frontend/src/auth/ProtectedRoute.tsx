import React from 'react';
import { useAuth } from './AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
  requiredPermissions?: string[];
  loadingComponent?: React.ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({
  children,
  requiredPermissions = [],
  loadingComponent = <div>Loading...</div>,
}) => {
  const { isAuthenticated, loading } = useAuth();

  if (loading) {
    return <>{loadingComponent}</>;
  }

  if (!isAuthenticated) {
    return (
      <div style={{ 
        display: 'flex', 
        flexDirection: 'column',
        alignItems: 'center', 
        justifyContent: 'center', 
        minHeight: '400px',
        padding: '2rem',
        textAlign: 'center'
      }}>
        <h2>Authentication Required</h2>
        <p>Please log in to access this page.</p>
        <div style={{ marginTop: '1rem' }}>
          <a 
            href="/auth/signin" 
            style={{
              padding: '0.75rem 1.5rem',
              background: '#667eea',
              color: 'white',
              textDecoration: 'none',
              borderRadius: '6px',
              marginRight: '1rem'
            }}
          >
            Sign In
          </a>
          <a 
            href="/auth/signup"
            style={{
              padding: '0.75rem 1.5rem',
              background: 'transparent',
              color: '#667eea',
              textDecoration: 'none',
              border: '1px solid #667eea',
              borderRadius: '6px'
            }}
          >
            Sign Up
          </a>
        </div>
      </div>
    );
  }

  // In a real implementation, you would check requiredPermissions here
  return <>{children}</>;
};

export default ProtectedRoute;