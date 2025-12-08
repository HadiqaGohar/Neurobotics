import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../../auth/LoginForm';

export default function SignIn(): JSX.Element {
  return (
    <Layout
      title="Sign In"
      description="Sign in to your Neurobotics AI account">
      <LoginForm 
        onSuccess={() => {
          // Redirect to home page after successful login
          window.location.href = '/';
        }}
        redirectTo="/"
      />
    </Layout>
  );
}