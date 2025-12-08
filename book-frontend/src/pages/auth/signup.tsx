import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../../auth/SignupForm';

export default function SignUp(): JSX.Element {
  return (
    <Layout
      title="Get Started"
      description="Create your Neurobotics AI account">
      <SignupForm 
        onSuccess={() => {
          // Redirect to home page after successful signup
          window.location.href = '/';
        }}
        redirectTo="/"
      />
    </Layout>
  );
}