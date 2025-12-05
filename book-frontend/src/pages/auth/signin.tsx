import React from 'react';
import Layout from '@theme/Layout';

export default function SignIn(): JSX.Element {
  return (
    <Layout
      title="Sign In"
      description="Sign in to your Neurobotics AI account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h1>Sign In</h1>
              </div>
              <div className="card__body">
                <p>Sign in functionality will be implemented soon.</p>
                <p>For now, you can access all the learning materials without signing in.</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}