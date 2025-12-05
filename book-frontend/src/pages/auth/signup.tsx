import React from 'react';
import Layout from '@theme/Layout';

export default function SignUp(): JSX.Element {
  return (
    <Layout
      title="Get Started"
      description="Create your Neurobotics AI account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h1>Get Started</h1>
              </div>
              <div className="card__body">
                <p>Account creation functionality will be implemented soon.</p>
                <p>For now, you can access all the learning materials without creating an account.</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}