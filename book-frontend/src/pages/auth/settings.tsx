import React, { useState } from 'react';
import Layout from '@theme/Layout';
import UserProfile from '../../auth/UserProfile';
import ProtectedRoute from '../../auth/ProtectedRoute';
import { useUserPreferences } from '../../auth/userPreferences';
import { useAuthStateManager } from '../../auth/authStateManager';

export default function Settings(): JSX.Element {
  const { preferences, updatePreferences } = useUserPreferences();
  const { snapshot, exportState, importState, validateState } = useAuthStateManager();
  const [activeTab, setActiveTab] = useState<'profile' | 'preferences' | 'security' | 'data'>('profile');
  const [importData, setImportData] = useState('');
  const [validationResult, setValidationResult] = useState<any>(null);

  const handlePreferenceChange = (key: string, value: any) => {
    updatePreferences({ [key]: value });
  };

  const handleExportData = () => {
    const data = exportState();
    const blob = new Blob([data], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `auth-backup-${new Date().toISOString().split('T')[0]}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const handleImportData = () => {
    try {
      const success = importState(importData);
      if (success) {
        alert('Data imported successfully!');
        setImportData('');
      } else {
        alert('Failed to import data. Please check the format.');
      }
    } catch (error) {
      alert('Error importing data: ' + error.message);
    }
  };

  const handleValidateState = () => {
    const result = validateState();
    setValidationResult(result);
  };

  return (
    <Layout
      title="Account Settings"
      description="Manage your account settings, preferences, and security options">
      <ProtectedRoute
        requiredPermissions={['read']}
        loadingComponent={
          <div style={{ 
            display: 'flex', 
            justifyContent: 'center', 
            alignItems: 'center', 
            minHeight: '400px' 
          }}>
            <div>Loading settings...</div>
          </div>
        }
      >
        <div className="settings-container" style={{ maxWidth: '1200px', margin: '0 auto', padding: '2rem' }}>
          <div className="settings-header" style={{ marginBottom: '2rem' }}>
            <h1>Account Settings</h1>
            <p>Manage your account information, preferences, and security settings</p>
          </div>

          <div className="settings-tabs" style={{ display: 'flex', borderBottom: '1px solid #e5e7eb', marginBottom: '2rem' }}>
            {[
              { key: 'profile', label: 'Profile' },
              { key: 'preferences', label: 'Preferences' },
              { key: 'security', label: 'Security' },
              { key: 'data', label: 'Data Management' },
            ].map(tab => (
              <button
                key={tab.key}
                onClick={() => setActiveTab(tab.key as any)}
                style={{
                  padding: '0.75rem 1.5rem',
                  border: 'none',
                  background: 'transparent',
                  borderBottom: activeTab === tab.key ? '2px solid #667eea' : '2px solid transparent',
                  color: activeTab === tab.key ? '#667eea' : '#6b7280',
                  fontWeight: activeTab === tab.key ? '600' : '500',
                  cursor: 'pointer',
                  transition: 'all 0.2s ease',
                }}
              >
                {tab.label}
              </button>
            ))}
          </div>

          <div className="settings-content">
            {activeTab === 'profile' && (
              <UserProfile 
                onSuccess={() => {
                  console.log('Profile updated successfully');
                }}
              />
            )}

            {activeTab === 'preferences' && (
              <div className="preferences-section">
                <h2>User Preferences</h2>
                <div style={{ display: 'grid', gap: '1.5rem', marginTop: '1rem' }}>
                  <div className="preference-group">
                    <h3>Appearance</h3>
                    <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
                      <label>Theme:</label>
                      <select 
                        value={preferences.theme} 
                        onChange={(e) => handlePreferenceChange('theme', e.target.value)}
                        style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid #d1d5db' }}
                      >
                        <option value="light">Light</option>
                        <option value="dark">Dark</option>
                        <option value="auto">Auto</option>
                      </select>
                    </div>
                  </div>

                  <div className="preference-group">
                    <h3>Language & Region</h3>
                    <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
                      <label>Language:</label>
                      <select 
                        value={preferences.language} 
                        onChange={(e) => handlePreferenceChange('language', e.target.value)}
                        style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid #d1d5db' }}
                      >
                        <option value="en">English</option>
                        <option value="es">Spanish</option>
                        <option value="fr">French</option>
                        <option value="de">German</option>
                      </select>
                    </div>
                  </div>

                  <div className="preference-group">
                    <h3>Notifications</h3>
                    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                      <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
                        <input 
                          type="checkbox" 
                          checked={preferences.notifications} 
                          onChange={(e) => handlePreferenceChange('notifications', e.target.checked)}
                        />
                        Enable notifications
                      </label>
                      <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
                        <input 
                          type="checkbox" 
                          checked={preferences.autoSave} 
                          onChange={(e) => handlePreferenceChange('autoSave', e.target.checked)}
                        />
                        Auto-save changes
                      </label>
                      <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
                        <input 
                          type="checkbox" 
                          checked={preferences.compactMode} 
                          onChange={(e) => handlePreferenceChange('compactMode', e.target.checked)}
                        />
                        Compact mode
                      </label>
                    </div>
                  </div>
                </div>
              </div>
            )}    
        {activeTab === 'security' && (
              <div className="security-section">
                <h2>Security Settings</h2>
                <div style={{ display: 'grid', gap: '1.5rem', marginTop: '1rem' }}>
                  <div className="security-info">
                    <h3>Account Security</h3>
                    <p>Your account is secured with the following features:</p>
                    <ul style={{ marginTop: '0.5rem', paddingLeft: '1.5rem' }}>
                      <li>Password-based authentication</li>
                      <li>Session management with automatic timeout</li>
                      <li>Secure token-based API access</li>
                      <li>Activity tracking and monitoring</li>
                    </ul>
                  </div>

                  <div className="validation-section">
                    <h3>Account Validation</h3>
                    <button 
                      onClick={handleValidateState}
                      style={{
                        padding: '0.75rem 1.5rem',
                        background: '#667eea',
                        color: 'white',
                        border: 'none',
                        borderRadius: '6px',
                        cursor: 'pointer',
                        marginBottom: '1rem'
                      }}
                    >
                      Validate Account State
                    </button>
                    
                    {validationResult && (
                      <div style={{
                        padding: '1rem',
                        border: `1px solid ${validationResult.isValid ? '#10b981' : '#ef4444'}`,
                        borderRadius: '6px',
                        background: validationResult.isValid ? '#f0fdf4' : '#fef2f2'
                      }}>
                        <h4>Validation Result: {validationResult.isValid ? 'Valid' : 'Issues Found'}</h4>
                        {validationResult.issues.length > 0 && (
                          <div>
                            <h5>Issues:</h5>
                            <ul>
                              {validationResult.issues.map((issue, index) => (
                                <li key={index}>{issue}</li>
                              ))}
                            </ul>
                          </div>
                        )}
                        {validationResult.recommendations.length > 0 && (
                          <div>
                            <h5>Recommendations:</h5>
                            <ul>
                              {validationResult.recommendations.map((rec, index) => (
                                <li key={index}>{rec}</li>
                              ))}
                            </ul>
                          </div>
                        )}
                      </div>
                    )}
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'data' && (
              <div className="data-section">
                <h2>Data Management</h2>
                <div style={{ display: 'grid', gap: '1.5rem', marginTop: '1rem' }}>
                  <div className="export-section">
                    <h3>Export Data</h3>
                    <p>Download a backup of your authentication data and preferences.</p>
                    <button 
                      onClick={handleExportData}
                      style={{
                        padding: '0.75rem 1.5rem',
                        background: '#10b981',
                        color: 'white',
                        border: 'none',
                        borderRadius: '6px',
                        cursor: 'pointer',
                        marginTop: '0.5rem'
                      }}
                    >
                      Export Data
                    </button>
                  </div>

                  <div className="import-section">
                    <h3>Import Data</h3>
                    <p>Restore your data from a previous backup.</p>
                    <textarea
                      value={importData}
                      onChange={(e) => setImportData(e.target.value)}
                      placeholder="Paste your backup data here..."
                      style={{
                        width: '100%',
                        height: '100px',
                        padding: '0.75rem',
                        border: '1px solid #d1d5db',
                        borderRadius: '6px',
                        marginTop: '0.5rem',
                        fontFamily: 'monospace',
                        fontSize: '0.875rem'
                      }}
                    />
                    <button 
                      onClick={handleImportData}
                      disabled={!importData.trim()}
                      style={{
                        padding: '0.75rem 1.5rem',
                        background: importData.trim() ? '#f59e0b' : '#d1d5db',
                        color: 'white',
                        border: 'none',
                        borderRadius: '6px',
                        cursor: importData.trim() ? 'pointer' : 'not-allowed',
                        marginTop: '0.5rem'
                      }}
                    >
                      Import Data
                    </button>
                  </div>

                  {snapshot && (
                    <div className="current-state">
                      <h3>Current State</h3>
                      <div style={{
                        background: '#f9fafb',
                        padding: '1rem',
                        borderRadius: '6px',
                        border: '1px solid #e5e7eb'
                      }}>
                        <p><strong>User:</strong> {snapshot.user?.email}</p>
                        <p><strong>Device ID:</strong> {snapshot.deviceId}</p>
                        <p><strong>Last Updated:</strong> {new Date(snapshot.timestamp).toLocaleString()}</p>
                        <p><strong>Authenticated:</strong> {snapshot.isAuthenticated ? 'Yes' : 'No'}</p>
                      </div>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
        </div>
      </ProtectedRoute>
    </Layout>
  );
}