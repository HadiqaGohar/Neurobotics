/**
 * Personalization component for user preferences and content customization.
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../auth/AuthContext';

// Types
interface UserPreferences {
  persona: string;
  experience_level: string;
  domains: string[];
  complexity: number;
  language: string;
  formats: string[];
}

interface UserProfile {
  user_id: string;
  persona: string;
  experience_level: string;
  domains_of_interest: string[];
  preferred_formats: string[];
  language_preference: string;
  complexity_preference: number;
  behavior_patterns: any;
  last_updated: string;
}

const PersonalizationComponent: React.FC = () => {
  const { user, isAuthenticated } = useAuth();
  const [preferences, setPreferences] = useState<UserPreferences>({
    persona: 'beginner',
    experience_level: 'beginner',
    domains: [],
    complexity: 1,
    language: 'english',
    formats: ['tutorial', 'guide'],
  });
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [showAdvanced, setShowAdvanced] = useState(false);

  // Available options
  const personas = [
    { value: 'beginner', label: 'Beginner', description: 'New to the topic, need basic explanations' },
    { value: 'intermediate', label: 'Intermediate', description: 'Some experience, looking for practical applications' },
    { value: 'advanced', label: 'Advanced', description: 'Experienced, interested in optimization and architecture' },
    { value: 'researcher', label: 'Researcher', description: 'Academic focus, interested in methodology and analysis' },
    { value: 'developer', label: 'Developer', description: 'Code-focused, need implementation details' },
    { value: 'student', label: 'Student', description: 'Learning-focused, need educational content' },
  ];

  const experienceLevels = [
    { value: 'beginner', label: 'Beginner' },
    { value: 'intermediate', label: 'Intermediate' },
    { value: 'advanced', label: 'Advanced' },
  ];

  const availableDomains = [
    { value: 'machine_learning', label: 'Machine Learning' },
    { value: 'web_development', label: 'Web Development' },
    { value: 'data_science', label: 'Data Science' },
    { value: 'cybersecurity', label: 'Cybersecurity' },
    { value: 'cloud_computing', label: 'Cloud Computing' },
    { value: 'mobile_development', label: 'Mobile Development' },
    { value: 'devops', label: 'DevOps' },
    { value: 'artificial_intelligence', label: 'Artificial Intelligence' },
  ];

  const contentFormats = [
    { value: 'tutorial', label: 'Tutorials' },
    { value: 'guide', label: 'Step-by-step Guides' },
    { value: 'example', label: 'Code Examples' },
    { value: 'case_study', label: 'Case Studies' },
    { value: 'research', label: 'Research Papers' },
    { value: 'documentation', label: 'Technical Documentation' },
    { value: 'best_practices', label: 'Best Practices' },
  ];

  const languages = [
    { value: 'english', label: 'English' },
    { value: 'urdu', label: 'Urdu' },
    { value: 'spanish', label: 'Spanish' },
    { value: 'french', label: 'French' },
    { value: 'german', label: 'German' },
  ];

  // Load user profile on component mount
  useEffect(() => {
    if (isAuthenticated && user) {
      loadUserProfile();
    }
  }, [isAuthenticated, user]);

  const loadUserProfile = async () => {
    setIsLoading(true);
    try {
      const response = await fetch('/api/personalization/profile', {
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('access_token')}`,
        },
      });

      if (response.ok) {
        const profile = await response.json();
        setUserProfile(profile);
        
        // Update preferences from profile
        setPreferences({
          persona: profile.persona,
          experience_level: profile.experience_level,
          domains: profile.domains_of_interest,
          complexity: profile.complexity_preference,
          language: profile.language_preference,
          formats: profile.preferred_formats,
        });
      }
    } catch (error) {
      console.error('Error loading user profile:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const savePreferences = async () => {
    if (!isAuthenticated) return;

    setIsSaving(true);
    try {
      const response = await fetch('/api/personalization/preferences', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('access_token')}`,
        },
        body: JSON.stringify(preferences),
      });

      if (response.ok) {
        // Reload profile to get updated data
        await loadUserProfile();
        alert('Preferences saved successfully!');
      } else {
        throw new Error('Failed to save preferences');
      }
    } catch (error) {
      console.error('Error saving preferences:', error);
      alert('Failed to save preferences. Please try again.');
    } finally {
      setIsSaving(false);
    }
  };

  const handlePreferenceChange = (key: keyof UserPreferences, value: any) => {
    setPreferences(prev => ({
      ...prev,
      [key]: value,
    }));
  };

  const handleDomainToggle = (domain: string) => {
    setPreferences(prev => ({
      ...prev,
      domains: prev.domains.includes(domain)
        ? prev.domains.filter(d => d !== domain)
        : [...prev.domains, domain].slice(0, 5), // Limit to 5 domains
    }));
  };

  const handleFormatToggle = (format: string) => {
    setPreferences(prev => ({
      ...prev,
      formats: prev.formats.includes(format)
        ? prev.formats.filter(f => f !== format)
        : [...prev.formats, format],
    }));
  };

  if (!isAuthenticated) {
    return (
      <div className="max-w-2xl mx-auto p-6 bg-white rounded-lg shadow-md">
        <div className="text-center">
          <h2 className="text-2xl font-bold text-gray-900 mb-4">Personalization</h2>
          <p className="text-gray-600">Please log in to customize your experience.</p>
        </div>
      </div>
    );
  }

  if (isLoading) {
    return (
      <div className="max-w-2xl mx-auto p-6 bg-white rounded-lg shadow-md">
        <div className="animate-pulse">
          <div className="h-8 bg-gray-200 rounded mb-4"></div>
          <div className="space-y-3">
            <div className="h-4 bg-gray-200 rounded"></div>
            <div className="h-4 bg-gray-200 rounded"></div>
            <div className="h-4 bg-gray-200 rounded"></div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="max-w-4xl mx-auto p-6 space-y-6">
      <div className="bg-white rounded-lg shadow-md p-6">
        <div className="flex items-center justify-between mb-6">
          <h2 className="text-2xl font-bold text-gray-900">Personalization Settings</h2>
          <button
            onClick={savePreferences}
            disabled={isSaving}
            className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:opacity-50"
          >
            {isSaving ? 'Saving...' : 'Save Preferences'}
          </button>
        </div>

        {/* User Profile Summary */}
        {userProfile && (
          <div className="mb-6 p-4 bg-gray-50 rounded-lg">
            <h3 className="text-lg font-semibold mb-2">Your Profile</h3>
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
              <div>
                <span className="font-medium">Persona:</span>
                <span className="ml-2 capitalize">{userProfile.persona}</span>
              </div>
              <div>
                <span className="font-medium">Level:</span>
                <span className="ml-2 capitalize">{userProfile.experience_level}</span>
              </div>
              <div>
                <span className="font-medium">Domains:</span>
                <span className="ml-2">{userProfile.domains_of_interest.length}</span>
              </div>
              <div>
                <span className="font-medium">Updated:</span>
                <span className="ml-2">{new Date(userProfile.last_updated).toLocaleDateString()}</span>
              </div>
            </div>
          </div>
        )}

        {/* Persona Selection */}
        <div className="mb-6">
          <label className="block text-sm font-medium text-gray-700 mb-3">
            Learning Persona
          </label>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            {personas.map((persona) => (
              <div
                key={persona.value}
                className={`p-3 border rounded-lg cursor-pointer transition-colors ${
                  preferences.persona === persona.value
                    ? 'border-blue-500 bg-blue-50'
                    : 'border-gray-200 hover:border-gray-300'
                }`}
                onClick={() => handlePreferenceChange('persona', persona.value)}
              >
                <div className="flex items-center">
                  <input
                    type="radio"
                    name="persona"
                    value={persona.value}
                    checked={preferences.persona === persona.value}
                    onChange={() => handlePreferenceChange('persona', persona.value)}
                    className="mr-3"
                  />
                  <div>
                    <div className="font-medium">{persona.label}</div>
                    <div className="text-sm text-gray-600">{persona.description}</div>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Experience Level */}
        <div className="mb-6">
          <label className="block text-sm font-medium text-gray-700 mb-3">
            Experience Level
          </label>
          <div className="flex space-x-4">
            {experienceLevels.map((level) => (
              <label key={level.value} className="flex items-center">
                <input
                  type="radio"
                  name="experience_level"
                  value={level.value}
                  checked={preferences.experience_level === level.value}
                  onChange={(e) => handlePreferenceChange('experience_level', e.target.value)}
                  className="mr-2"
                />
                {level.label}
              </label>
            ))}
          </div>
        </div>

        {/* Complexity Preference */}
        <div className="mb-6">
          <label className="block text-sm font-medium text-gray-700 mb-3">
            Content Complexity: {preferences.complexity}
          </label>
          <input
            type="range"
            min="1"
            max="3"
            value={preferences.complexity}
            onChange={(e) => handlePreferenceChange('complexity', parseInt(e.target.value))}
            className="w-full"
          />
          <div className="flex justify-between text-sm text-gray-600 mt-1">
            <span>Simple</span>
            <span>Moderate</span>
            <span>Advanced</span>
          </div>
        </div>

        {/* Domains of Interest */}
        <div className="mb-6">
          <label className="block text-sm font-medium text-gray-700 mb-3">
            Domains of Interest (Select up to 5)
          </label>
          <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
            {availableDomains.map((domain) => (
              <label
                key={domain.value}
                className={`flex items-center p-3 border rounded-lg cursor-pointer transition-colors ${
                  preferences.domains.includes(domain.value)
                    ? 'border-blue-500 bg-blue-50'
                    : 'border-gray-200 hover:border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={preferences.domains.includes(domain.value)}
                  onChange={() => handleDomainToggle(domain.value)}
                  disabled={!preferences.domains.includes(domain.value) && preferences.domains.length >= 5}
                  className="mr-2"
                />
                {domain.label}
              </label>
            ))}
          </div>
        </div>

        {/* Language Preference */}
        <div className="mb-6">
          <label className="block text-sm font-medium text-gray-700 mb-3">
            Language Preference
          </label>
          <select
            value={preferences.language}
            onChange={(e) => handlePreferenceChange('language', e.target.value)}
            className="w-full p-2 border border-gray-300 rounded-md focus:ring-blue-500 focus:border-blue-500"
          >
            {languages.map((lang) => (
              <option key={lang.value} value={lang.value}>
                {lang.label}
              </option>
            ))}
          </select>
        </div>

        {/* Advanced Settings */}
        <div className="mb-6">
          <button
            onClick={() => setShowAdvanced(!showAdvanced)}
            className="flex items-center text-blue-600 hover:text-blue-700"
          >
            <svg
              className={`w-4 h-4 mr-2 transition-transform ${showAdvanced ? 'rotate-90' : ''}`}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
            </svg>
            Advanced Settings
          </button>

          {showAdvanced && (
            <div className="mt-4 space-y-4">
              {/* Content Formats */}
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-3">
                  Preferred Content Formats
                </label>
                <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
                  {contentFormats.map((format) => (
                    <label
                      key={format.value}
                      className={`flex items-center p-2 border rounded cursor-pointer transition-colors ${
                        preferences.formats.includes(format.value)
                          ? 'border-blue-500 bg-blue-50'
                          : 'border-gray-200 hover:border-gray-300'
                      }`}
                    >
                      <input
                        type="checkbox"
                        checked={preferences.formats.includes(format.value)}
                        onChange={() => handleFormatToggle(format.value)}
                        className="mr-2"
                      />
                      {format.label}
                    </label>
                  ))}
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Behavior Insights */}
        {userProfile?.behavior_patterns && (
          <div className="mb-6 p-4 bg-gray-50 rounded-lg">
            <h3 className="text-lg font-semibold mb-3">Your Learning Insights</h3>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4 text-sm">
              <div>
                <span className="font-medium">Average Query Complexity:</span>
                <div className="mt-1">
                  <div className="w-full bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-600 h-2 rounded-full"
                      style={{
                        width: `${(userProfile.behavior_patterns.average_complexity / 3) * 100}%`,
                      }}
                    ></div>
                  </div>
                </div>
              </div>
              <div>
                <span className="font-medium">Interaction Frequency:</span>
                <div className="text-lg font-bold text-blue-600">
                  {userProfile.behavior_patterns.interaction_frequency || 0}
                </div>
              </div>
              <div>
                <span className="font-medium">Common Topics:</span>
                <div className="flex flex-wrap gap-1 mt-1">
                  {(userProfile.behavior_patterns.common_topics || []).slice(0, 3).map((topic: string) => (
                    <span
                      key={topic}
                      className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded"
                    >
                      {topic}
                    </span>
                  ))}
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Reset to Defaults */}
        <div className="pt-4 border-t border-gray-200">
          <button
            onClick={() => {
              setPreferences({
                persona: 'beginner',
                experience_level: 'beginner',
                domains: [],
                complexity: 1,
                language: 'english',
                formats: ['tutorial', 'guide'],
              });
            }}
            className="text-red-600 hover:text-red-700 text-sm"
          >
            Reset to Defaults
          </button>
        </div>
      </div>
    </div>
  );
};

export default PersonalizationComponent;