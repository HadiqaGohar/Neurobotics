/**
 * Software Background Collection Component
 */

import React, { useState } from 'react';
import {
  SoftwareCategory,
  ExperienceLevel,
  SoftwareBackground,
  SOFTWARE_CATEGORY_LABELS,
  EXPERIENCE_LEVEL_LABELS,
  PROGRAMMING_LANGUAGES,
  FRAMEWORKS,
} from '../types/personalization';

interface SoftwareBackgroundFormProps {
  value: SoftwareBackground;
  onChange: (background: SoftwareBackground) => void;
  className?: string;
}

const SoftwareBackgroundForm: React.FC<SoftwareBackgroundFormProps> = ({
  value,
  onChange,
  className = '',
}) => {
  const [showAllLanguages, setShowAllLanguages] = useState(false);
  const [showAllFrameworks, setShowAllFrameworks] = useState(false);

  const handleCategoryChange = (category: SoftwareCategory, checked: boolean) => {
    const newCategories = checked
      ? [...value.categories, category]
      : value.categories.filter(c => c !== category);
    
    onChange({
      ...value,
      categories: newCategories,
    });
  };

  const handleLanguageChange = (language: string, checked: boolean) => {
    const newLanguages = checked
      ? [...value.preferredLanguages, language]
      : value.preferredLanguages.filter(l => l !== language);
    
    onChange({
      ...value,
      preferredLanguages: newLanguages,
    });
  };

  const handleFrameworkChange = (framework: string, checked: boolean) => {
    const newFrameworks = checked
      ? [...value.frameworks, framework]
      : value.frameworks.filter(f => f !== framework);
    
    onChange({
      ...value,
      frameworks: newFrameworks,
    });
  };

  const displayedLanguages = showAllLanguages 
    ? PROGRAMMING_LANGUAGES 
    : PROGRAMMING_LANGUAGES.slice(0, 8);

  const displayedFrameworks = showAllFrameworks 
    ? FRAMEWORKS 
    : FRAMEWORKS.slice(0, 8);

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="text-center">
        <h3 className="text-lg font-medium text-gray-900 mb-2">
          Software Development Background
        </h3>
        <p className="text-sm text-gray-600">
          Help us personalize your learning experience by sharing your software development background
        </p>
      </div>

      {/* Software Categories */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which areas of software development interest you? (Select all that apply)
        </label>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
          {Object.entries(SOFTWARE_CATEGORY_LABELS).map(([key, label]) => {
            const category = key as SoftwareCategory;
            const isChecked = value.categories.includes(category);
            
            return (
              <label
                key={category}
                className={`relative flex items-center p-3 border rounded-lg cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-blue-500 bg-blue-50 ring-1 ring-blue-500' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={isChecked}
                  onChange={(e) => handleCategoryChange(category, e.target.checked)}
                  className="sr-only"
                />
                <div className={`flex-shrink-0 w-4 h-4 rounded border-2 mr-3 flex items-center justify-center ${
                  isChecked 
                    ? 'bg-blue-500 border-blue-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-3 h-3 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <span className="text-sm font-medium text-gray-900">{label}</span>
              </label>
            );
          })}
        </div>
      </div>

      {/* Experience Level */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          What's your overall software development experience level?
        </label>
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
          {Object.entries(EXPERIENCE_LEVEL_LABELS).map(([key, label]) => {
            const level = key as ExperienceLevel;
            const isSelected = value.experienceLevel === level;
            
            return (
              <label
                key={level}
                className={`relative flex items-center justify-center p-3 border rounded-lg cursor-pointer hover:bg-gray-50 transition-colors ${
                  isSelected 
                    ? 'border-blue-500 bg-blue-50 ring-1 ring-blue-500' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="radio"
                  name="experienceLevel"
                  value={level}
                  checked={isSelected}
                  onChange={(e) => onChange({ ...value, experienceLevel: e.target.value as ExperienceLevel })}
                  className="sr-only"
                />
                <span className={`text-sm font-medium ${
                  isSelected ? 'text-blue-900' : 'text-gray-900'
                }`}>
                  {label}
                </span>
              </label>
            );
          })}
        </div>
      </div>

      {/* Programming Languages */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which programming languages do you prefer? (Optional)
        </label>
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2">
          {displayedLanguages.map((language) => {
            const isChecked = value.preferredLanguages.includes(language);
            
            return (
              <label
                key={language}
                className={`relative flex items-center p-2 border rounded cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-blue-500 bg-blue-50' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={isChecked}
                  onChange={(e) => handleLanguageChange(language, e.target.checked)}
                  className="sr-only"
                />
                <div className={`flex-shrink-0 w-3 h-3 rounded border mr-2 flex items-center justify-center ${
                  isChecked 
                    ? 'bg-blue-500 border-blue-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <span className="text-xs font-medium text-gray-900">{language}</span>
              </label>
            );
          })}
        </div>
        
        {PROGRAMMING_LANGUAGES.length > 8 && (
          <button
            type="button"
            onClick={() => setShowAllLanguages(!showAllLanguages)}
            className="mt-2 text-sm text-blue-600 hover:text-blue-500"
          >
            {showAllLanguages ? 'Show less' : `Show ${PROGRAMMING_LANGUAGES.length - 8} more languages`}
          </button>
        )}
      </div>

      {/* Frameworks */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which frameworks or libraries do you use? (Optional)
        </label>
        <div className="grid grid-cols-2 sm:grid-cols-3 gap-2">
          {displayedFrameworks.map((framework) => {
            const isChecked = value.frameworks.includes(framework);
            
            return (
              <label
                key={framework}
                className={`relative flex items-center p-2 border rounded cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-blue-500 bg-blue-50' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={isChecked}
                  onChange={(e) => handleFrameworkChange(framework, e.target.checked)}
                  className="sr-only"
                />
                <div className={`flex-shrink-0 w-3 h-3 rounded border mr-2 flex items-center justify-center ${
                  isChecked 
                    ? 'bg-blue-500 border-blue-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <span className="text-xs font-medium text-gray-900">{framework}</span>
              </label>
            );
          })}
        </div>
        
        {FRAMEWORKS.length > 8 && (
          <button
            type="button"
            onClick={() => setShowAllFrameworks(!showAllFrameworks)}
            className="mt-2 text-sm text-blue-600 hover:text-blue-500"
          >
            {showAllFrameworks ? 'Show less' : `Show ${FRAMEWORKS.length - 8} more frameworks`}
          </button>
        )}
      </div>

      {/* Skip Option */}
      <div className="text-center pt-4 border-t border-gray-200">
        <p className="text-xs text-gray-500">
          You can skip this step and set your preferences later in your profile settings
        </p>
      </div>
    </div>
  );
};

export default SoftwareBackgroundForm;