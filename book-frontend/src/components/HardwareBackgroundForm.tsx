/**
 * Hardware Background Collection Component
 */

import React, { useState } from 'react';
import {
  HardwareCategory,
  ExperienceLevel,
  HardwareBackground,
  HARDWARE_CATEGORY_LABELS,
  EXPERIENCE_LEVEL_LABELS,
  HARDWARE_PLATFORMS,
  HARDWARE_COMPONENTS,
} from '../types/personalization';

interface HardwareBackgroundFormProps {
  value: HardwareBackground;
  onChange: (background: HardwareBackground) => void;
  className?: string;
}

const HardwareBackgroundForm: React.FC<HardwareBackgroundFormProps> = ({
  value,
  onChange,
  className = '',
}) => {
  const [showAllPlatforms, setShowAllPlatforms] = useState(false);
  const [showAllComponents, setShowAllComponents] = useState(false);

  const handleCategoryChange = (category: HardwareCategory, checked: boolean) => {
    const newCategories = checked
      ? [...value.categories, category]
      : value.categories.filter(c => c !== category);
    
    onChange({
      ...value,
      categories: newCategories,
    });
  };

  const handlePlatformChange = (platform: string, checked: boolean) => {
    const newPlatforms = checked
      ? [...value.platforms, platform]
      : value.platforms.filter(p => p !== platform);
    
    onChange({
      ...value,
      platforms: newPlatforms,
    });
  };

  const handleComponentChange = (component: string, checked: boolean) => {
    const newComponents = checked
      ? [...value.components, component]
      : value.components.filter(c => c !== component);
    
    onChange({
      ...value,
      components: newComponents,
    });
  };

  const displayedPlatforms = showAllPlatforms 
    ? HARDWARE_PLATFORMS 
    : HARDWARE_PLATFORMS.slice(0, 8);

  const displayedComponents = showAllComponents 
    ? HARDWARE_COMPONENTS 
    : HARDWARE_COMPONENTS.slice(0, 8);

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="text-center">
        <h3 className="text-lg font-medium text-gray-900 mb-2">
          Hardware Development Background
        </h3>
        <p className="text-sm text-gray-600">
          Tell us about your hardware and electronics experience to get relevant examples
        </p>
      </div>

      {/* Hardware Categories */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which areas of hardware development interest you? (Select all that apply)
        </label>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
          {Object.entries(HARDWARE_CATEGORY_LABELS).map(([key, label]) => {
            const category = key as HardwareCategory;
            const isChecked = value.categories.includes(category);
            
            return (
              <label
                key={category}
                className={`relative flex items-center p-3 border rounded-lg cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-purple-500 bg-purple-50 ring-1 ring-purple-500' 
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
                    ? 'bg-purple-500 border-purple-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-3 h-3 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <div>
                  <span className="text-sm font-medium text-gray-900">{label}</span>
                  {/* Add category descriptions */}
                  {category === HardwareCategory.IOT_DEVICES && (
                    <p className="text-xs text-gray-500 mt-1">Smart devices, sensors, connectivity</p>
                  )}
                  {category === HardwareCategory.ROBOTICS && (
                    <p className="text-xs text-gray-500 mt-1">Robot control, actuators, navigation</p>
                  )}
                  {category === HardwareCategory.EMBEDDED_SYSTEMS && (
                    <p className="text-xs text-gray-500 mt-1">Microcontrollers, real-time systems</p>
                  )}
                  {category === HardwareCategory.ELECTRONICS && (
                    <p className="text-xs text-gray-500 mt-1">Circuit design, analog/digital circuits</p>
                  )}
                </div>
              </label>
            );
          })}
        </div>
      </div>

      {/* Experience Level */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          What's your hardware development experience level?
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
                    ? 'border-purple-500 bg-purple-50 ring-1 ring-purple-500' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="radio"
                  name="hardwareExperienceLevel"
                  value={level}
                  checked={isSelected}
                  onChange={(e) => onChange({ ...value, experienceLevel: e.target.value as ExperienceLevel })}
                  className="sr-only"
                />
                <span className={`text-sm font-medium ${
                  isSelected ? 'text-purple-900' : 'text-gray-900'
                }`}>
                  {label}
                </span>
              </label>
            );
          })}
        </div>
        
        {/* Experience level descriptions */}
        <div className="mt-2 text-xs text-gray-500">
          {value.experienceLevel === ExperienceLevel.BEGINNER && (
            <p>New to hardware development, learning basics</p>
          )}
          {value.experienceLevel === ExperienceLevel.INTERMEDIATE && (
            <p>Some experience with basic circuits and microcontrollers</p>
          )}
          {value.experienceLevel === ExperienceLevel.ADVANCED && (
            <p>Comfortable with complex designs and multiple platforms</p>
          )}
          {value.experienceLevel === ExperienceLevel.EXPERT && (
            <p>Professional experience, can design from scratch</p>
          )}
        </div>
      </div>

      {/* Hardware Platforms */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which hardware platforms have you worked with? (Optional)
        </label>
        <div className="grid grid-cols-2 sm:grid-cols-3 gap-2">
          {displayedPlatforms.map((platform) => {
            const isChecked = value.platforms.includes(platform);
            
            return (
              <label
                key={platform}
                className={`relative flex items-center p-2 border rounded cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-purple-500 bg-purple-50' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={isChecked}
                  onChange={(e) => handlePlatformChange(platform, e.target.checked)}
                  className="sr-only"
                />
                <div className={`flex-shrink-0 w-3 h-3 rounded border mr-2 flex items-center justify-center ${
                  isChecked 
                    ? 'bg-purple-500 border-purple-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <span className="text-xs font-medium text-gray-900">{platform}</span>
              </label>
            );
          })}
        </div>
        
        {HARDWARE_PLATFORMS.length > 8 && (
          <button
            type="button"
            onClick={() => setShowAllPlatforms(!showAllPlatforms)}
            className="mt-2 text-sm text-purple-600 hover:text-purple-500"
          >
            {showAllPlatforms ? 'Show less' : `Show ${HARDWARE_PLATFORMS.length - 8} more platforms`}
          </button>
        )}
      </div>

      {/* Hardware Components */}
      <div>
        <label className="block text-sm font-medium text-gray-700 mb-3">
          Which hardware components are you familiar with? (Optional)
        </label>
        <div className="grid grid-cols-2 sm:grid-cols-3 gap-2">
          {displayedComponents.map((component) => {
            const isChecked = value.components.includes(component);
            
            return (
              <label
                key={component}
                className={`relative flex items-center p-2 border rounded cursor-pointer hover:bg-gray-50 transition-colors ${
                  isChecked 
                    ? 'border-purple-500 bg-purple-50' 
                    : 'border-gray-300'
                }`}
              >
                <input
                  type="checkbox"
                  checked={isChecked}
                  onChange={(e) => handleComponentChange(component, e.target.checked)}
                  className="sr-only"
                />
                <div className={`flex-shrink-0 w-3 h-3 rounded border mr-2 flex items-center justify-center ${
                  isChecked 
                    ? 'bg-purple-500 border-purple-500' 
                    : 'border-gray-300'
                }`}>
                  {isChecked && (
                    <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </div>
                <span className="text-xs font-medium text-gray-900">{component}</span>
              </label>
            );
          })}
        </div>
        
        {HARDWARE_COMPONENTS.length > 8 && (
          <button
            type="button"
            onClick={() => setShowAllComponents(!showAllComponents)}
            className="mt-2 text-sm text-purple-600 hover:text-purple-500"
          >
            {showAllComponents ? 'Show less' : `Show ${HARDWARE_COMPONENTS.length - 8} more components`}
          </button>
        )}
      </div>

      {/* No Hardware Experience Option */}
      <div className="bg-gray-50 rounded-lg p-4">
        <div className="flex items-start">
          <div className="flex-shrink-0">
            <svg className="h-5 w-5 text-gray-400" fill="currentColor" viewBox="0 0 20 20">
              <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clipRule="evenodd" />
            </svg>
          </div>
          <div className="ml-3">
            <h4 className="text-sm font-medium text-gray-900">New to hardware?</h4>
            <p className="text-sm text-gray-600 mt-1">
              No problem! We'll provide beginner-friendly explanations and start with the basics. 
              You can always update your preferences as you learn more.
            </p>
          </div>
        </div>
      </div>

      {/* Skip Option */}
      <div className="text-center pt-4 border-t border-gray-200">
        <p className="text-xs text-gray-500">
          You can skip this step and set your hardware preferences later in your profile settings
        </p>
      </div>
    </div>
  );
};

export default HardwareBackgroundForm;