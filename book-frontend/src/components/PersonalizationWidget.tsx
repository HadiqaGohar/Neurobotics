/**
 * Chapter personalization widget component
 */

import React, { useState, useEffect } from 'react';
import { personalizationAPI } from '../services/personalizationAPI';

interface PersonalizationWidgetProps {
  chapterId: string;
  sectionId?: string;
  onContentChange?: (content: any) => void;
}

const PersonalizationWidget: React.FC<PersonalizationWidgetProps> = ({
  chapterId,
  sectionId,
  onContentChange
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [currentSettings, setCurrentSettings] = useState<any>(null);
  const [personalizedContent, setPersonalizedContent] = useState<any>(null);
  const [loading, setLoading] = useState(false);

  const handlePersonalizationToggle = async () => {
    setLoading(true);
    try {
      const content = await personalizationAPI.getPersonalizedContent(
        localStorage.getItem('auth_token') || '',
        chapterId,
        sectionId
      );
      setPersonalizedContent(content);
      if (onContentChange) {
        onContentChange(content);
      }
    } catch (error) {
      console.error('Error getting personalized content:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalization-widget">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="personalization-toggle-btn"
        disabled={loading}
      >
        {loading ? 'Loading...' : '🎯 Personalize'}
      </button>
      
      {isOpen && (
        <div className="personalization-panel">
          <h3>Content Personalization</h3>
          <button onClick={handlePersonalizationToggle}>
            Apply Personalization
          </button>
        </div>
      )}
    </div>
  );
};

export default PersonalizationWidget;