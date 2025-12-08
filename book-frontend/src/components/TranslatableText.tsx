/**
 * Translatable Text Component
 * Automatically translates text based on current language
 */

import React, { useState, useEffect } from 'react';
import { useMultilingualContext } from '../contexts/MultilingualContext';

interface TranslatableTextProps {
  children: string;
  className?: string;
  tag?: keyof React.JSX.IntrinsicElements;
  fallback?: string;
  enableTranslation?: boolean;
}

const TranslatableText: React.FC<TranslatableTextProps> = ({
  children,
  className = '',
  tag: Tag = 'span',
  fallback,
  enableTranslation = true
}) => {
  const { currentLanguage, translateText, isTranslating } = useMultilingualContext();
  const [translatedText, setTranslatedText] = useState<string>(children);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!enableTranslation || currentLanguage === 'en') {
      setTranslatedText(children);
      return;
    }

    const translateContent = async () => {
      setIsLoading(true);
      setError(null);
      
      try {
        const translated = await translateText(children);
        setTranslatedText(translated);
      } catch (error) {
        console.error('Translation error:', error);
        setError('Translation failed');
        setTranslatedText(fallback || children);
      } finally {
        setIsLoading(false);
      }
    };

    translateContent();
  }, [children, currentLanguage, translateText, enableTranslation, fallback]);

  const Component = Tag as React.ElementType;
  
  return (
    <Component 
      className={`${className} ${isLoading || isTranslating ? 'opacity-70' : ''}`}
      title={error ? `Translation error: ${error}` : undefined}
    >
      {translatedText}
      {(isLoading || isTranslating) && (
        <span className=\"ml-1 inline-block w-2 h-2 bg-blue-500 rounded-full animate-pulse\" aria-label=\"Translating...\"></span>
      )}
    </Component>
  );
};

export default TranslatableText;