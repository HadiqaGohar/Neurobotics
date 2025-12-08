/**
 * React hook for multilingual functionality
 */

import { useState, useEffect, useCallback } from 'react';
import { multilingualAPI, Language, UserLanguagePreferences } from '../services/multilingualAPI';

interface UseMultilingualReturn {
  // Language state
  currentLanguage: string;
  supportedLanguages: Language[];
  isRTL: boolean;
  
  // Loading states
  isLoading: boolean;
  isTranslating: boolean;
  
  // Actions
  setLanguage: (language: string) => Promise<void>;
  translateText: (text: string, targetLanguage?: string) => Promise<string>;
  getUserPreferences: () => Promise<UserLanguagePreferences | null>;
  updateUserPreferences: (preferences: Partial<UserLanguagePreferences>) => Promise<boolean>;
  
  // Content
  getTranslatedContent: (contentType: string, contentId: string, language?: string) => Promise<any>;
  
  // Error state
  error: string | null;
}

export const useMultilingual = (): UseMultilingualReturn => {
  const [currentLanguage, setCurrentLanguage] = useState<string>('en');
  const [supportedLanguages, setSupportedLanguages] = useState<Language[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [translationCache, setTranslationCache] = useState<Map<string, string>>(new Map());

  // Initialize supported languages and detect user language
  useEffect(() => {
    initializeLanguages();
  }, []);

  // Update document direction when language changes
  useEffect(() => {
    const currentLangInfo = supportedLanguages.find(lang => lang.code === currentLanguage);
    if (currentLangInfo) {
      document.documentElement.dir = currentLangInfo.direction;
      document.documentElement.lang = currentLanguage;
    }
  }, [currentLanguage, supportedLanguages]);

  const initializeLanguages = async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      // Load supported languages
      const languages = await multilingualAPI.getSupportedLanguages();
      setSupportedLanguages(languages);

      // Try to get user preferences first
      const token = localStorage.getItem('auth_token');
      if (token) {
        try {
          const preferences = await multilingualAPI.getUserLanguagePreferences(token);
          if (preferences.preferred_language) {
            setCurrentLanguage(preferences.preferred_language);
            return;
          }
        } catch (error) {
          console.warn('Could not load user preferences, falling back to detection');
        }
      }

      // Fallback to language detection
      try {
        const detection = await multilingualAPI.detectUserLanguage();
        setCurrentLanguage(detection.detected_language);
      } catch (error) {
        console.warn('Language detection failed, using default');
        setCurrentLanguage('en');
      }

    } catch (error) {
      console.error('Error initializing languages:', error);
      setError('Failed to load language settings');
      // Set fallback languages
      setSupportedLanguages([
        { code: 'en', name: 'English', native_name: 'English', direction: 'ltr', is_default: true },
        { code: 'ur', name: 'Urdu', native_name: 'اردو', direction: 'rtl', is_default: false }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const setLanguage = useCallback(async (language: string) => {
    setError(null);
    
    try {
      // Update local state immediately for better UX
      setCurrentLanguage(language);
      
      // Save to localStorage
      localStorage.setItem('preferred_language', language);
      
      // Update user preferences if authenticated
      const token = localStorage.getItem('auth_token');
      if (token) {
        await multilingualAPI.updateUserLanguagePreferences(token, {
          preferred_language: language
        });
      }
      
    } catch (error) {
      console.error('Error setting language:', error);
      setError('Failed to update language preference');
    }
  }, []);

  const translateText = useCallback(async (text: string, targetLanguage?: string): Promise<string> => {
    const target = targetLanguage || currentLanguage;
    
    // Return original text if target is English or empty
    if (target === 'en' || !text.trim()) {
      return text;
    }

    // Check cache first
    const cacheKey = `${text}:${target}`;
    if (translationCache.has(cacheKey)) {
      return translationCache.get(cacheKey)!;
    }

    setIsTranslating(true);
    setError(null);
    
    try {
      // For now, return the original text as we haven't implemented AI translation yet
      // TODO: Implement actual translation service integration
      const translatedText = text; // Placeholder
      
      // Cache the result
      setTranslationCache(prev => new Map(prev.set(cacheKey, translatedText)));
      
      return translatedText;
      
    } catch (error) {
      console.error('Translation error:', error);
      setError('Translation failed');
      return text; // Return original text on error
    } finally {
      setIsTranslating(false);
    }
  }, [currentLanguage, translationCache]);

  const getUserPreferences = useCallback(async (): Promise<UserLanguagePreferences | null> => {
    const token = localStorage.getItem('auth_token');
    if (!token) return null;

    setError(null);
    
    try {
      return await multilingualAPI.getUserLanguagePreferences(token);
    } catch (error) {
      console.error('Error getting user preferences:', error);
      setError('Failed to load user preferences');
      return null;
    }
  }, []);

  const updateUserPreferences = useCallback(async (preferences: Partial<UserLanguagePreferences>): Promise<boolean> => {
    const token = localStorage.getItem('auth_token');
    if (!token) return false;

    setError(null);
    
    try {
      await multilingualAPI.updateUserLanguagePreferences(token, preferences);
      
      // Update current language if it was changed
      if (preferences.preferred_language) {
        setCurrentLanguage(preferences.preferred_language);
      }
      
      return true;
    } catch (error) {
      console.error('Error updating user preferences:', error);
      setError('Failed to update preferences');
      return false;
    }
  }, []);

  const getTranslatedContent = useCallback(async (
    contentType: string, 
    contentId: string, 
    language?: string
  ) => {
    const targetLanguage = language || currentLanguage;
    setError(null);
    
    try {
      return await multilingualAPI.getTranslatedContent(
        contentType,
        contentId,
        targetLanguage,
        true // Enable fallback
      );
    } catch (error) {
      console.error('Error getting translated content:', error);
      setError('Failed to load translated content');
      throw error;
    }
  }, [currentLanguage]);

  // Determine if current language is RTL
  const isRTL = supportedLanguages.find(lang => lang.code === currentLanguage)?.direction === 'rtl' || false;

  return {
    // State
    currentLanguage,
    supportedLanguages,
    isRTL,
    isLoading,
    isTranslating,
    error,
    
    // Actions
    setLanguage,
    translateText,
    getUserPreferences,
    updateUserPreferences,
    getTranslatedContent,
  };
};

export default useMultilingual;