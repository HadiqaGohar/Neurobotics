/**
 * Multilingual Context Provider for React application
 */

import React, { createContext, useContext, ReactNode } from 'react';
import { useMultilingual } from '../hooks/useMultilingual';
import { Language, UserLanguagePreferences } from '../services/multilingualAPI';

interface MultilingualContextType {
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

const MultilingualContext = createContext<MultilingualContextType | undefined>(undefined);

interface MultilingualProviderProps {
  children: ReactNode;
}

export const MultilingualProvider: React.FC<MultilingualProviderProps> = ({ children }) => {
  const multilingualState = useMultilingual();

  return (
    <MultilingualContext.Provider value={multilingualState}>
      {children}
    </MultilingualContext.Provider>
  );
};

export const useMultilingualContext = (): MultilingualContextType => {
  const context = useContext(MultilingualContext);
  if (context === undefined) {
    throw new Error('useMultilingualContext must be used within a MultilingualProvider');
  }
  return context;
};

export default MultilingualContext;