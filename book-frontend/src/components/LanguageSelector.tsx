/**
 * Language selector component for multi-lingual support.
 */

import React, { useState, useEffect, useContext, createContext } from 'react';

// Types
interface Language {
  code: string;
  name: string;
  native_name: string;
  rtl: boolean;
}

interface LanguageContextType {
  currentLanguage: string;
  setLanguage: (language: string) => void;
  supportedLanguages: Language[];
  isRTL: boolean;
  translateText: (text: string, targetLang?: string) => Promise<string>;
  isTranslating: boolean;
}

// Language Context
const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

// Language Provider Component
export const LanguageProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [currentLanguage, setCurrentLanguage] = useState<string>('en');
  const [supportedLanguages, setSupportedLanguages] = useState<Language[]>([]);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationCache, setTranslationCache] = useState<Map<string, string>>(new Map());

  // Load supported languages on mount
  useEffect(() => {
    loadSupportedLanguages();
    
    // Load saved language preference
    const savedLanguage = localStorage.getItem('preferred_language');
    if (savedLanguage) {
      setCurrentLanguage(savedLanguage);
    }
  }, []);

  // Update document direction when language changes
  useEffect(() => {
    const currentLangInfo = supportedLanguages.find(lang => lang.code === currentLanguage);
    if (currentLangInfo) {
      document.documentElement.dir = currentLangInfo.rtl ? 'rtl' : 'ltr';
      document.documentElement.lang = currentLanguage;
    }
  }, [currentLanguage, supportedLanguages]);

  const loadSupportedLanguages = async () => {
    try {
      // Use our new multilingual API
      const { multilingualAPI } = await import('../services/multilingualAPI');
      const languages = await multilingualAPI.getSupportedLanguages();
      setSupportedLanguages(languages.map(lang => ({
        code: lang.code,
        name: lang.name,
        native_name: lang.native_name,
        rtl: lang.direction === 'rtl',
      })));
    } catch (error) {
      console.error('Error loading supported languages:', error);
      // Fallback to default languages
      setSupportedLanguages([
        { code: 'en', name: 'English', native_name: 'English', rtl: false },
        { code: 'ur', name: 'Urdu', native_name: 'اردو', rtl: true },
      ]);
    }
  };

  const setLanguage = (language: string) => {
    setCurrentLanguage(language);
    localStorage.setItem('preferred_language', language);
  };

  const translateText = async (text: string, targetLang?: string): Promise<string> => {
    const target = targetLang || currentLanguage;
    
    // Return original text if target is English or same as current
    if (target === 'en' || !text.trim()) {
      return text;
    }

    // Check cache first
    const cacheKey = `${text}:${target}`;
    if (translationCache.has(cacheKey)) {
      return translationCache.get(cacheKey)!;
    }

    setIsTranslating(true);
    try {
      const response = await fetch('/api/translation/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text,
          target_language: target,
          source_language: 'en',
        }),
      });

      if (response.ok) {
        const result = await response.json();
        const translatedText = result.translated_text || text;
        
        // Cache the translation
        setTranslationCache(prev => new Map(prev.set(cacheKey, translatedText)));
        
        return translatedText;
      } else {
        console.error('Translation API error:', response.statusText);
        return text;
      }
    } catch (error) {
      console.error('Translation error:', error);
      return text;
    } finally {
      setIsTranslating(false);
    }
  };

  const isRTL = supportedLanguages.find(lang => lang.code === currentLanguage)?.rtl || false;

  const contextValue: LanguageContextType = {
    currentLanguage,
    setLanguage,
    supportedLanguages,
    isRTL,
    translateText,
    isTranslating,
  };

  return (
    <LanguageContext.Provider value={contextValue}>
      {children}
    </LanguageContext.Provider>
  );
};

// Hook to use language context
export const useLanguage = (): LanguageContextType => {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};

// Language Selector Component
const LanguageSelector: React.FC<{
  className?: string;
  showFlags?: boolean;
  compact?: boolean;
}> = ({ className = '', showFlags = true, compact = false }) => {
  const { currentLanguage, setLanguage, supportedLanguages, isTranslating } = useLanguage();
  const [isOpen, setIsOpen] = useState(false);

  // Flag emojis for languages
  const languageFlags: Record<string, string> = {
    en: '🇺🇸',
    ur: '🇵🇰',
    ar: '🇸🇦',
    es: '🇪🇸',
    fr: '🇫🇷',
    de: '🇩🇪',
    zh: '🇨🇳',
    hi: '🇮🇳',
    ja: '🇯🇵',
    ko: '🇰🇷',
  };

  const currentLangInfo = supportedLanguages.find(lang => lang.code === currentLanguage);

  if (compact) {
    return (
      <div className={`relative ${className}`}>
        <select
          value={currentLanguage}
          onChange={(e) => setLanguage(e.target.value)}
          className="appearance-none bg-white border border-gray-300 rounded px-3 py-1 pr-8 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
          disabled={isTranslating}
        >
          {supportedLanguages.map((language) => (
            <option key={language.code} value={language.code}>
              {showFlags && languageFlags[language.code]} {language.native_name}
            </option>
          ))}
        </select>
        <div className="absolute inset-y-0 right-0 flex items-center px-2 pointer-events-none">
          <svg className="w-4 h-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        </div>
        {isTranslating && (
          <div className="absolute -top-1 -right-1">
            <div className="w-3 h-3 bg-blue-500 rounded-full animate-pulse"></div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className={`relative ${className}`}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center space-x-2 px-3 py-2 bg-white border border-gray-300 rounded-md shadow-sm hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500"
        disabled={isTranslating}
      >
        {showFlags && currentLangInfo && (
          <span className="text-lg">{languageFlags[currentLanguage]}</span>
        )}
        <span className="text-sm font-medium">
          {currentLangInfo?.native_name || 'Select Language'}
        </span>
        <svg
          className={`w-4 h-4 text-gray-400 transition-transform ${isOpen ? 'rotate-180' : ''}`}
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
        </svg>
        {isTranslating && (
          <div className="w-4 h-4 border-2 border-blue-500 border-t-transparent rounded-full animate-spin"></div>
        )}
      </button>

      {isOpen && (
        <div className="absolute top-full left-0 mt-1 w-64 bg-white border border-gray-200 rounded-md shadow-lg z-50">
          <div className="py-1">
            {supportedLanguages.map((language) => (
              <button
                key={language.code}
                onClick={() => {
                  setLanguage(language.code);
                  setIsOpen(false);
                }}
                className={`w-full flex items-center space-x-3 px-4 py-2 text-left hover:bg-gray-100 ${
                  currentLanguage === language.code ? 'bg-blue-50 text-blue-700' : 'text-gray-700'
                }`}
              >
                {showFlags && (
                  <span className="text-lg">{languageFlags[language.code] || '🌐'}</span>
                )}
                <div className="flex-1">
                  <div className="text-sm font-medium">{language.native_name}</div>
                  <div className="text-xs text-gray-500">{language.name}</div>
                </div>
                {currentLanguage === language.code && (
                  <svg className="w-4 h-4 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
                    <path
                      fillRule="evenodd"
                      d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
                      clipRule="evenodd"
                    />
                  </svg>
                )}
              </button>
            ))}
          </div>
          
          {/* Translation Status */}
          {isTranslating && (
            <div className="border-t border-gray-200 px-4 py-2">
              <div className="flex items-center space-x-2 text-xs text-gray-500">
                <div className="w-3 h-3 border border-blue-500 border-t-transparent rounded-full animate-spin"></div>
                <span>Translating content...</span>
              </div>
            </div>
          )}
        </div>
      )}

      {/* Click outside to close */}
      {isOpen && (
        <div
          className="fixed inset-0 z-40"
          onClick={() => setIsOpen(false)}
        />
      )}
    </div>
  );
};

// Translatable Text Component
export const TranslatableText: React.FC<{
  children: string;
  className?: string;
  tag?: keyof React.JSX.IntrinsicElements;
}> = ({ children, className = '', tag: Tag = 'span' }) => {
  const { translateText, currentLanguage } = useLanguage();
  const [translatedText, setTranslatedText] = useState<string>(children);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (currentLanguage === 'en') {
      setTranslatedText(children);
      return;
    }

    const translateContent = async () => {
      setIsLoading(true);
      try {
        const translated = await translateText(children);
        setTranslatedText(translated);
      } catch (error) {
        console.error('Translation error:', error);
        setTranslatedText(children);
      } finally {
        setIsLoading(false);
      }
    };

    translateContent();
  }, [children, currentLanguage, translateText]);

  const Component = Tag as React.ElementType;
  
  return (
    <Component className={`${className} ${isLoading ? 'opacity-70' : ''}`}>
      {translatedText}
      {isLoading && (
        <span className="ml-1 inline-block w-2 h-2 bg-blue-500 rounded-full animate-pulse"></span>
      )}
    </Component>
  );
};

// Language Detection Component
export const LanguageDetector: React.FC<{
  text: string;
  onLanguageDetected?: (language: string, confidence: number) => void;
}> = ({ text, onLanguageDetected }) => {
  const [detectedLanguage, setDetectedLanguage] = useState<string | null>(null);
  const [confidence, setConfidence] = useState<number>(0);
  const [isDetecting, setIsDetecting] = useState(false);

  useEffect(() => {
    if (!text.trim()) return;

    const detectLanguage = async () => {
      setIsDetecting(true);
      try {
        const response = await fetch('/api/translation/detect', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ text }),
        });

        if (response.ok) {
          const result = await response.json();
          setDetectedLanguage(result.language);
          setConfidence(result.confidence);
          
          if (onLanguageDetected) {
            onLanguageDetected(result.language, result.confidence);
          }
        }
      } catch (error) {
        console.error('Language detection error:', error);
      } finally {
        setIsDetecting(false);
      }
    };

    const timeoutId = setTimeout(detectLanguage, 500); // Debounce
    return () => clearTimeout(timeoutId);
  }, [text, onLanguageDetected]);

  if (!detectedLanguage || isDetecting) {
    return null;
  }

  return (
    <div className="text-xs text-gray-500 mt-1">
      Detected: {detectedLanguage} ({Math.round(confidence * 100)}% confidence)
    </div>
  );
};

// RTL Support Component
export const RTLWrapper: React.FC<{
  children: React.ReactNode;
  className?: string;
}> = ({ children, className = '' }) => {
  const { isRTL } = useLanguage();

  return (
    <div className={`${className} ${isRTL ? 'rtl' : 'ltr'}`} dir={isRTL ? 'rtl' : 'ltr'}>
      {children}
    </div>
  );
};

// Translation Quality Indicator
export const TranslationQuality: React.FC<{
  originalText: string;
  translatedText: string;
  targetLanguage: string;
}> = ({ originalText, translatedText, targetLanguage }) => {
  const [quality, setQuality] = useState<string>('unknown');
  const [isValidating, setIsValidating] = useState(false);

  useEffect(() => {
    const validateQuality = async () => {
      if (!originalText || !translatedText || targetLanguage === 'en') return;

      setIsValidating(true);
      try {
        const response = await fetch('/api/translation/validate', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            original: originalText,
            translated: translatedText,
            target_language: targetLanguage,
          }),
        });

        if (response.ok) {
          const result = await response.json();
          setQuality(result.quality_estimate);
        }
      } catch (error) {
        console.error('Quality validation error:', error);
      } finally {
        setIsValidating(false);
      }
    };

    validateQuality();
  }, [originalText, translatedText, targetLanguage]);

  if (isValidating) {
    return (
      <div className="flex items-center space-x-1 text-xs text-gray-500">
        <div className="w-2 h-2 bg-gray-400 rounded-full animate-pulse"></div>
        <span>Validating...</span>
      </div>
    );
  }

  const qualityColors = {
    good: 'text-green-600',
    fair: 'text-yellow-600',
    poor: 'text-red-600',
    unknown: 'text-gray-500',
  };

  return (
    <div className={`text-xs ${qualityColors[quality as keyof typeof qualityColors]}`}>
      Translation quality: {quality}
    </div>
  );
};

export default LanguageSelector;