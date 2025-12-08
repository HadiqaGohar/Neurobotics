/**
 * Internationalization (i18n) configuration
 */

import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import LanguageDetector from 'i18next-browser-languagedetector';
import Backend from 'i18next-http-backend';

// Import translation files
import enTranslations from '../locales/en.json';
import urTranslations from '../locales/ur.json';

// Language resources
const resources = {
  en: {
    translation: enTranslations
  },
  ur: {
    translation: urTranslations
  }
};

// Language detection options
const detectionOptions = {
  // Order of language detection methods
  order: [
    'localStorage',
    'sessionStorage', 
    'navigator',
    'htmlTag',
    'path',
    'subdomain'
  ],
  
  // Cache user language
  caches: ['localStorage', 'sessionStorage'],
  
  // Exclude certain detection methods
  excludeCacheFor: ['cimode'],
  
  // Check for language in localStorage
  lookupLocalStorage: 'preferred_language',
  lookupSessionStorage: 'i18nextLng',
  
  // HTML tag attribute to look for language
  htmlTag: document.documentElement,
  
  // Convert language codes
  convertDetectedLanguage: (lng: string) => {
    // Handle language variants
    if (lng.startsWith('ur')) return 'ur';
    if (lng.startsWith('en')) return 'en';
    return lng;
  }
};

// Initialize i18n
i18n
  .use(Backend)
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    resources,
    
    // Fallback language
    fallbackLng: 'en',
    
    // Default language
    lng: 'en',
    
    // Language detection
    detection: detectionOptions,
    
    // Interpolation options
    interpolation: {
      escapeValue: false, // React already escapes values
      formatSeparator: ',',
      format: (value, format, lng) => {
        // Custom formatting
        if (format === 'uppercase') return value.toUpperCase();
        if (format === 'lowercase') return value.toLowerCase();
        if (format === 'number' && lng === 'ur') {
          // Format numbers for Urdu (use Arabic-Indic numerals if needed)
          return new Intl.NumberFormat('ur-PK').format(value);
        }
        return value;
      }
    },
    
    // React options
    react: {
      useSuspense: false,
      bindI18n: 'languageChanged loaded',
      bindI18nStore: 'added removed',
      transEmptyNodeValue: '',
      transSupportBasicHtmlNodes: true,
      transKeepBasicHtmlNodesFor: ['br', 'strong', 'i', 'em', 'span']
    },
    
    // Backend options for loading translations
    backend: {
      loadPath: '/locales/{{lng}}.json',
      addPath: '/locales/add/{{lng}}/{{ns}}',
      allowMultiLoading: false,
      crossDomain: false,
      withCredentials: false,
      requestOptions: {
        mode: 'cors',
        credentials: 'same-origin',
        cache: 'default'
      }
    },
    
    // Debug mode (disable in production)
    debug: process.env.NODE_ENV === 'development',
    
    // Namespace options
    defaultNS: 'translation',
    ns: ['translation'],
    
    // Key separator
    keySeparator: '.',
    nsSeparator: ':',
    
    // Pluralization
    pluralSeparator: '_',
    contextSeparator: '_',
    
    // Missing key handling
    saveMissing: process.env.NODE_ENV === 'development',
    missingKeyHandler: (lng, ns, key, fallbackValue) => {
      if (process.env.NODE_ENV === 'development') {
        console.warn(`Missing translation key: ${key} for language: ${lng}`);
      }
    },
    
    // Post-processing
    postProcess: ['interval', 'plural'],
    
    // Clean code on production
    cleanCode: true,
    
    // Load languages
    preload: ['en', 'ur'],
    
    // Whitelist languages
    supportedLngs: ['en', 'ur'],
    nonExplicitSupportedLngs: false,
    
    // Load options
    load: 'languageOnly',
    
    // Update missing keys
    updateMissing: process.env.NODE_ENV === 'development',
    
    // Return objects for missing keys
    returnObjects: false,
    
    // Return empty string for missing keys
    returnEmptyString: false,
    
    // Return null for missing keys
    returnNull: false,
    
    // Join arrays
    joinArrays: false,
    
    // Override options
    overloadTranslationOptionHandler: (args) => {
      return {
        defaultValue: args[1]
      };
    }
  });

// Language change handler
i18n.on('languageChanged', (lng) => {
  // Update document direction and language
  const isRTL = lng === 'ur';
  document.documentElement.dir = isRTL ? 'rtl' : 'ltr';
  document.documentElement.lang = lng;
  
  // Update body class for styling
  document.body.classList.toggle('rtl', isRTL);
  document.body.classList.toggle('ltr', !isRTL);
  
  // Store language preference
  localStorage.setItem('preferred_language', lng);
  
  // Emit custom event for other components
  window.dispatchEvent(new CustomEvent('languageChanged', { 
    detail: { language: lng, isRTL } 
  }));
  
  console.log(`Language changed to: ${lng}`);
});

// Error handler
i18n.on('failedLoading', (lng, ns, msg) => {
  console.error(`Failed to load language ${lng} namespace ${ns}: ${msg}`);
});

// Loaded handler
i18n.on('loaded', (loaded) => {
  console.log('i18n loaded:', loaded);
});

// Initialize direction on startup
const currentLang = i18n.language || 'en';
const isRTL = currentLang === 'ur';
document.documentElement.dir = isRTL ? 'rtl' : 'ltr';
document.documentElement.lang = currentLang;
document.body.classList.toggle('rtl', isRTL);
document.body.classList.toggle('ltr', !isRTL);

export default i18n;