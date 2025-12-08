/**
 * i18n Helper Functions
 */

import i18n from './index';
import { TFunction } from 'i18next';

/**
 * Get translation function with type safety
 */
export const getTranslation = (): TFunction => {
  return i18n.t;
};

/**
 * Get current language
 */
export const getCurrentLanguage = (): string => {
  return i18n.language || 'en';
};

/**
 * Check if current language is RTL
 */
export const isRTL = (): boolean => {
  const currentLang = getCurrentLanguage();
  return ['ur', 'ar', 'fa', 'he'].includes(currentLang);
};

/**
 * Get language direction
 */
export const getLanguageDirection = (language?: string): 'ltr' | 'rtl' => {
  const lang = language || getCurrentLanguage();
  return ['ur', 'ar', 'fa', 'he'].includes(lang) ? 'rtl' : 'ltr';
};

/**
 * Change language with proper cleanup
 */
export const changeLanguage = async (language: string): Promise<void> => {
  try {
    await i18n.changeLanguage(language);
    
    // Update document attributes
    const isRTLLang = getLanguageDirection(language) === 'rtl';
    document.documentElement.dir = isRTLLang ? 'rtl' : 'ltr';
    document.documentElement.lang = language;
    
    // Update body classes
    document.body.classList.toggle('rtl', isRTLLang);
    document.body.classList.toggle('ltr', !isRTLLang);
    
    // Store preference
    localStorage.setItem('preferred_language', language);
    
    console.log(`Language changed to: ${language}`);
  } catch (error) {
    console.error('Error changing language:', error);
  }
};

/**
 * Get supported languages
 */
export const getSupportedLanguages = () => {
  return [
    {
      code: 'en',
      name: 'English',
      nativeName: 'English',
      direction: 'ltr' as const,
      flag: '🇺🇸'
    },
    {
      code: 'ur',
      name: 'Urdu',
      nativeName: 'اردو',
      direction: 'rtl' as const,
      flag: '🇵🇰'
    }
  ];
};

/**
 * Format number according to current locale
 */
export const formatNumber = (
  number: number,
  options?: Intl.NumberFormatOptions
): string => {
  const currentLang = getCurrentLanguage();
  const locale = currentLang === 'ur' ? 'ur-PK' : 'en-US';
  
  try {
    return new Intl.NumberFormat(locale, options).format(number);
  } catch (error) {
    console.error('Error formatting number:', error);
    return number.toString();
  }
};

/**
 * Format date according to current locale
 */
export const formatDate = (
  date: Date | string,
  options?: Intl.DateTimeFormatOptions
): string => {
  const currentLang = getCurrentLanguage();
  const locale = currentLang === 'ur' ? 'ur-PK' : 'en-US';
  
  try {
    const dateObj = typeof date === 'string' ? new Date(date) : date;
    return new Intl.DateTimeFormat(locale, options).format(dateObj);
  } catch (error) {
    console.error('Error formatting date:', error);
    return date.toString();
  }
};

/**
 * Format currency according to current locale
 */
export const formatCurrency = (
  amount: number,
  currency: string = 'USD'
): string => {
  const currentLang = getCurrentLanguage();
  const locale = currentLang === 'ur' ? 'ur-PK' : 'en-US';
  
  // Use appropriate currency for locale
  const localeCurrency = currentLang === 'ur' ? 'PKR' : currency;
  
  try {
    return new Intl.NumberFormat(locale, {
      style: 'currency',
      currency: localeCurrency
    }).format(amount);
  } catch (error) {
    console.error('Error formatting currency:', error);
    return `${localeCurrency} ${amount}`;
  }
};

/**
 * Get relative time string
 */
export const getRelativeTime = (date: Date | string): string => {
  const t = getTranslation();
  const now = new Date();
  const targetDate = typeof date === 'string' ? new Date(date) : date;
  const diffInSeconds = Math.floor((now.getTime() - targetDate.getTime()) / 1000);
  
  if (diffInSeconds < 60) {
    return t('time.seconds_ago', { count: diffInSeconds });
  } else if (diffInSeconds < 3600) {
    const minutes = Math.floor(diffInSeconds / 60);
    return t('time.minutes_ago', { count: minutes });
  } else if (diffInSeconds < 86400) {
    const hours = Math.floor(diffInSeconds / 3600);
    return t('time.hours_ago', { count: hours });
  } else if (diffInSeconds < 604800) {
    const days = Math.floor(diffInSeconds / 86400);
    return t('time.days_ago', { count: days });
  } else if (diffInSeconds < 2629746) {
    const weeks = Math.floor(diffInSeconds / 604800);
    return t('time.weeks_ago', { count: weeks });
  } else if (diffInSeconds < 31556952) {
    const months = Math.floor(diffInSeconds / 2629746);
    return t('time.months_ago', { count: months });
  } else {
    const years = Math.floor(diffInSeconds / 31556952);
    return t('time.years_ago', { count: years });
  }
};

/**
 * Pluralize text based on count
 */
export const pluralize = (
  key: string,
  count: number,
  options?: any
): string => {
  const t = getTranslation();
  return t(key, { count, ...options });
};

/**
 * Get text with fallback
 */
export const getTextWithFallback = (
  key: string,
  fallback: string,
  options?: any
): string => {
  const t = getTranslation();
  
  try {
    const translation = t(key, { ...options, defaultValue: fallback });
    return translation || fallback;
  } catch (error) {
    console.error(`Translation error for key "${key}":`, error);
    return fallback;
  }
};

/**
 * Check if translation exists
 */
export const hasTranslation = (key: string): boolean => {
  return i18n.exists(key);
};

/**
 * Get all translations for current language
 */
export const getAllTranslations = (): Record<string, any> => {
  const currentLang = getCurrentLanguage();
  return i18n.getResourceBundle(currentLang, 'translation') || {};
};

/**
 * Add translation dynamically
 */
export const addTranslation = (
  language: string,
  key: string,
  value: string
): void => {
  i18n.addResource(language, 'translation', key, value);
};

/**
 * Remove translation
 */
export const removeTranslation = (
  language: string,
  key: string
): void => {
  i18n.removeResourceBundle(language, 'translation');
};

/**
 * Get language-specific CSS class
 */
export const getLanguageClass = (language?: string): string => {
  const lang = language || getCurrentLanguage();
  const direction = getLanguageDirection(lang);
  return `lang-${lang} ${direction}`;
};

/**
 * Get font family for language
 */
export const getLanguageFontFamily = (language?: string): string => {
  const lang = language || getCurrentLanguage();
  
  switch (lang) {
    case 'ur':
      return "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif";
    case 'ar':
      return "'Noto Sans Arabic', 'Arabic Typesetting', serif";
    default:
      return "'Inter', 'Segoe UI', 'Roboto', sans-serif";
  }
};

/**
 * Validate RTL text
 */
export const validateRTLText = (text: string, language?: string): boolean => {
  const lang = language || getCurrentLanguage();
  
  if (!['ur', 'ar', 'fa', 'he'].includes(lang)) {
    return true; // Not an RTL language
  }
  
  // Check for RTL characters
  const rtlRegex = /[\u0590-\u05FF\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]/;
  return rtlRegex.test(text);
};

/**
 * Convert numbers to local format
 */
export const convertToLocalNumbers = (text: string, language?: string): string => {
  const lang = language || getCurrentLanguage();
  
  if (lang === 'ur') {
    // Convert to Arabic-Indic numerals if needed
    const arabicNumerals = ['٠', '١', '٢', '٣', '٤', '٥', '٦', '٧', '٨', '٩'];
    return text.replace(/[0-9]/g, (match) => arabicNumerals[parseInt(match)]);
  }
  
  return text;
};

/**
 * Get keyboard layout direction
 */
export const getKeyboardDirection = (language?: string): 'ltr' | 'rtl' => {
  return getLanguageDirection(language);
};

/**
 * Handle keyboard navigation for RTL
 */
export const handleRTLKeyNavigation = (
  event: KeyboardEvent,
  language?: string
): KeyboardEvent => {
  const isRTLLang = getLanguageDirection(language) === 'rtl';
  
  if (isRTLLang) {
    // Swap arrow keys for RTL navigation
    if (event.key === 'ArrowLeft') {
      Object.defineProperty(event, 'key', { value: 'ArrowRight' });
    } else if (event.key === 'ArrowRight') {
      Object.defineProperty(event, 'key', { value: 'ArrowLeft' });
    }
  }
  
  return event;
};

export default {
  getTranslation,
  getCurrentLanguage,
  isRTL,
  getLanguageDirection,
  changeLanguage,
  getSupportedLanguages,
  formatNumber,
  formatDate,
  formatCurrency,
  getRelativeTime,
  pluralize,
  getTextWithFallback,
  hasTranslation,
  getAllTranslations,
  addTranslation,
  removeTranslation,
  getLanguageClass,
  getLanguageFontFamily,
  validateRTLText,
  convertToLocalNumbers,
  getKeyboardDirection,
  handleRTLKeyNavigation
};