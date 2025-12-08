/**
 * RTL (Right-to-Left) Utilities
 * Helper functions for RTL language support
 */

export interface RTLConfig {
  isRTL: boolean;
  language: string;
  direction: 'ltr' | 'rtl';
}

/**
 * List of RTL languages
 */
export const RTL_LANGUAGES = [
  'ar', // Arabic
  'he', // Hebrew
  'fa', // Persian/Farsi
  'ur', // Urdu
  'ps', // Pashto
  'sd', // Sindhi
  'ku', // Kurdish (Sorani)
  'dv', // Dhivehi/Maldivian
  'yi', // Yiddish
];

/**
 * Check if a language is RTL
 */
export const isRTLLanguage = (languageCode: string): boolean => {
  return RTL_LANGUAGES.includes(languageCode.toLowerCase());
};

/**
 * Get text direction for a language
 */
export const getTextDirection = (languageCode: string): 'ltr' | 'rtl' => {
  return isRTLLanguage(languageCode) ? 'rtl' : 'ltr';
};

/**
 * Apply RTL configuration to document
 */
export const applyRTLConfig = (config: RTLConfig): void => {
  const { isRTL, language, direction } = config;
  
  // Set document direction
  document.documentElement.dir = direction;
  document.documentElement.lang = language;
  
  // Add/remove RTL class
  if (isRTL) {
    document.documentElement.classList.add('rtl');
    document.body.classList.add('rtl-body');
  } else {
    document.documentElement.classList.remove('rtl');
    document.body.classList.remove('rtl-body');
  }
  
  // Set CSS custom properties
  document.documentElement.style.setProperty('--text-direction', direction);
  document.documentElement.style.setProperty('--start-direction', isRTL ? 'right' : 'left');
  document.documentElement.style.setProperty('--end-direction', isRTL ? 'left' : 'right');
};

/**
 * Get RTL-aware positioning
 */
export const getRTLPosition = (
  position: 'left' | 'right' | 'start' | 'end',
  isRTL: boolean
): 'left' | 'right' => {
  switch (position) {
    case 'start':
      return isRTL ? 'right' : 'left';
    case 'end':
      return isRTL ? 'left' : 'right';
    case 'left':
    case 'right':
    default:
      return position;
  }
};

/**
 * Get RTL-aware margin/padding classes
 */
export const getRTLSpacing = (
  type: 'margin' | 'padding',
  side: 'start' | 'end' | 'left' | 'right' | 'top' | 'bottom',
  size: number | string,
  isRTL: boolean
): string => {
  const prefix = type === 'margin' ? 'm' : 'p';
  
  let sideClass: string;
  switch (side) {
    case 'start':
      sideClass = isRTL ? 'r' : 'l';
      break;
    case 'end':
      sideClass = isRTL ? 'l' : 'r';
      break;
    case 'left':
      sideClass = 'l';
      break;
    case 'right':
      sideClass = 'r';
      break;
    case 'top':
      sideClass = 't';
      break;
    case 'bottom':
      sideClass = 'b';
      break;
    default:
      sideClass = '';
  }
  
  return `${prefix}${sideClass}-${size}`;
};

/**
 * Get RTL-aware flex direction
 */
export const getRTLFlexDirection = (
  direction: 'row' | 'row-reverse' | 'column' | 'column-reverse',
  isRTL: boolean
): string => {
  if (!isRTL) return `flex-${direction}`;
  
  switch (direction) {
    case 'row':
      return 'flex-row-reverse';
    case 'row-reverse':
      return 'flex-row';
    default:
      return `flex-${direction}`;
  }
};

/**
 * Get RTL-aware text alignment
 */
export const getRTLTextAlign = (
  align: 'left' | 'right' | 'center' | 'start' | 'end',
  isRTL: boolean
): string => {
  switch (align) {
    case 'start':
      return isRTL ? 'text-right' : 'text-left';
    case 'end':
      return isRTL ? 'text-left' : 'text-right';
    case 'left':
    case 'right':
    case 'center':
    default:
      return `text-${align}`;
  }
};

/**
 * Transform CSS properties for RTL
 */
export const transformCSSForRTL = (
  styles: Record<string, string | number>,
  isRTL: boolean
): Record<string, string | number> => {
  if (!isRTL) return styles;
  
  const transformed = { ...styles };
  
  // Transform positioning
  if ('left' in transformed && 'right' in transformed) {
    [transformed.left, transformed.right] = [transformed.right, transformed.left];
  } else if ('left' in transformed) {
    transformed.right = transformed.left;
    delete transformed.left;
  } else if ('right' in transformed) {
    transformed.left = transformed.right;
    delete transformed.right;
  }
  
  // Transform margins
  if ('marginLeft' in transformed && 'marginRight' in transformed) {
    [transformed.marginLeft, transformed.marginRight] = [transformed.marginRight, transformed.marginLeft];
  } else if ('marginLeft' in transformed) {
    transformed.marginRight = transformed.marginLeft;
    delete transformed.marginLeft;
  } else if ('marginRight' in transformed) {
    transformed.marginLeft = transformed.marginRight;
    delete transformed.marginRight;
  }
  
  // Transform padding
  if ('paddingLeft' in transformed && 'paddingRight' in transformed) {
    [transformed.paddingLeft, transformed.paddingRight] = [transformed.paddingRight, transformed.paddingLeft];
  } else if ('paddingLeft' in transformed) {
    transformed.paddingRight = transformed.paddingLeft;
    delete transformed.paddingLeft;
  } else if ('paddingRight' in transformed) {
    transformed.paddingLeft = transformed.paddingRight;
    delete transformed.paddingRight;
  }
  
  // Transform borders
  if ('borderLeft' in transformed && 'borderRight' in transformed) {
    [transformed.borderLeft, transformed.borderRight] = [transformed.borderRight, transformed.borderLeft];
  } else if ('borderLeft' in transformed) {
    transformed.borderRight = transformed.borderLeft;
    delete transformed.borderLeft;
  } else if ('borderRight' in transformed) {
    transformed.borderLeft = transformed.borderRight;
    delete transformed.borderRight;
  }
  
  // Transform text alignment
  if (transformed.textAlign === 'left') {
    transformed.textAlign = 'right';
  } else if (transformed.textAlign === 'right') {
    transformed.textAlign = 'left';
  }
  
  return transformed;
};

/**
 * Get font family for RTL languages
 */
export const getRTLFontFamily = (languageCode: string): string => {
  switch (languageCode.toLowerCase()) {
    case 'ar':
      return "'Noto Sans Arabic', 'Arabic Typesetting', 'Traditional Arabic', serif";
    case 'ur':
      return "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif";
    case 'fa':
      return "'Noto Sans Persian', 'Iranian Sans', serif";
    case 'he':
      return "'Noto Sans Hebrew', 'David', serif";
    case 'ps':
      return "'Noto Sans Arabic', 'Arabic Typesetting', serif";
    default:
      return "'Noto Sans', sans-serif";
  }
};

/**
 * Format numbers for RTL languages
 */
export const formatNumberForRTL = (
  number: number | string,
  languageCode: string,
  options?: Intl.NumberFormatOptions
): string => {
  try {
    const num = typeof number === 'string' ? parseFloat(number) : number;
    return new Intl.NumberFormat(languageCode, options).format(num);
  } catch (error) {
    return String(number);
  }
};

/**
 * Format date for RTL languages
 */
export const formatDateForRTL = (
  date: Date | string,
  languageCode: string,
  options?: Intl.DateTimeFormatOptions
): string => {
  try {
    const dateObj = typeof date === 'string' ? new Date(date) : date;
    return new Intl.DateTimeFormat(languageCode, options).format(dateObj);
  } catch (error) {
    return String(date);
  }
};

/**
 * Get RTL-aware keyboard navigation
 */
export const getRTLKeyboardNavigation = (
  key: string,
  isRTL: boolean
): string => {
  if (!isRTL) return key;
  
  switch (key) {
    case 'ArrowLeft':
      return 'ArrowRight';
    case 'ArrowRight':
      return 'ArrowLeft';
    case 'Home':
      return 'End';
    case 'End':
      return 'Home';
    default:
      return key;
  }
};

/**
 * Validate RTL text input
 */
export const validateRTLText = (text: string, languageCode: string): boolean => {
  if (!isRTLLanguage(languageCode)) return true;
  
  // Check for proper RTL characters
  const rtlRegex = /[\u0590-\u05FF\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]/;
  return rtlRegex.test(text);
};

/**
 * Get bidirectional text direction
 */
export const getBidiTextDirection = (text: string): 'ltr' | 'rtl' | 'mixed' => {
  const ltrRegex = /[A-Za-z]/;
  const rtlRegex = /[\u0590-\u05FF\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]/;
  
  const hasLTR = ltrRegex.test(text);
  const hasRTL = rtlRegex.test(text);
  
  if (hasLTR && hasRTL) return 'mixed';
  if (hasRTL) return 'rtl';
  return 'ltr';
};

/**
 * Wrap mixed content for proper display
 */
export const wrapMixedContent = (text: string): string => {
  const direction = getBidiTextDirection(text);
  
  if (direction === 'mixed') {
    // Add Unicode bidirectional formatting characters
    return `\u202D${text}\u202C`; // LTR override
  }
  
  return text;
};

export default {
  isRTLLanguage,
  getTextDirection,
  applyRTLConfig,
  getRTLPosition,
  getRTLSpacing,
  getRTLFlexDirection,
  getRTLTextAlign,
  transformCSSForRTL,
  getRTLFontFamily,
  formatNumberForRTL,
  formatDateForRTL,
  getRTLKeyboardNavigation,
  validateRTLText,
  getBidiTextDirection,
  wrapMixedContent,
};