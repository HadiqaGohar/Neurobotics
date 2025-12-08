/**
 * Urdu Typography Utilities
 * Advanced utilities for Urdu text rendering and typography
 */

export interface UrduTypographyConfig {
  fontFamily: string;
  fontSize: string;
  lineHeight: string;
  letterSpacing: string;
  textAlign: 'right' | 'left' | 'center';
  direction: 'rtl' | 'ltr';
  fontFeatures: string[];
}

export interface UrduTextMetrics {
  width: number;
  height: number;
  lineCount: number;
  characterCount: number;
  wordCount: number;
}

/**
 * Font families for different Urdu text types
 */
export const UrduFontFamilies = {
  NASTALEEQ: "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Pak Nastaleeq', serif",
  NASKH: "'Noto Sans Arabic', 'Arabic Typesetting', 'Traditional Arabic', sans-serif",
  MIXED: "'Noto Nastaliq Urdu', 'Roboto', 'Segoe UI', sans-serif",
  MONOSPACE: "'Fira Code', 'Monaco', 'Consolas', monospace"
} as const;

/**
 * Typography scales optimized for Urdu text
 */
export const UrduTypographyScale = {
  xs: { fontSize: '0.875rem', lineHeight: '1.4' },
  sm: { fontSize: '1rem', lineHeight: '1.5' },
  base: { fontSize: '1.125rem', lineHeight: '1.6' },
  lg: { fontSize: '1.25rem', lineHeight: '1.6' },
  xl: { fontSize: '1.375rem', lineHeight: '1.5' },
  '2xl': { fontSize: '1.5rem', lineHeight: '1.4' },
  '3xl': { fontSize: '1.75rem', lineHeight: '1.4' },
  '4xl': { fontSize: '2rem', lineHeight: '1.3' },
  '5xl': { fontSize: '2.5rem', lineHeight: '1.2' },
  '6xl': { fontSize: '3rem', lineHeight: '1.1' }
} as const;

/**
 * OpenType font features for Urdu text
 */
export const UrduFontFeatures = {
  BASIC: ['kern', 'liga', 'calt', 'ccmp'],
  ADVANCED: ['kern', 'liga', 'calt', 'ccmp', 'mark', 'mkmk', 'init', 'medi', 'fina', 'isol'],
  CONTEXTUAL: ['calt', 'ccmp', 'init', 'medi', 'fina', 'isol'],
  LIGATURES: ['liga', 'clig', 'dlig', 'hlig'],
  DIACRITICS: ['mark', 'mkmk', 'ccmp']
} as const;

/**
 * Urdu punctuation and special characters
 */
export const UrduPunctuation = {
  FULL_STOP: '۔',
  QUESTION_MARK: '؟',
  COMMA: '،',
  SEMICOLON: '؍',
  COLON: ':',
  EXCLAMATION: '!',
  QUOTATION_START: '«',
  QUOTATION_END: '»',
  PARENTHESIS_START: ')',
  PARENTHESIS_END: '(',
  BRACKET_START: ']',
  BRACKET_END: '['
} as const;

/**
 * Urdu numerals (Arabic-Indic digits)
 */
export const UrduNumerals = {
  '0': '۰',
  '1': '۱',
  '2': '۲',
  '3': '۳',
  '4': '۴',
  '5': '۵',
  '6': '۶',
  '7': '۷',
  '8': '۸',
  '9': '۹'
} as const;

/**
 * Check if text contains Urdu characters
 */
export const containsUrduText = (text: string): boolean => {
  const urduRegex = /[\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]/;
  return urduRegex.test(text);
};

/**
 * Check if text is primarily Urdu
 */
export const isPrimarilyUrdu = (text: string): boolean => {
  const urduRegex = /[\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]/g;
  const urduMatches = text.match(urduRegex) || [];
  const totalChars = text.replace(/\s/g, '').length;
  
  return totalChars > 0 && (urduMatches.length / totalChars) > 0.5;
};

/**
 * Convert English numerals to Urdu numerals
 */
export const convertToUrduNumerals = (text: string): string => {
  return text.replace(/[0-9]/g, (digit) => UrduNumerals[digit as keyof typeof UrduNumerals] || digit);
};

/**
 * Convert Urdu numerals to English numerals
 */
export const convertToEnglishNumerals = (text: string): string => {
  const urduToEnglish = Object.fromEntries(
    Object.entries(UrduNumerals).map(([eng, urdu]) => [urdu, eng])
  );
  
  return text.replace(/[۰-۹]/g, (digit) => urduToEnglish[digit] || digit);
};

/**
 * Normalize Urdu punctuation
 */
export const normalizeUrduPunctuation = (text: string): string => {
  return text
    .replace(/\./g, UrduPunctuation.FULL_STOP)
    .replace(/\?/g, UrduPunctuation.QUESTION_MARK)
    .replace(/,/g, UrduPunctuation.COMMA)
    .replace(/;/g, UrduPunctuation.SEMICOLON);
};

/**
 * Get optimal typography configuration for Urdu text
 */
export const getUrduTypographyConfig = (
  textType: 'heading' | 'paragraph' | 'caption' | 'code' = 'paragraph',
  size: keyof typeof UrduTypographyScale = 'base',
  features: keyof typeof UrduFontFeatures = 'ADVANCED'
): UrduTypographyConfig => {
  const scale = UrduTypographyScale[size];
  const fontFeatures = UrduFontFeatures[features];
  
  let fontFamily = UrduFontFamilies.NASTALEEQ;
  let letterSpacing = '0em';
  
  switch (textType) {
    case 'heading':
      fontFamily = UrduFontFamilies.NASTALEEQ;
      letterSpacing = '-0.025em';
      break;
    case 'paragraph':
      fontFamily = UrduFontFamilies.NASTALEEQ;
      letterSpacing = '0em';
      break;
    case 'caption':
      fontFamily = UrduFontFamilies.NASKH;
      letterSpacing = '0.025em';
      break;
    case 'code':
      fontFamily = UrduFontFamilies.MONOSPACE;
      letterSpacing = '0em';
      break;
  }
  
  return {
    fontFamily,
    fontSize: scale.fontSize,
    lineHeight: scale.lineHeight,
    letterSpacing,
    textAlign: 'right',
    direction: 'rtl',
    fontFeatures
  };
};

/**
 * Apply typography configuration to an element
 */
export const applyUrduTypography = (
  element: HTMLElement,
  config: UrduTypographyConfig
): void => {
  element.style.fontFamily = config.fontFamily;
  element.style.fontSize = config.fontSize;
  element.style.lineHeight = config.lineHeight;
  element.style.letterSpacing = config.letterSpacing;
  element.style.textAlign = config.textAlign;
  element.style.direction = config.direction;
  
  // Apply font features
  const fontFeatureSettings = config.fontFeatures
    .map(feature => `"${feature}" 1`)
    .join(', ');
  
  element.style.fontFeatureSettings = fontFeatureSettings;
  element.style.textRendering = 'optimizeLegibility';
  element.style.webkitFontSmoothing = 'antialiased';
  element.style.mozOsxFontSmoothing = 'grayscale';
};

/**
 * Measure Urdu text dimensions
 */
export const measureUrduText = (
  text: string,
  config: UrduTypographyConfig,
  maxWidth?: number
): UrduTextMetrics => {
  // Create a temporary element for measurement
  const measureElement = document.createElement('div');
  measureElement.style.position = 'absolute';
  measureElement.style.visibility = 'hidden';
  measureElement.style.whiteSpace = maxWidth ? 'normal' : 'nowrap';
  measureElement.style.width = maxWidth ? `${maxWidth}px` : 'auto';
  measureElement.textContent = text;
  
  // Apply typography configuration
  applyUrduTypography(measureElement, config);
  
  // Add to DOM for measurement
  document.body.appendChild(measureElement);
  
  const rect = measureElement.getBoundingClientRect();
  const lineHeight = parseFloat(getComputedStyle(measureElement).lineHeight);
  const lineCount = Math.ceil(rect.height / lineHeight);
  
  // Clean up
  document.body.removeChild(measureElement);
  
  return {
    width: rect.width,
    height: rect.height,
    lineCount,
    characterCount: text.length,
    wordCount: text.trim().split(/\s+/).length
  };
};

/**
 * Generate CSS for Urdu typography
 */
export const generateUrduTypographyCSS = (
  selector: string,
  config: UrduTypographyConfig
): string => {
  const fontFeatureSettings = config.fontFeatures
    .map(feature => `"${feature}" 1`)
    .join(', ');
  
  return `
${selector} {
  font-family: ${config.fontFamily};
  font-size: ${config.fontSize};
  line-height: ${config.lineHeight};
  letter-spacing: ${config.letterSpacing};
  text-align: ${config.textAlign};
  direction: ${config.direction};
  font-feature-settings: ${fontFeatureSettings};
  text-rendering: optimizeLegibility;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}`;
};

/**
 * Optimize Urdu text for display
 */
export const optimizeUrduText = (text: string): string => {
  let optimized = text;
  
  // Normalize whitespace
  optimized = optimized.replace(/\s+/g, ' ').trim();
  
  // Convert numerals if needed
  if (containsUrduText(optimized)) {
    optimized = convertToUrduNumerals(optimized);
    optimized = normalizeUrduPunctuation(optimized);
  }
  
  // Remove unnecessary characters
  optimized = optimized.replace(/[\u200B-\u200D\uFEFF]/g, ''); // Zero-width characters
  
  return optimized;
};

/**
 * Split Urdu text into lines for better rendering
 */
export const splitUrduTextIntoLines = (
  text: string,
  maxWidth: number,
  config: UrduTypographyConfig
): string[] => {
  const words = text.split(/\s+/);
  const lines: string[] = [];
  let currentLine = '';
  
  for (const word of words) {
    const testLine = currentLine ? `${currentLine} ${word}` : word;
    const metrics = measureUrduText(testLine, config);
    
    if (metrics.width <= maxWidth) {
      currentLine = testLine;
    } else {
      if (currentLine) {
        lines.push(currentLine);
        currentLine = word;
      } else {
        // Word is too long, force break
        lines.push(word);
      }
    }
  }
  
  if (currentLine) {
    lines.push(currentLine);
  }
  
  return lines;
};

/**
 * Create responsive Urdu typography
 */
export const createResponsiveUrduTypography = (
  baseConfig: UrduTypographyConfig,
  breakpoints: { [key: string]: Partial<UrduTypographyConfig> }
): string => {
  let css = generateUrduTypographyCSS('.urdu-responsive', baseConfig);
  
  for (const [breakpoint, config] of Object.entries(breakpoints)) {
    const mergedConfig = { ...baseConfig, ...config };
    css += `
@media ${breakpoint} {
${generateUrduTypographyCSS('.urdu-responsive', mergedConfig)}
}`;
  }
  
  return css;
};

/**
 * Validate Urdu font support
 */
export const validateUrduFontSupport = async (): Promise<{
  nastaleeq: boolean;
  naskh: boolean;
  fallback: boolean;
}> => {
  const testText = 'اردو ٹیسٹ';
  
  const checkFont = async (fontFamily: string): Promise<boolean> => {
    try {
      const canvas = document.createElement('canvas');
      const context = canvas.getContext('2d');
      
      if (!context) return false;
      
      // Test with fallback font
      context.font = '20px serif';
      const fallbackWidth = context.measureText(testText).width;
      
      // Test with target font
      context.font = `20px ${fontFamily}`;
      const targetWidth = context.measureText(testText).width;
      
      return Math.abs(targetWidth - fallbackWidth) > 1;
    } catch {
      return false;
    }
  };
  
  const [nastaleeq, naskh, fallback] = await Promise.all([
    checkFont(UrduFontFamilies.NASTALEEQ),
    checkFont(UrduFontFamilies.NASKH),
    checkFont('serif')
  ]);
  
  return { nastaleeq, naskh, fallback };
};

/**
 * Load Urdu fonts dynamically
 */
export const loadUrduFonts = async (): Promise<void> => {
  if ('fonts' in document) {
    const fontPromises = [
      new FontFace('Noto Nastaliq Urdu', 'url(/fonts/NotoNastaliqUrdu-Regular.woff2)').load(),
      new FontFace('Noto Sans Arabic', 'url(/fonts/NotoSansArabic-Regular.woff2)').load()
    ];
    
    try {
      const fonts = await Promise.all(fontPromises);
      fonts.forEach(font => {
        document.fonts.add(font);
      });
      
      // Add loaded class to document
      document.documentElement.classList.add('urdu-fonts-loaded');
    } catch (error) {
      console.warn('Failed to load Urdu fonts:', error);
    }
  }
};

/**
 * Initialize Urdu typography system
 */
export const initializeUrduTypography = async (): Promise<void> => {
  // Load fonts
  await loadUrduFonts();
  
  // Validate font support
  const fontSupport = await validateUrduFontSupport();
  
  // Add support classes to document
  if (fontSupport.nastaleeq) {
    document.documentElement.classList.add('urdu-nastaleeq-supported');
  }
  if (fontSupport.naskh) {
    document.documentElement.classList.add('urdu-naskh-supported');
  }
  
  // Set up automatic text optimization
  const observer = new MutationObserver((mutations) => {
    mutations.forEach((mutation) => {
      mutation.addedNodes.forEach((node) => {
        if (node.nodeType === Node.TEXT_NODE && node.textContent) {
          const optimized = optimizeUrduText(node.textContent);
          if (optimized !== node.textContent) {
            node.textContent = optimized;
          }
        }
      });
    });
  });
  
  observer.observe(document.body, {
    childList: true,
    subtree: true,
    characterData: true
  });
};

export default {
  UrduFontFamilies,
  UrduTypographyScale,
  UrduFontFeatures,
  UrduPunctuation,
  UrduNumerals,
  containsUrduText,
  isPrimarilyUrdu,
  convertToUrduNumerals,
  convertToEnglishNumerals,
  normalizeUrduPunctuation,
  getUrduTypographyConfig,
  applyUrduTypography,
  measureUrduText,
  generateUrduTypographyCSS,
  optimizeUrduText,
  splitUrduTextIntoLines,
  createResponsiveUrduTypography,
  validateUrduFontSupport,
  loadUrduFonts,
  initializeUrduTypography
};