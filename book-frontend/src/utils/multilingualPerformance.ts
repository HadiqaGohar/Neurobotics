/**
 * Multilingual Performance Optimization Utilities
 * Client-side performance optimizations for multilingual content
 */

interface FontLoadingConfig {
  fontFamily: string;
  variants: string[];
  subset: string;
  display: string;
  preload: boolean;
}

interface LazyLoadingConfig {
  threshold: number;
  rootMargin: string;
  batchSize: number;
}

interface CacheConfig {
  maxSize: number;
  ttl: number; // Time to live in milliseconds
}

interface PerformanceMetrics {
  translationLoadTime: number[];
  fontLoadTime: number[];
  languageSwitchTime: number[];
  cacheHitRatio: number;
}

class MultilingualPerformanceOptimizer {
  private translationCache: Map<string, { data: any; timestamp: number; ttl: number }>;
  private fontCache: Map<string, FontFace>;
  private intersectionObserver: IntersectionObserver | null;
  private performanceMetrics: PerformanceMetrics;
  private cacheConfig: CacheConfig;
  private lazyLoadingConfig: LazyLoadingConfig;

  constructor() {
    this.translationCache = new Map();
    this.fontCache = new Map();
    this.intersectionObserver = null;
    this.performanceMetrics = {
      translationLoadTime: [],
      fontLoadTime: [],
      languageSwitchTime: [],
      cacheHitRatio: 0
    };
    this.cacheConfig = {
      maxSize: 1000,
      ttl: 30 * 60 * 1000 // 30 minutes
    };
    this.lazyLoadingConfig = {
      threshold: 0.1,
      rootMargin: '50px',
      batchSize: 5
    };

    this.initializePerformanceObserver();
    this.setupIntersectionObserver();
  }

  /**
   * Initialize performance observer for monitoring
   */
  private initializePerformanceObserver(): void {
    if ('PerformanceObserver' in window) {
      try {
        const observer = new PerformanceObserver((list) => {
          const entries = list.getEntries();
          entries.forEach((entry) => {
            if (entry.name.includes('translation')) {
              this.recordMetric('translationLoadTime', entry.duration);
            } else if (entry.name.includes('font')) {
              this.recordMetric('fontLoadTime', entry.duration);
            } else if (entry.name.includes('language-switch')) {
              this.recordMetric('languageSwitchTime', entry.duration);
            }
          });
        });

        observer.observe({ entryTypes: ['measure', 'navigation', 'resource'] });
      } catch (error) {
        console.warn('Performance Observer not supported:', error);
      }
    }
  }

  /**
   * Set up intersection observer for lazy loading
   */
  private setupIntersectionObserver(): void {
    if ('IntersectionObserver' in window) {
      this.intersectionObserver = new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            if (entry.isIntersecting) {
              const element = entry.target as HTMLElement;
              this.loadTranslationContent(element);
              this.intersectionObserver?.unobserve(element);
            }
          });
        },
        {
          threshold: this.lazyLoadingConfig.threshold,
          rootMargin: this.lazyLoadingConfig.rootMargin
        }
      );
    }
  }

  /**
   * Optimize font loading for specific language
   */
  async optimizeFontLoading(languageCode: string): Promise<void> {
    const startTime = performance.now();

    try {
      const fontConfig = this.getFontConfig(languageCode);
      
      // Preload critical fonts
      await this.preloadFonts(fontConfig);
      
      // Set up font display optimization
      this.optimizeFontDisplay(fontConfig);
      
      // Cache font configuration
      this.cacheFontConfig(languageCode, fontConfig);

      const endTime = performance.now();
      this.recordMetric('fontLoadTime', endTime - startTime);
    } catch (error) {
      console.error('Font optimization failed:', error);
    }
  }

  /**
   * Get font configuration for language
   */
  private getFontConfig(languageCode: string): FontLoadingConfig {
    const fontConfigs: Record<string, FontLoadingConfig> = {
      'ur': {
        fontFamily: 'Noto Nastaliq Urdu',
        variants: ['400', '700'],
        subset: 'urdu',
        display: 'swap',
        preload: true
      },
      'ar': {
        fontFamily: 'Noto Sans Arabic',
        variants: ['400', '700'],
        subset: 'arabic',
        display: 'swap',
        preload: true
      },
      'en': {
        fontFamily: 'Inter',
        variants: ['400', '500', '700'],
        subset: 'latin',
        display: 'swap',
        preload: false
      }
    };

    return fontConfigs[languageCode] || fontConfigs['en'];
  }

  /**
   * Preload critical fonts
   */
  private async preloadFonts(config: FontLoadingConfig): Promise<void> {
    if (!config.preload) return;

    const promises = config.variants.map(async (weight) => {
      const fontUrl = this.generateFontUrl(config.fontFamily, weight, config.subset);
      
      try {
        // Create font face
        const fontFace = new FontFace(
          config.fontFamily,
          `url(${fontUrl})`,
          {
            weight,
            display: config.display as FontDisplay
          }
        );

        // Load font
        await fontFace.load();
        
        // Add to document fonts
        document.fonts.add(fontFace);
        
        // Cache font face
        this.fontCache.set(`${config.fontFamily}-${weight}`, fontFace);
        
        console.log(`Font loaded: ${config.fontFamily} ${weight}`);
      } catch (error) {
        console.warn(`Failed to load font: ${config.fontFamily} ${weight}`, error);
      }
    });

    await Promise.allSettled(promises);
  }

  /**
   * Generate optimized font URL
   */
  private generateFontUrl(fontFamily: string, weight: string, subset: string): string {
    const family = fontFamily.replace(/\s+/g, '+');
    return `https://fonts.googleapis.com/css2?family=${family}:wght@${weight}&subset=${subset}&display=swap`;
  }

  /**
   * Optimize font display strategy
   */
  private optimizeFontDisplay(config: FontLoadingConfig): void {
    // Add font-display CSS if not already present
    const styleId = `font-display-${config.fontFamily.replace(/\s+/g, '-')}`;
    
    if (!document.getElementById(styleId)) {
      const style = document.createElement('style');
      style.id = styleId;
      style.textContent = `
        @font-face {
          font-family: '${config.fontFamily}';
          font-display: ${config.display};
        }
      `;
      document.head.appendChild(style);
    }
  }

  /**
   * Cache font configuration
   */
  private cacheFontConfig(languageCode: string, config: FontLoadingConfig): void {
    const cacheKey = `font-config-${languageCode}`;
    localStorage.setItem(cacheKey, JSON.stringify(config));
  }

  /**
   * Cache translation data
   */
  cacheTranslation(key: string, data: any, customTtl?: number): void {
    const ttl = customTtl || this.cacheConfig.ttl;
    
    // Clean up expired entries
    this.cleanupCache();
    
    // Check cache size limit
    if (this.translationCache.size >= this.cacheConfig.maxSize) {
      // Remove oldest entry
      const oldestKey = this.translationCache.keys().next().value;
      this.translationCache.delete(oldestKey);
    }
    
    this.translationCache.set(key, {
      data,
      timestamp: Date.now(),
      ttl
    });
  }

  /**
   * Get cached translation
   */
  getCachedTranslation(key: string): any | null {
    const cached = this.translationCache.get(key);
    
    if (!cached) {
      return null;
    }
    
    // Check if expired
    if (Date.now() - cached.timestamp > cached.ttl) {
      this.translationCache.delete(key);
      return null;
    }
    
    return cached.data;
  }

  /**
   * Clean up expired cache entries
   */
  private cleanupCache(): void {
    const now = Date.now();
    
    for (const [key, value] of this.translationCache.entries()) {
      if (now - value.timestamp > value.ttl) {
        this.translationCache.delete(key);
      }
    }
  }

  /**
   * Implement lazy loading for translation content
   */
  enableLazyLoading(elements: NodeListOf<Element> | Element[]): void {
    if (!this.intersectionObserver) {
      console.warn('Intersection Observer not supported');
      return;
    }

    elements.forEach((element) => {
      // Add loading placeholder
      element.setAttribute('data-lazy-loading', 'true');
      element.innerHTML = '<div class="translation-loading">Loading translation...</div>';
      
      // Observe element
      this.intersectionObserver!.observe(element);
    });
  }

  /**
   * Load translation content for element
   */
  private async loadTranslationContent(element: HTMLElement): Promise<void> {
    const contentType = element.getAttribute('data-content-type');
    const contentId = element.getAttribute('data-content-id');
    const languageCode = element.getAttribute('data-language-code');
    
    if (!contentType || !contentId || !languageCode) {
      console.warn('Missing required attributes for lazy loading');
      return;
    }

    const cacheKey = `${contentType}-${contentId}-${languageCode}`;
    
    // Check cache first
    let translationData = this.getCachedTranslation(cacheKey);
    
    if (!translationData) {
      try {
        // Load from API
        const response = await fetch(
          `/api/v1/multilingual/content/${contentType}/${contentId}?language=${languageCode}`
        );
        translationData = await response.json();
        
        // Cache the result
        this.cacheTranslation(cacheKey, translationData);
      } catch (error) {
        console.error('Failed to load translation:', error);
        element.innerHTML = '<div class="translation-error">Failed to load translation</div>';
        return;
      }
    }

    // Update element content
    if (translationData.content) {
      element.innerHTML = translationData.content.content || translationData.content.title || '';
      element.removeAttribute('data-lazy-loading');
    }
  }

  /**
   * Optimize language switching performance
   */
  async optimizeLanguageSwitch(
    fromLanguage: string,
    toLanguage: string,
    contentElements: Element[]
  ): Promise<void> {
    const startTime = performance.now();
    performance.mark('language-switch-start');

    try {
      // Preload fonts for target language
      await this.optimizeFontLoading(toLanguage);
      
      // Batch update content elements
      await this.batchUpdateContent(contentElements, toLanguage);
      
      // Update document direction if needed
      this.updateDocumentDirection(toLanguage);
      
      // Update language-specific CSS
      this.updateLanguageCSS(fromLanguage, toLanguage);
      
      performance.mark('language-switch-end');
      performance.measure('language-switch', 'language-switch-start', 'language-switch-end');
      
      const endTime = performance.now();
      this.recordMetric('languageSwitchTime', endTime - startTime);
      
    } catch (error) {
      console.error('Language switch optimization failed:', error);
    }
  }

  /**
   * Batch update content elements
   */
  private async batchUpdateContent(elements: Element[], languageCode: string): Promise<void> {
    const batches = this.createBatches(Array.from(elements), this.lazyLoadingConfig.batchSize);
    
    for (const batch of batches) {
      const promises = batch.map(async (element) => {
        const htmlElement = element as HTMLElement;
        await this.loadTranslationContent(htmlElement);
      });
      
      await Promise.allSettled(promises);
      
      // Small delay between batches to prevent blocking
      await new Promise(resolve => setTimeout(resolve, 10));
    }
  }

  /**
   * Create batches from array
   */
  private createBatches<T>(array: T[], batchSize: number): T[][] {
    const batches: T[][] = [];
    
    for (let i = 0; i < array.length; i += batchSize) {
      batches.push(array.slice(i, i + batchSize));
    }
    
    return batches;
  }

  /**
   * Update document direction for RTL languages
   */
  private updateDocumentDirection(languageCode: string): void {
    const rtlLanguages = ['ur', 'ar', 'he', 'fa'];
    const isRTL = rtlLanguages.includes(languageCode);
    
    document.documentElement.dir = isRTL ? 'rtl' : 'ltr';
    document.documentElement.lang = languageCode;
  }

  /**
   * Update language-specific CSS classes
   */
  private updateLanguageCSS(fromLanguage: string, toLanguage: string): void {
    const body = document.body;
    
    // Remove old language class
    body.classList.remove(`lang-${fromLanguage}`);
    
    // Add new language class
    body.classList.add(`lang-${toLanguage}`);
    
    // Add RTL class if needed
    const rtlLanguages = ['ur', 'ar', 'he', 'fa'];
    if (rtlLanguages.includes(toLanguage)) {
      body.classList.add('rtl');
    } else {
      body.classList.remove('rtl');
    }
  }

  /**
   * Record performance metric
   */
  private recordMetric(type: keyof PerformanceMetrics, value: number): void {
    if (Array.isArray(this.performanceMetrics[type])) {
      const metrics = this.performanceMetrics[type] as number[];
      metrics.push(value);
      
      // Keep only last 100 measurements
      if (metrics.length > 100) {
        metrics.splice(0, metrics.length - 100);
      }
    }
  }

  /**
   * Get performance report
   */
  getPerformanceReport(): {
    averageTranslationLoadTime: number;
    averageFontLoadTime: number;
    averageLanguageSwitchTime: number;
    cacheHitRatio: number;
    cacheSize: number;
  } {
    const avgTranslationTime = this.calculateAverage(this.performanceMetrics.translationLoadTime);
    const avgFontTime = this.calculateAverage(this.performanceMetrics.fontLoadTime);
    const avgSwitchTime = this.calculateAverage(this.performanceMetrics.languageSwitchTime);
    
    return {
      averageTranslationLoadTime: avgTranslationTime,
      averageFontLoadTime: avgFontTime,
      averageLanguageSwitchTime: avgSwitchTime,
      cacheHitRatio: this.performanceMetrics.cacheHitRatio,
      cacheSize: this.translationCache.size
    };
  }

  /**
   * Calculate average of array
   */
  private calculateAverage(values: number[]): number {
    if (values.length === 0) return 0;
    return values.reduce((sum, value) => sum + value, 0) / values.length;
  }

  /**
   * Clear all caches
   */
  clearCaches(): void {
    this.translationCache.clear();
    this.fontCache.clear();
    
    // Clear localStorage font configs
    const keys = Object.keys(localStorage);
    keys.forEach(key => {
      if (key.startsWith('font-config-')) {
        localStorage.removeItem(key);
      }
    });
  }

  /**
   * Preload critical translations
   */
  async preloadCriticalTranslations(
    contentItems: Array<{
      contentType: string;
      contentId: string;
      languageCode: string;
    }>
  ): Promise<void> {
    const promises = contentItems.map(async (item) => {
      const cacheKey = `${item.contentType}-${item.contentId}-${item.languageCode}`;
      
      // Skip if already cached
      if (this.getCachedTranslation(cacheKey)) {
        return;
      }
      
      try {
        const response = await fetch(
          `/api/v1/multilingual/content/${item.contentType}/${item.contentId}?language=${item.languageCode}`
        );
        const data = await response.json();
        
        this.cacheTranslation(cacheKey, data);
      } catch (error) {
        console.warn(`Failed to preload translation: ${cacheKey}`, error);
      }
    });
    
    await Promise.allSettled(promises);
  }
}

// Export singleton instance
export const multilingualPerformanceOptimizer = new MultilingualPerformanceOptimizer();

// Export utility functions
export const optimizeFontLoading = (languageCode: string) => 
  multilingualPerformanceOptimizer.optimizeFontLoading(languageCode);

export const enableLazyLoading = (elements: NodeListOf<Element> | Element[]) =>
  multilingualPerformanceOptimizer.enableLazyLoading(elements);

export const optimizeLanguageSwitch = (
  fromLanguage: string,
  toLanguage: string,
  contentElements: Element[]
) => multilingualPerformanceOptimizer.optimizeLanguageSwitch(fromLanguage, toLanguage, contentElements);

export const getPerformanceReport = () =>
  multilingualPerformanceOptimizer.getPerformanceReport();