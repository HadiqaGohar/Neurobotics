/**
 * API service for multilingual content support
 */

import axios from 'axios';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export interface Language {
  code: string;
  name: string;
  native_name: string;
  direction: 'ltr' | 'rtl';
  is_default: boolean;
}

export interface UserLanguagePreferences {
  preferred_language: string;
  secondary_language?: string;
  language_preferences?: Record<string, any>;
}

export interface TranslatedContent {
  id: number;
  title?: string;
  content?: string;
  metadata?: Record<string, any>;
  quality_score?: number;
  version: number;
  updated_at: string;
}

export interface ContentResponse {
  content: TranslatedContent;
  language: string;
  fallback_used: boolean;
  requested_language?: string;
}

export interface TerminologyTranslation {
  term: string;
  translation: string;
  definition?: string;
  context?: string;
  domain: string;
  source_language: string;
  target_language: string;
}

export interface LanguageDetectionResult {
  detected_language: string;
  confidence: 'high' | 'medium' | 'low';
}

export interface TranslationMemoryResult {
  found: boolean;
  source_text: string;
  target_text?: string;
  quality_score?: number;
  usage_count?: number;
  match_type?: string;
  message?: string;
}

class MultilingualAPI {
  private baseURL: string;

  constructor() {
    this.baseURL = `${API_BASE_URL}/multilingual`;
  }

  /**
   * Get all supported languages
   */
  async getSupportedLanguages(): Promise<Language[]> {
    try {
      const response = await axios.get(`${this.baseURL}/languages`);
      return response.data;
    } catch (error) {
      console.error('Error fetching supported languages:', error);
      throw error;
    }
  }

  /**
   * Detect user's preferred language
   */
  async detectUserLanguage(): Promise<LanguageDetectionResult> {
    try {
      const response = await axios.post(`${this.baseURL}/detect-language`);
      return response.data;
    } catch (error) {
      console.error('Error detecting user language:', error);
      throw error;
    }
  }

  /**
   * Get user's language preferences
   */
  async getUserLanguagePreferences(token: string): Promise<UserLanguagePreferences> {
    try {
      const response = await axios.get(`${this.baseURL}/user/preferences`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      return response.data;
    } catch (error) {
      console.error('Error fetching user language preferences:', error);
      throw error;
    }
  }

  /**
   * Update user's language preferences
   */
  async updateUserLanguagePreferences(
    token: string, 
    preferences: Partial<UserLanguagePreferences>
  ): Promise<{ message: string }> {
    try {
      const response = await axios.put(
        `${this.baseURL}/user/preferences`,
        preferences,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error) {
      console.error('Error updating user language preferences:', error);
      throw error;
    }
  }

  /**
   * Get translated content for specific language
   */
  async getTranslatedContent(
    contentType: string,
    contentId: string,
    language: string = 'en',
    fallback: boolean = true
  ): Promise<ContentResponse> {
    try {
      const response = await axios.get(
        `${this.baseURL}/content/${contentType}/${contentId}`,
        {
          params: { language, fallback }
        }
      );
      return response.data;
    } catch (error) {
      console.error('Error fetching translated content:', error);
      throw error;
    }
  }

  /**
   * Get terminology translation from glossary
   */
  async getTerminologyTranslation(
    term: string,
    sourceLanguage: string = 'en',
    targetLanguage: string = 'ur',
    domain?: string
  ): Promise<TerminologyTranslation> {
    try {
      const params: any = {
        source_language: sourceLanguage,
        target_language: targetLanguage
      };
      
      if (domain) {
        params.domain = domain;
      }

      const response = await axios.get(
        `${this.baseURL}/terminology/${encodeURIComponent(term)}`,
        { params }
      );
      return response.data;
    } catch (error) {
      console.error('Error fetching terminology translation:', error);
      throw error;
    }
  }

  /**
   * Search translation memory for similar translations
   */
  async searchTranslationMemory(
    sourceText: string,
    sourceLanguage: string,
    targetLanguage: string,
    similarityThreshold: number = 0.8
  ): Promise<TranslationMemoryResult> {
    try {
      const response = await axios.get(
        `${this.baseURL}/translation-memory/search`,
        {
          params: {
            source_text: sourceText,
            source_language: sourceLanguage,
            target_language: targetLanguage,
            similarity_threshold: similarityThreshold
          }
        }
      );
      return response.data;
    } catch (error) {
      console.error('Error searching translation memory:', error);
      throw error;
    }
  }

  /**
   * Add translation to translation memory
   */
  async addTranslationMemory(
    token: string,
    sourceText: string,
    targetText: string,
    sourceLanguage: string,
    targetLanguage: string,
    context?: string,
    domain?: string,
    qualityScore?: number
  ): Promise<{ message: string }> {
    try {
      const response = await axios.post(
        `${this.baseURL}/translation-memory`,
        {
          source_text: sourceText,
          target_text: targetText,
          source_language: sourceLanguage,
          target_language: targetLanguage,
          context,
          domain,
          quality_score: qualityScore
        },
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error) {
      console.error('Error adding translation memory:', error);
      throw error;
    }
  }

  /**
   * Get language usage and translation statistics
   */
  async getLanguageStatistics(token: string): Promise<Record<string, any>> {
    try {
      const response = await axios.get(`${this.baseURL}/statistics`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      return response.data;
    } catch (error) {
      console.error('Error fetching language statistics:', error);
      throw error;
    }
  }
}

export const multilingualAPI = new MultilingualAPI();