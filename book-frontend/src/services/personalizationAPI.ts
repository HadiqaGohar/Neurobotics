/**
 * API service for personalization features
 */

import { 
  SoftwareBackground, 
  HardwareBackground, 
  UserPreferences 
} from '../types/personalization';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8001/api/v1';

export interface PersonalizationAPIResponse {
  message: string;
}

export interface SaveSignupPreferencesRequest {
  user_id: number;
  software_background?: SoftwareBackground;
  hardware_background?: HardwareBackground;
}

class PersonalizationAPI {
  private baseURL: string;

  constructor(baseURL: string = API_BASE_URL) {
    this.baseURL = baseURL;
  }

  /**
   * Save user preferences during signup (no auth required)
   */
  async saveSignupPreferences(
    userId: number,
    softwareBackground?: SoftwareBackground,
    hardwareBackground?: HardwareBackground
  ): Promise<PersonalizationAPIResponse> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/preferences/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: userId,
          software_background: softwareBackground,
          hardware_background: hardwareBackground,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error saving signup preferences:', error);
      throw error;
    }
  }

  /**
   * Get user preferences (requires auth)
   */
  async getUserPreferences(token: string): Promise<UserPreferences> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/preferences`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting user preferences:', error);
      throw error;
    }
  }

  /**
   * Update user preferences (requires auth)
   */
  async updateUserPreferences(
    token: string,
    updates: {
      softwareBackground?: SoftwareBackground;
      hardwareBackground?: HardwareBackground;
      contentComplexity?: string;
      explanationDepth?: string;
      exampleStyle?: string;
    }
  ): Promise<PersonalizationAPIResponse> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/preferences`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          software_background: updates.softwareBackground,
          hardware_background: updates.hardwareBackground,
          content_complexity: updates.contentComplexity,
          explanation_depth: updates.explanationDepth,
          example_style: updates.exampleStyle,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error updating user preferences:', error);
      throw error;
    }
  }

  /**
   * Get personalized content for a chapter (requires auth)
   */
  async getPersonalizedContent(
    token: string,
    chapterId: string,
    sectionId?: string,
    forceRegenerate: boolean = false
  ): Promise<any> {
    try {
      const params = new URLSearchParams();
      if (sectionId) params.append('section_id', sectionId);
      if (forceRegenerate) params.append('force_regenerate', 'true');

      const url = `${this.baseURL}/personalization/content/${chapterId}${
        params.toString() ? `?${params.toString()}` : ''
      }`;

      const response = await fetch(url, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting personalized content:', error);
      throw error;
    }
  }

  /**
   * Get available software categories
   */
  async getSoftwareCategories(): Promise<{ categories: string[] }> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/categories/software`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting software categories:', error);
      throw error;
    }
  }

  /**
   * Get available hardware categories
   */
  async getHardwareCategories(): Promise<{ categories: string[] }> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/categories/hardware`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting hardware categories:', error);
      throw error;
    }
  }

  /**
   * Get supported programming languages
   */
  async getSupportedLanguages(): Promise<{ languages: string[] }> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/languages`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting supported languages:', error);
      throw error;
    }
  }

  /**
   * Get supported hardware platforms
   */
  async getSupportedPlatforms(): Promise<{ platforms: string[] }> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/platforms`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting supported platforms:', error);
      throw error;
    }
  }

  /**
   * Health check for personalization service
   */
  async healthCheck(): Promise<{ status: string; service: string; timestamp: string }> {
    try {
      const response = await fetch(`${this.baseURL}/personalization/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error checking personalization service health:', error);
      throw error;
    }
  }
}

// Export singleton instance
export const personalizationAPI = new PersonalizationAPI();
export default personalizationAPI;