/**
 * API service for community translation contributions
 */

import axios from 'axios';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export interface TranslationContribution {
  content_type: string;
  content_id: string;
  language_code: string;
  title?: string;
  content?: string;
  notes?: string;
}

export interface ReviewSubmission {
  rating: number;
  feedback: string;
  suggested_improvements?: Record<string, string>;
}

export interface CorrectionSubmission {
  corrections: Record<string, string>;
  explanation: string;
}

export interface TerminologySubmission {
  term: string;
  translation: string;
  source_language: string;
  target_language: string;
  definition?: string;
  context?: string;
  domain: string;
}

export interface ContributionResponse {
  success: boolean;
  contribution_id?: number;
  status?: string;
  message?: string;
  error?: string;
}

export interface UserContribution {
  id: number;
  type: string;
  content_reference?: string;
  language?: string;
  title?: string;
  status: string;
  created_at: string;
  updated_at: string;
  votes?: {
    upvotes: number;
    downvotes: number;
    vote_score: number;
  };
  reviews?: number;
  average_rating?: number;
}

export interface UserStats {
  reputation: number;
  level: string;
  translation_count: number;
  approved_translations: number;
  terminology_contributions: number;
  approval_rate: number;
  privileges: string[];
}

export interface LeaderboardEntry {
  user_id: number;
  username: string;
  full_name?: string;
  reputation: number;
  level: string;
  translation_count: number;
  period_contributions: number;
}

export interface Leaderboard {
  leaderboard: LeaderboardEntry[];
  period: string;
  total_contributors: number;
}

export interface ContributionsResponse {
  contributions: UserContribution[];
  total_count: number;
  user_stats?: UserStats;
}

class CommunityTranslationAPI {
  private baseURL: string;

  constructor() {
    this.baseURL = `${API_BASE_URL}/multilingual/community`;
  }

  /**
   * Submit a translation contribution
   */
  async submitTranslation(
    contribution: TranslationContribution,
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/translations`,
        contribution,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error submitting translation:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to submit translation'
      };
    }
  }

  /**
   * Submit a review for a translation
   */
  async submitReview(
    translationId: number,
    review: ReviewSubmission,
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/translations/${translationId}/reviews`,
        review,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error submitting review:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to submit review'
      };
    }
  }

  /**
   * Submit corrections for a translation
   */
  async submitCorrection(
    translationId: number,
    correction: CorrectionSubmission,
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/translations/${translationId}/corrections`,
        correction,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error submitting correction:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to submit correction'
      };
    }
  }

  /**
   * Submit terminology contribution
   */
  async submitTerminology(
    terminology: TerminologySubmission,
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/terminology`,
        terminology,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error submitting terminology:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to submit terminology'
      };
    }
  }

  /**
   * Vote on a contribution
   */
  async voteOnContribution(
    contributionId: number,
    contributionType: string,
    vote: 'upvote' | 'downvote',
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/contributions/${contributionId}/vote`,
        {
          contribution_type: contributionType,
          vote: vote
        },
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error voting on contribution:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to vote'
      };
    }
  }

  /**
   * Get user's contributions
   */
  async getUserContributions(
    userId: number,
    contributionType?: string,
    limit: number = 50,
    offset: number = 0,
    token: string
  ): Promise<ContributionsResponse> {
    try {
      const params: any = { limit, offset };
      if (contributionType) {
        params.contribution_type = contributionType;
      }

      const response = await axios.get(
        `${this.baseURL}/users/${userId}/contributions`,
        {
          params,
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching user contributions:', error);
      return {
        contributions: [],
        total_count: 0
      };
    }
  }

  /**
   * Get contributions for specific content
   */
  async getContentContributions(
    contentType: string,
    contentId: string,
    languageCode: string,
    token: string
  ): Promise<ContributionsResponse> {
    try {
      const response = await axios.get(
        `${this.baseURL}/content/${contentType}/${contentId}/contributions`,
        {
          params: { language_code: languageCode },
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching content contributions:', error);
      return {
        contributions: [],
        total_count: 0
      };
    }
  }

  /**
   * Get user statistics
   */
  async getUserStats(userId: number, token: string): Promise<UserStats> {
    try {
      const response = await axios.get(
        `${this.baseURL}/users/${userId}/stats`,
        {
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching user stats:', error);
      return {
        reputation: 0,
        level: 'Beginner',
        translation_count: 0,
        approved_translations: 0,
        terminology_contributions: 0,
        approval_rate: 0,
        privileges: []
      };
    }
  }

  /**
   * Get community leaderboard
   */
  async getLeaderboard(
    period: string = 'all_time',
    limit: number = 20,
    token?: string
  ): Promise<Leaderboard> {
    try {
      const headers: any = {};
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }

      const response = await axios.get(
        `${this.baseURL}/leaderboard`,
        {
          params: { period, limit },
          headers
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching leaderboard:', error);
      return {
        leaderboard: [],
        period,
        total_contributors: 0
      };
    }
  }

  /**
   * Get translation quality metrics
   */
  async getQualityMetrics(
    contentType?: string,
    languageCode?: string,
    token?: string
  ): Promise<Record<string, any>> {
    try {
      const params: any = {};
      if (contentType) params.content_type = contentType;
      if (languageCode) params.language_code = languageCode;

      const headers: any = {};
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }

      const response = await axios.get(
        `${this.baseURL}/quality-metrics`,
        {
          params,
          headers
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching quality metrics:', error);
      return {};
    }
  }

  /**
   * Get contribution guidelines
   */
  async getContributionGuidelines(
    languageCode: string,
    token?: string
  ): Promise<Record<string, any>> {
    try {
      const headers: any = {};
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }

      const response = await axios.get(
        `${this.baseURL}/guidelines/${languageCode}`,
        { headers }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error fetching contribution guidelines:', error);
      return {};
    }
  }

  /**
   * Report inappropriate content
   */
  async reportContent(
    contributionId: number,
    contributionType: string,
    reason: string,
    description: string,
    token: string
  ): Promise<ContributionResponse> {
    try {
      const response = await axios.post(
        `${this.baseURL}/contributions/${contributionId}/report`,
        {
          contribution_type: contributionType,
          reason,
          description
        },
        {
          headers: {
            Authorization: `Bearer ${token}`,
            'Content-Type': 'application/json'
          }
        }
      );
      return response.data;
    } catch (error: any) {
      console.error('Error reporting content:', error);
      return {
        success: false,
        error: error.response?.data?.detail || 'Failed to report content'
      };
    }
  }

  /**
   * Get user's reputation history
   */
  async getReputationHistory(
    userId: number,
    limit: number = 50,
    token: string
  ): Promise<any[]> {
    try {
      const response = await axios.get(
        `${this.baseURL}/users/${userId}/reputation-history`,
        {
          params: { limit },
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );
      return response.data.history || [];
    } catch (error: any) {
      console.error('Error fetching reputation history:', error);
      return [];
    }
  }

  /**
   * Get available translation tasks
   */
  async getAvailableTasks(
    languageCode?: string,
    priority?: string,
    limit: number = 20,
    token?: string
  ): Promise<any[]> {
    try {
      const params: any = { limit };
      if (languageCode) params.language_code = languageCode;
      if (priority) params.priority = priority;

      const headers: any = {};
      if (token) {
        headers.Authorization = `Bearer ${token}`;
      }

      const response = await axios.get(
        `${this.baseURL}/tasks`,
        {
          params,
          headers
        }
      );
      return response.data.tasks || [];
    } catch (error: any) {
      console.error('Error fetching available tasks:', error);
      return [];
    }
  }
}

export const communityTranslationAPI = new CommunityTranslationAPI();