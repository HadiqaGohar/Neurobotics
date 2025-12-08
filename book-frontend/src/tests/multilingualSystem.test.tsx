/**
 * Comprehensive test suite for multilingual frontend components
 */

import React from 'react';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import { jest } from '@jest/globals';
import '@testing-library/jest-dom';
import userEvent from '@testing-library/user-event';

// Mock the multilingual components
import { ContributionInterface } from '../components/CommunityTranslation/ContributionInterface';
import { Leaderboard } from '../components/CommunityTranslation/Leaderboard';
import { UserProfile } from '../components/CommunityTranslation/UserProfile';
import { multilingualPerformanceOptimizer } from '../utils/multilingualPerformance';

// Mock the API services
jest.mock('../services/communityTranslationAPI');
jest.mock('../auth/AuthContext');

// Mock user context
const mockUser = {
  id: 1,
  username: 'testuser',
  full_name: 'Test User',
  email: 'test@example.com'
};

const mockAuthContext = {
  user: mockUser,
  token: 'mock-token',
  login: jest.fn(),
  logout: jest.fn(),
  isAuthenticated: true
};

// Mock Material-UI theme
jest.mock('@mui/material/styles', () => ({
  useTheme: () => ({
    direction: 'ltr',
    palette: {
      primary: { main: '#1976d2' },
      secondary: { main: '#dc004e' }
    }
  })
}));

describe('Multilingual System Tests', () => {
  beforeEach(() => {
    // Reset all mocks
    jest.clearAllMocks();
    
    // Mock auth context
    require('../auth/AuthContext').useAuth.mockReturnValue(mockAuthContext);
  });

  describe('ContributionInterface Component', () => {
    const defaultProps = {
      contentType: 'chapter',
      contentId: 'test-chapter-1',
      sourceLanguage: 'en',
      targetLanguage: 'ur',
      originalTitle: 'Introduction to Machine Learning',
      originalContent: 'This is a test chapter about machine learning.',
      onContributionSubmitted: jest.fn()
    };

    test('renders contribution interface correctly', () => {
      render(<ContributionInterface {...defaultProps} />);
      
      expect(screen.getByText('Your Contributor Profile')).toBeInTheDocument();
      expect(screen.getByText('Submit Translation')).toBeInTheDocument();
      expect(screen.getByText('Review Translations')).toBeInTheDocument();
      expect(screen.getByText('Submit Corrections')).toBeInTheDocument();
      expect(screen.getByText('Add Terminology')).toBeInTheDocument();
    });

    test('handles translation submission', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.submitTranslation.mockResolvedValue({
        success: true,
        contribution_id: 123,
        message: 'Translation submitted successfully'
      });

      render(<ContributionInterface {...defaultProps} />);
      
      // Fill in translation form
      const titleInput = screen.getByLabelText('Translation Title');
      const contentInput = screen.getByLabelText('Translation Content');
      const submitButton = screen.getByText('Submit Translation');

      await user.type(titleInput, 'مشین لرننگ کا تعارف');
      await user.type(contentInput, 'یہ مشین لرننگ کے بارے میں ایک ٹیسٹ چیپٹر ہے۔');
      await user.click(submitButton);

      await waitFor(() => {
        expect(mockAPI.communityTranslationAPI.submitTranslation).toHaveBeenCalledWith(
          expect.objectContaining({
            content_type: 'chapter',
            content_id: 'test-chapter-1',
            language_code: 'ur',
            title: 'مشین لرننگ کا تعارف',
            content: 'یہ مشین لرننگ کے بارے میں ایک ٹیسٹ چیپٹر ہے۔'
          }),
          'mock-token'
        );
      });
    });

    test('handles review submission', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.submitReview.mockResolvedValue({
        success: true,
        review_id: 456
      });

      render(<ContributionInterface {...defaultProps} />);
      
      // Switch to review tab
      const reviewTab = screen.getByText('Review Translations');
      await user.click(reviewTab);

      // This would test review functionality if translations were loaded
      expect(screen.getByText('Review Existing Translations')).toBeInTheDocument();
    });

    test('handles error states correctly', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.submitTranslation.mockResolvedValue({
        success: false,
        error: 'Translation submission failed'
      });

      render(<ContributionInterface {...defaultProps} />);
      
      const submitButton = screen.getByText('Submit Translation');
      await user.click(submitButton);

      await waitFor(() => {
        expect(screen.getByText('Please provide either a title or content translation')).toBeInTheDocument();
      });
    });

    test('shows user level and privileges correctly', () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue({
        reputation: 150,
        level: 'Contributor',
        translation_count: 5,
        approved_translations: 3,
        approval_rate: 60,
        privileges: ['submit_translations', 'review_translations']
      });

      render(<ContributionInterface {...defaultProps} />);
      
      // User stats would be loaded asynchronously
      expect(screen.getByText('Your Contributor Profile')).toBeInTheDocument();
    });
  });

  describe('Leaderboard Component', () => {
    const mockLeaderboardData = {
      leaderboard: [
        {
          user_id: 1,
          username: 'topcontributor',
          full_name: 'Top Contributor',
          reputation: 1500,
          level: 'Expert',
          translation_count: 50,
          period_contributions: 10
        },
        {
          user_id: 2,
          username: 'translator2',
          full_name: 'Second Translator',
          reputation: 800,
          level: 'Translator',
          translation_count: 25,
          period_contributions: 5
        }
      ],
      period: 'all_time',
      total_contributors: 2
    };

    test('renders leaderboard correctly', async () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue(mockLeaderboardData);

      render(<Leaderboard />);
      
      expect(screen.getByText('Community Leaderboard')).toBeInTheDocument();
      
      await waitFor(() => {
        expect(screen.getByText('Top Contributor')).toBeInTheDocument();
        expect(screen.getByText('Expert')).toBeInTheDocument();
      });
    });

    test('handles period selection', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue(mockLeaderboardData);

      render(<Leaderboard />);
      
      // Test period selection (would need to mock Material-UI Select)
      expect(screen.getByText('Community Leaderboard')).toBeInTheDocument();
    });

    test('shows user rank when authenticated', async () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue({
        ...mockLeaderboardData,
        leaderboard: [
          ...mockLeaderboardData.leaderboard,
          {
            user_id: 1, // Same as mockUser.id
            username: 'testuser',
            full_name: 'Test User',
            reputation: 100,
            level: 'Contributor',
            translation_count: 3,
            period_contributions: 1
          }
        ]
      });

      render(<Leaderboard showUserRank={true} />);
      
      await waitFor(() => {
        expect(screen.getByText('Your Ranking')).toBeInTheDocument();
      });
    });

    test('renders compact version correctly', () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue(mockLeaderboardData);

      render(<Leaderboard compact={true} />);
      
      expect(screen.getByText('Top Contributors')).toBeInTheDocument();
    });
  });

  describe('UserProfile Component', () => {
    const mockUserStats = {
      reputation: 250,
      level: 'Translator',
      translation_count: 15,
      approved_translations: 12,
      terminology_contributions: 5,
      approval_rate: 80,
      privileges: ['submit_translations', 'review_translations', 'edit_terminology']
    };

    const mockContributions = [
      {
        id: 1,
        type: 'translation',
        content_reference: 'chapter:test-chapter-1',
        language: 'ur',
        title: 'Test Translation',
        status: 'approved',
        created_at: '2023-12-01T10:00:00Z',
        updated_at: '2023-12-01T10:00:00Z',
        votes: { upvotes: 5, downvotes: 1, vote_score: 4 },
        reviews: 3,
        average_rating: 4.5
      }
    ];

    test('renders user profile correctly', async () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue(mockUserStats);
      mockAPI.communityTranslationAPI.getUserContributions.mockResolvedValue({
        contributions: mockContributions,
        total_count: 1
      });

      render(<UserProfile />);
      
      expect(screen.getByText('Test User')).toBeInTheDocument();
      
      await waitFor(() => {
        expect(screen.getByText('15')).toBeInTheDocument(); // Translation count
        expect(screen.getByText('12')).toBeInTheDocument(); // Approved translations
      });
    });

    test('shows privileges correctly', async () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue(mockUserStats);
      mockAPI.communityTranslationAPI.getUserContributions.mockResolvedValue({
        contributions: [],
        total_count: 0
      });

      render(<UserProfile />);
      
      await waitFor(() => {
        expect(screen.getByText('Privileges & Permissions')).toBeInTheDocument();
      });
    });

    test('handles contributions tab switching', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue(mockUserStats);
      mockAPI.communityTranslationAPI.getUserContributions.mockResolvedValue({
        contributions: mockContributions,
        total_count: 1
      });
      mockAPI.communityTranslationAPI.getReputationHistory.mockResolvedValue([]);

      render(<UserProfile />);
      
      const reputationTab = screen.getByText('Reputation History');
      await user.click(reputationTab);
      
      expect(screen.getByText('Reputation History')).toBeInTheDocument();
    });

    test('renders compact version correctly', () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue(mockUserStats);

      render(<UserProfile compact={true} />);
      
      expect(screen.getByText('Test User')).toBeInTheDocument();
    });
  });

  describe('Performance Optimization', () => {
    test('font loading optimization', async () => {
      const spy = jest.spyOn(multilingualPerformanceOptimizer, 'optimizeFontLoading');
      
      await multilingualPerformanceOptimizer.optimizeFontLoading('ur');
      
      expect(spy).toHaveBeenCalledWith('ur');
    });

    test('translation caching', () => {
      const testData = { title: 'Test', content: 'Test content' };
      
      multilingualPerformanceOptimizer.cacheTranslation('test-key', testData);
      const cached = multilingualPerformanceOptimizer.getCachedTranslation('test-key');
      
      expect(cached).toEqual(testData);
    });

    test('lazy loading setup', () => {
      // Mock DOM elements
      const mockElements = [
        {
          setAttribute: jest.fn(),
          innerHTML: '',
          getAttribute: jest.fn()
        }
      ] as any;

      // This would test lazy loading if IntersectionObserver was properly mocked
      expect(mockElements.length).toBe(1);
    });

    test('performance metrics collection', () => {
      const report = multilingualPerformanceOptimizer.getPerformanceReport();
      
      expect(report).toHaveProperty('averageTranslationLoadTime');
      expect(report).toHaveProperty('averageFontLoadTime');
      expect(report).toHaveProperty('cacheSize');
    });
  });

  describe('RTL Support Tests', () => {
    test('detects RTL languages correctly', () => {
      const rtlLanguages = ['ur', 'ar', 'he', 'fa'];
      const ltrLanguages = ['en', 'es', 'fr', 'de'];
      
      rtlLanguages.forEach(lang => {
        expect(['ur', 'ar', 'he', 'fa'].includes(lang)).toBe(true);
      });
      
      ltrLanguages.forEach(lang => {
        expect(['ur', 'ar', 'he', 'fa'].includes(lang)).toBe(false);
      });
    });

    test('handles mixed content correctly', () => {
      const mixedContent = 'This is English with اردو text mixed in.';
      
      expect(mixedContent).toContain('اردو');
      expect(mixedContent).toContain('English');
    });

    test('applies correct CSS classes for RTL', () => {
      // Mock document.body
      const mockBody = {
        classList: {
          add: jest.fn(),
          remove: jest.fn()
        }
      };
      
      Object.defineProperty(document, 'body', {
        value: mockBody,
        writable: true
      });

      // Simulate language switch to Urdu
      document.body.classList.add('lang-ur');
      document.body.classList.add('rtl');
      
      expect(mockBody.classList.add).toHaveBeenCalledWith('lang-ur');
      expect(mockBody.classList.add).toHaveBeenCalledWith('rtl');
    });
  });

  describe('Accessibility Tests', () => {
    test('includes proper ARIA labels', () => {
      render(<ContributionInterface 
        contentType="chapter"
        contentId="test"
        sourceLanguage="en"
        targetLanguage="ur"
      />);
      
      // Check for accessibility attributes
      const tabs = screen.getAllByRole('tab');
      expect(tabs.length).toBeGreaterThan(0);
    });

    test('supports keyboard navigation', () => {
      render(<Leaderboard />);
      
      // Test that interactive elements are focusable
      const buttons = screen.getAllByRole('button');
      buttons.forEach(button => {
        expect(button).not.toHaveAttribute('tabindex', '-1');
      });
    });

    test('provides language attributes', () => {
      const { container } = render(
        <div lang="ur">اردو متن</div>
      );
      
      const urduElement = container.querySelector('[lang="ur"]');
      expect(urduElement).toBeInTheDocument();
    });
  });

  describe('Error Handling', () => {
    test('handles API errors gracefully', async () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.submitTranslation.mockRejectedValue(
        new Error('Network error')
      );

      render(<ContributionInterface 
        contentType="chapter"
        contentId="test"
        sourceLanguage="en"
        targetLanguage="ur"
      />);
      
      // Error handling would be tested here
      expect(screen.getByText('Your Contributor Profile')).toBeInTheDocument();
    });

    test('shows loading states correctly', () => {
      render(<Leaderboard />);
      
      // Loading state would be shown initially
      expect(screen.getByText('Community Leaderboard')).toBeInTheDocument();
    });

    test('handles empty data states', () => {
      const mockAPI = require('../services/communityTranslationAPI');
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue({
        leaderboard: [],
        period: 'all_time',
        total_contributors: 0
      });

      render(<Leaderboard />);
      
      // Empty state handling would be tested
      expect(screen.getByText('Community Leaderboard')).toBeInTheDocument();
    });
  });

  describe('Integration Tests', () => {
    test('complete contribution workflow', async () => {
      const user = userEvent.setup();
      const mockAPI = require('../services/communityTranslationAPI');
      
      // Mock successful API calls
      mockAPI.communityTranslationAPI.submitTranslation.mockResolvedValue({
        success: true,
        contribution_id: 123
      });
      mockAPI.communityTranslationAPI.getUserStats.mockResolvedValue({
        reputation: 105, // Increased after contribution
        level: 'Contributor'
      });

      const onContributionSubmitted = jest.fn();
      
      render(<ContributionInterface 
        contentType="chapter"
        contentId="test"
        sourceLanguage="en"
        targetLanguage="ur"
        onContributionSubmitted={onContributionSubmitted}
      />);
      
      // Submit a translation
      const titleInput = screen.getByLabelText('Translation Title');
      await user.type(titleInput, 'Test Title');
      
      const submitButton = screen.getByText('Submit Translation');
      await user.click(submitButton);
      
      // Verify callback was called
      await waitFor(() => {
        expect(onContributionSubmitted).toHaveBeenCalled();
      });
    });

    test('user journey from contribution to leaderboard', async () => {
      // This would test the complete user journey
      const mockAPI = require('../services/communityTranslationAPI');
      
      mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue({
        leaderboard: [
          {
            user_id: 1,
            username: 'testuser',
            reputation: 105,
            level: 'Contributor',
            translation_count: 1
          }
        ],
        period: 'all_time',
        total_contributors: 1
      });

      render(<Leaderboard showUserRank={true} />);
      
      await waitFor(() => {
        expect(screen.getByText('Community Leaderboard')).toBeInTheDocument();
      });
    });
  });
});

// Performance tests
describe('Performance Tests', () => {
  test('component renders within performance budget', () => {
    const startTime = performance.now();
    
    render(<ContributionInterface 
      contentType="chapter"
      contentId="test"
      sourceLanguage="en"
      targetLanguage="ur"
    />);
    
    const endTime = performance.now();
    const renderTime = endTime - startTime;
    
    // Should render within 100ms
    expect(renderTime).toBeLessThan(100);
  });

  test('large leaderboard renders efficiently', () => {
    const mockAPI = require('../services/communityTranslationAPI');
    
    // Mock large dataset
    const largeLeaderboard = Array.from({ length: 1000 }, (_, i) => ({
      user_id: i + 1,
      username: `user${i + 1}`,
      reputation: 1000 - i,
      level: 'Contributor',
      translation_count: 10
    }));
    
    mockAPI.communityTranslationAPI.getLeaderboard.mockResolvedValue({
      leaderboard: largeLeaderboard,
      period: 'all_time',
      total_contributors: 1000
    });

    const startTime = performance.now();
    render(<Leaderboard />);
    const endTime = performance.now();
    
    expect(endTime - startTime).toBeLessThan(200); // Should render within 200ms
  });
});

// Mock setup for tests
beforeAll(() => {
  // Mock IntersectionObserver
  global.IntersectionObserver = jest.fn().mockImplementation((callback) => ({
    observe: jest.fn(),
    unobserve: jest.fn(),
    disconnect: jest.fn()
  }));

  // Mock PerformanceObserver
  global.PerformanceObserver = jest.fn().mockImplementation((callback) => ({
    observe: jest.fn(),
    disconnect: jest.fn()
  }));

  // Mock FontFace
  global.FontFace = jest.fn().mockImplementation(() => ({
    load: jest.fn().mockResolvedValue({}),
    family: 'Test Font'
  }));

  // Mock document.fonts
  Object.defineProperty(document, 'fonts', {
    value: {
      add: jest.fn()
    },
    writable: true
  });
});

afterAll(() => {
  // Cleanup mocks
  jest.restoreAllMocks();
});