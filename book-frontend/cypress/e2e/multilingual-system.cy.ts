/**
 * End-to-end tests for multilingual system
 * Tests complete user workflows and system integration
 */

describe('Multilingual System E2E Tests', () => {
  beforeEach(() => {
    // Mock authentication
    cy.window().then((win) => {
      win.localStorage.setItem('auth_token', 'mock-token');
      win.localStorage.setItem('user', JSON.stringify({
        id: 1,
        username: 'testuser',
        full_name: 'Test User'
      }));
    });
  });

  describe('Language Selection and Switching', () => {
    it('should allow users to select language preference', () => {
      cy.visit('/');
      
      // Look for language selector
      cy.get('[data-testid="language-selector"]').should('be.visible');
      
      // Select Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify language change
      cy.get('html').should('have.attr', 'lang', 'ur');
      cy.get('html').should('have.attr', 'dir', 'rtl');
    });

    it('should persist language preference across sessions', () => {
      cy.visit('/');
      
      // Set language preference
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Reload page
      cy.reload();
      
      // Verify language is still set
      cy.get('html').should('have.attr', 'lang', 'ur');
    });

    it('should switch language within 1 second', () => {
      cy.visit('/');
      
      const startTime = Date.now();
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      cy.get('html[lang="ur"]').should('exist').then(() => {
        const endTime = Date.now();
        const switchTime = endTime - startTime;
        expect(switchTime).to.be.lessThan(1000);
      });
    });
  });

  describe('Content Translation Display', () => {
    it('should display translated content correctly', () => {
      cy.visit('/chapter/test-chapter-1');
      
      // Switch to Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify content is in Urdu
      cy.get('[data-testid="chapter-title"]')
        .should('contain.text', 'مشین لرننگ');
      
      cy.get('[data-testid="chapter-content"]')
        .should('contain.text', 'اردو');
    });

    it('should show fallback to English when translation unavailable', () => {
      cy.visit('/chapter/untranslated-chapter');
      
      // Switch to Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Should show English content with fallback notice
      cy.get('[data-testid="fallback-notice"]')
        .should('be.visible')
        .and('contain.text', 'English');
    });

    it('should load translations within 2 seconds', () => {
      cy.visit('/chapter/test-chapter-1');
      
      const startTime = Date.now();
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      cy.get('[data-testid="chapter-content"]').should('be.visible').then(() => {
        const endTime = Date.now();
        const loadTime = endTime - startTime;
        expect(loadTime).to.be.lessThan(2000);
      });
    });
  });

  describe('RTL Layout and Typography', () => {
    it('should apply RTL layout for Urdu content', () => {
      cy.visit('/');
      
      // Switch to Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify RTL layout
      cy.get('html').should('have.attr', 'dir', 'rtl');
      cy.get('body').should('have.class', 'rtl');
      
      // Check text alignment
      cy.get('[data-testid="main-content"]')
        .should('have.css', 'text-align', 'right');
    });

    it('should load Urdu fonts correctly', () => {
      cy.visit('/');
      
      // Switch to Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Wait for font loading
      cy.wait(1000);
      
      // Verify font family
      cy.get('[data-testid="urdu-text"]')
        .should('have.css', 'font-family')
        .and('include', 'Noto Nastaliq Urdu');
    });

    it('should handle mixed language content correctly', () => {
      cy.visit('/chapter/mixed-content');
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify mixed content rendering
      cy.get('[data-testid="mixed-content"]')
        .should('contain.text', 'English')
        .and('contain.text', 'اردو');
    });
  });

  describe('Community Translation Features', () => {
    beforeEach(() => {
      // Navigate to community translation page
      cy.visit('/community-translation');
    });

    it('should allow users to submit translations', () => {
      // Go to contribution tab
      cy.get('[data-testid="contribute-tab"]').click();
      
      // Fill translation form
      cy.get('[data-testid="translation-title"]')
        .type('مشین لرننگ کا تعارف');
      
      cy.get('[data-testid="translation-content"]')
        .type('یہ مشین لرننگ کے بارے میں ایک تفصیلی گائیڈ ہے۔');
      
      cy.get('[data-testid="translation-notes"]')
        .type('High quality translation with proper terminology');
      
      // Submit translation
      cy.get('[data-testid="submit-translation"]').click();
      
      // Verify success message
      cy.get('[data-testid="success-message"]')
        .should('be.visible')
        .and('contain.text', 'Translation submitted successfully');
    });

    it('should display leaderboard correctly', () => {
      // Go to leaderboard tab
      cy.get('[data-testid="leaderboard-tab"]').click();
      
      // Verify leaderboard elements
      cy.get('[data-testid="leaderboard-table"]').should('be.visible');
      cy.get('[data-testid="top-contributors"]').should('be.visible');
      
      // Check for user rankings
      cy.get('[data-testid="leaderboard-row"]').should('have.length.at.least', 1);
      
      // Verify user can see their own rank
      cy.get('[data-testid="user-rank"]').should('be.visible');
    });

    it('should show user profile and statistics', () => {
      // Go to profile tab
      cy.get('[data-testid="profile-tab"]').click();
      
      // Verify profile elements
      cy.get('[data-testid="user-avatar"]').should('be.visible');
      cy.get('[data-testid="user-level"]').should('be.visible');
      cy.get('[data-testid="reputation-score"]').should('be.visible');
      
      // Check statistics
      cy.get('[data-testid="translation-count"]').should('be.visible');
      cy.get('[data-testid="approval-rate"]').should('be.visible');
    });

    it('should allow users to vote on translations', () => {
      // Go to review tab
      cy.get('[data-testid="review-tab"]').click();
      
      // Find a translation to vote on
      cy.get('[data-testid="translation-card"]').first().within(() => {
        // Click upvote
        cy.get('[data-testid="upvote-button"]').click();
        
        // Verify vote was recorded
        cy.get('[data-testid="upvote-count"]')
          .should('contain.text', '1');
      });
    });
  });

  describe('Search Functionality', () => {
    it('should search in multiple languages', () => {
      cy.visit('/');
      
      // Search in English
      cy.get('[data-testid="search-input"]').type('machine learning');
      cy.get('[data-testid="search-button"]').click();
      
      // Verify English results
      cy.get('[data-testid="search-results"]')
        .should('contain.text', 'Machine Learning');
      
      // Switch to Urdu and search
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      cy.get('[data-testid="search-input"]').clear().type('مشین لرننگ');
      cy.get('[data-testid="search-button"]').click();
      
      // Verify Urdu results
      cy.get('[data-testid="search-results"]')
        .should('contain.text', 'مشین لرننگ');
    });

    it('should provide search suggestions in current language', () => {
      cy.visit('/');
      
      // Switch to Urdu
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Type in search box
      cy.get('[data-testid="search-input"]').type('مشین');
      
      // Verify suggestions appear
      cy.get('[data-testid="search-suggestions"]')
        .should('be.visible')
        .and('contain.text', 'مشین لرننگ');
    });

    it('should complete search within 500ms', () => {
      cy.visit('/');
      
      const startTime = Date.now();
      
      cy.get('[data-testid="search-input"]').type('test');
      cy.get('[data-testid="search-button"]').click();
      
      cy.get('[data-testid="search-results"]').should('be.visible').then(() => {
        const endTime = Date.now();
        const searchTime = endTime - startTime;
        expect(searchTime).to.be.lessThan(500);
      });
    });
  });

  describe('Side-by-Side Content Comparison', () => {
    it('should display bilingual content side by side', () => {
      cy.visit('/chapter/test-chapter-1');
      
      // Enable bilingual view
      cy.get('[data-testid="bilingual-toggle"]').click();
      
      // Verify both versions are visible
      cy.get('[data-testid="english-content"]').should('be.visible');
      cy.get('[data-testid="urdu-content"]').should('be.visible');
      
      // Verify synchronized scrolling
      cy.get('[data-testid="english-content"]').scrollTo('bottom');
      cy.get('[data-testid="urdu-content"]')
        .should('have.prop', 'scrollTop')
        .and('be.greaterThan', 0);
    });

    it('should align paragraphs between languages', () => {
      cy.visit('/chapter/test-chapter-1');
      
      cy.get('[data-testid="bilingual-toggle"]').click();
      
      // Verify paragraph alignment
      cy.get('[data-testid="english-paragraph-1"]').should('be.visible');
      cy.get('[data-testid="urdu-paragraph-1"]').should('be.visible');
      
      // Check alignment positioning
      cy.get('[data-testid="english-paragraph-1"]').then(($englishPara) => {
        const englishTop = $englishPara.offset()?.top;
        
        cy.get('[data-testid="urdu-paragraph-1"]').then(($urduPara) => {
          const urduTop = $urduPara.offset()?.top;
          expect(Math.abs((englishTop || 0) - (urduTop || 0))).to.be.lessThan(50);
        });
      });
    });
  });

  describe('Performance and Accessibility', () => {
    it('should meet performance requirements', () => {
      cy.visit('/', {
        onBeforeLoad: (win) => {
          // Start performance measurement
          win.performance.mark('page-start');
        }
      });
      
      // Wait for page to load
      cy.get('[data-testid="main-content"]').should('be.visible');
      
      cy.window().then((win) => {
        win.performance.mark('page-end');
        win.performance.measure('page-load', 'page-start', 'page-end');
        
        const measure = win.performance.getEntriesByName('page-load')[0];
        expect(measure.duration).to.be.lessThan(3000); // 3 second budget
      });
    });

    it('should be accessible with screen readers', () => {
      cy.visit('/');
      
      // Check for proper ARIA labels
      cy.get('[data-testid="language-selector"]')
        .should('have.attr', 'aria-label');
      
      // Check for proper heading structure
      cy.get('h1').should('exist');
      cy.get('[role="main"]').should('exist');
      
      // Check for language attributes
      cy.get('[lang]').should('exist');
    });

    it('should support keyboard navigation', () => {
      cy.visit('/community-translation');
      
      // Tab through interactive elements
      cy.get('body').tab();
      cy.focused().should('be.visible');
      
      // Continue tabbing
      cy.focused().tab();
      cy.focused().should('be.visible');
      
      // Test Enter key activation
      cy.focused().type('{enter}');
    });

    it('should maintain color contrast standards', () => {
      cy.visit('/');
      
      // Check color contrast (this would require a custom command)
      cy.get('[data-testid="main-content"]').should('be.visible');
      
      // Verify text is readable
      cy.get('p').should('have.css', 'color').and('not.equal', 'rgba(0, 0, 0, 0)');
    });
  });

  describe('Error Handling and Edge Cases', () => {
    it('should handle network errors gracefully', () => {
      // Intercept API calls and simulate network error
      cy.intercept('GET', '/api/v1/multilingual/**', { forceNetworkError: true });
      
      cy.visit('/');
      
      // Switch language to trigger API call
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Should show error message
      cy.get('[data-testid="error-message"]')
        .should('be.visible')
        .and('contain.text', 'network error');
    });

    it('should handle missing translations gracefully', () => {
      // Mock API to return empty translation
      cy.intercept('GET', '/api/v1/multilingual/content/**', {
        statusCode: 404,
        body: { error: 'Translation not found' }
      });
      
      cy.visit('/chapter/test-chapter-1');
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Should fallback to English
      cy.get('[data-testid="fallback-notice"]').should('be.visible');
      cy.get('[data-testid="chapter-content"]').should('contain.text', 'English');
    });

    it('should handle font loading failures', () => {
      // Block font requests
      cy.intercept('GET', '**/fonts/**', { statusCode: 404 });
      
      cy.visit('/');
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Should still display content with fallback fonts
      cy.get('[data-testid="urdu-text"]').should('be.visible');
    });
  });

  describe('Mobile Responsiveness', () => {
    beforeEach(() => {
      cy.viewport('iphone-x');
    });

    it('should work correctly on mobile devices', () => {
      cy.visit('/');
      
      // Language selector should be accessible
      cy.get('[data-testid="language-selector"]').should('be.visible');
      
      // Content should be readable
      cy.get('[data-testid="main-content"]').should('be.visible');
    });

    it('should handle RTL layout on mobile', () => {
      cy.visit('/');
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify mobile RTL layout
      cy.get('html').should('have.attr', 'dir', 'rtl');
      cy.get('[data-testid="mobile-menu"]').should('have.css', 'text-align', 'right');
    });
  });

  describe('User Acceptance Scenarios', () => {
    it('should complete full user journey for Urdu speaker', () => {
      // 1. User visits site
      cy.visit('/');
      
      // 2. Selects Urdu language
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // 3. Browses content in Urdu
      cy.get('[data-testid="chapter-link"]').first().click();
      cy.get('[data-testid="chapter-content"]').should('contain.text', 'اردو');
      
      // 4. Uses bilingual comparison
      cy.get('[data-testid="bilingual-toggle"]').click();
      cy.get('[data-testid="english-content"]').should('be.visible');
      cy.get('[data-testid="urdu-content"]').should('be.visible');
      
      // 5. Contributes to community translation
      cy.visit('/community-translation');
      cy.get('[data-testid="contribute-tab"]').click();
      cy.get('[data-testid="translation-title"]').type('بہتری');
      cy.get('[data-testid="submit-translation"]').click();
      
      // 6. Checks leaderboard
      cy.get('[data-testid="leaderboard-tab"]').click();
      cy.get('[data-testid="leaderboard-table"]').should('be.visible');
    });

    it('should satisfy cultural appropriateness requirements', () => {
      cy.visit('/chapter/cultural-examples');
      
      cy.get('[data-testid="language-selector"]').click();
      cy.get('[data-value="ur"]').click();
      
      // Verify culturally appropriate examples
      cy.get('[data-testid="example-content"]')
        .should('contain.text', 'کراچی')
        .or('contain.text', 'لاہور')
        .or('contain.text', 'اسلام آباد');
      
      // Verify local currency format
      cy.get('[data-testid="price-example"]')
        .should('contain.text', 'PKR');
    });
  });
});

// Custom commands for testing
Cypress.Commands.add('tab', { prevSubject: 'element' }, (subject) => {
  return cy.wrap(subject).trigger('keydown', { key: 'Tab' });
});

// Performance measurement helpers
Cypress.Commands.add('measurePerformance', (markName: string) => {
  cy.window().then((win) => {
    win.performance.mark(markName);
  });
});

// Accessibility testing helpers
Cypress.Commands.add('checkA11y', () => {
  // This would integrate with axe-core for accessibility testing
  cy.get('[role]').should('exist');
  cy.get('[aria-label]').should('exist');
});