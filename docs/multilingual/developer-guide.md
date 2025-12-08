# Developer Guide - Multilingual System

## Quick Start

### Adding a New Language

```python
# Backend - Add language configuration
from src.multilingual.language_framework import language_framework

result = await language_framework.add_new_language(
    language_code="ar",
    language_name="Arabic", 
    native_name="العربية",
    template="rtl_arabic_script"
)
```

```typescript
// Frontend - Update language selector
const languages = [
  { code: 'en', name: 'English', nativeName: 'English' },
  { code: 'ur', name: 'Urdu', nativeName: 'اردو' },
  { code: 'ar', name: 'Arabic', nativeName: 'العربية' }
];
```

### Translation Integration

```typescript
// Get translated content
const content = await multilingualAPI.getTranslatedContent(
  'chapter', 'chapter-1', 'ur', true
);

// Advanced translation with context
const translation = await advancedTranslationAPI.translate({
  source_text: "Machine Learning",
  source_language: "en",
  target_language: "ur",
  domain: "technical"
});
```

### Performance Optimization

```typescript
// Enable performance optimizations
import { optimizeFontLoading, enableLazyLoading } from '../utils/multilingualPerformance';

// Optimize fonts for language
await optimizeFontLoading('ur');

// Enable lazy loading for content
const elements = document.querySelectorAll('[data-lazy-translate]');
enableLazyLoading(elements);
```

## Architecture Patterns

### Component Structure
```
src/
├── components/
│   ├── CommunityTranslation/
│   ├── UX/
│   └── Monitoring/
├── services/
│   ├── multilingualAPI.ts
│   └── communityTranslationAPI.ts
└── utils/
    └── multilingualPerformance.ts
```

### State Management
```typescript
// Language context
const LanguageContext = createContext({
  currentLanguage: 'en',
  availableLanguages: [],
  switchLanguage: (code: string) => {},
  isRTL: false
});
```

## Testing Guidelines

### Unit Tests
```typescript
describe('Multilingual Components', () => {
  test('language switching', () => {
    render(<LanguageSelector />);
    fireEvent.click(screen.getByText('اردو'));
    expect(document.documentElement.lang).toBe('ur');
  });
});
```

### E2E Tests
```typescript
// Cypress test
cy.get('[data-testid="language-selector"]').click();
cy.get('[data-value="ur"]').click();
cy.get('html').should('have.attr', 'dir', 'rtl');
```

## Deployment

### Feature Flags
```python
# Check if feature is enabled for user
enabled = await deployment_system.is_feature_enabled_for_user(
    user_id=user.id,
    feature=FeatureFlag.LANGUAGE_SWITCHING,
    user_segments=["beta_testers"]
)
```

### Monitoring
```python
# Record performance metrics
await monitoring_system.record_metric(
    name="translation_load_time",
    value=1.2,
    metric_type=MetricType.PERFORMANCE,
    unit="seconds"
)
```