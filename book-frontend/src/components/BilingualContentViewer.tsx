/**
 * Bilingual Content Viewer Component
 * Side-by-side content comparison with synchronized scrolling
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useMultilingualContext } from '../contexts/MultilingualContext';
import { multilingualAPI } from '../services/multilingualAPI';
import './BilingualContentViewer.css';

interface ContentSection {
  id: string;
  type: 'paragraph' | 'heading' | 'code' | 'list' | 'quote';
  content: string;
  level?: number; // For headings
}

interface BilingualContent {
  source: {
    language: string;
    title: string;
    sections: ContentSection[];
  };
  target: {
    language: string;
    title: string;
    sections: ContentSection[];
  };
}

interface BilingualContentViewerProps {
  contentType: string;
  contentId: string;
  sourceLanguage?: string;
  targetLanguage?: string;
  className?: string;
  onLanguageSwitch?: (sourceLanguage: string, targetLanguage: string) => void;
}

const BilingualContentViewer: React.FC<BilingualContentViewerProps> = ({
  contentType,
  contentId,
  sourceLanguage = 'en',
  targetLanguage,
  className = '',
  onLanguageSwitch
}) => {
  const { currentLanguage, supportedLanguages } = useMultilingualContext();
  const [content, setContent] = useState<BilingualContent | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [viewMode, setViewMode] = useState<'side-by-side' | 'single' | 'overlay'>('side-by-side');
  const [activeLanguage, setActiveLanguage] = useState<'source' | 'target'>('source');
  const [syncScroll, setSyncScroll] = useState(true);
  const [highlightDifferences, setHighlightDifferences] = useState(false);
  
  const sourceRef = useRef<HTMLDivElement>(null);
  const targetRef = useRef<HTMLDivElement>(null);
  const scrollTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isScrollingRef = useRef(false);

  const effectiveTargetLanguage = targetLanguage || currentLanguage;

  useEffect(() => {
    loadBilingualContent();
  }, [contentType, contentId, sourceLanguage, effectiveTargetLanguage]);

  const loadBilingualContent = async () => {
    setLoading(true);
    setError(null);

    try {
      // Load source content
      const sourceResponse = await multilingualAPI.getTranslatedContent(
        contentType,
        contentId,
        sourceLanguage,
        false
      );

      // Load target content
      const targetResponse = await multilingualAPI.getTranslatedContent(
        contentType,
        contentId,
        effectiveTargetLanguage,
        true // Enable fallback
      );

      if (sourceResponse.content && targetResponse.content) {
        const bilingualContent: BilingualContent = {
          source: {
            language: sourceLanguage,
            title: sourceResponse.content.title || 'Untitled',
            sections: parseContentSections(sourceResponse.content.content || '')
          },
          target: {
            language: effectiveTargetLanguage,
            title: targetResponse.content.title || 'Untitled',
            sections: parseContentSections(targetResponse.content.content || '')
          }
        };

        setContent(bilingualContent);
      } else {
        setError('Content not available in one or both languages');
      }
    } catch (err) {
      console.error('Error loading bilingual content:', err);
      setError('Failed to load content');
    } finally {
      setLoading(false);
    }
  };

  const parseContentSections = (content: string): ContentSection[] => {
    const sections: ContentSection[] = [];
    const lines = content.split('\n');
    let currentSection = '';
    let sectionId = 0;

    for (const line of lines) {
      const trimmedLine = line.trim();
      
      if (!trimmedLine) {
        if (currentSection) {
          sections.push({
            id: `section-${sectionId++}`,
            type: 'paragraph',
            content: currentSection.trim()
          });
          currentSection = '';
        }
        continue;
      }

      // Detect headings
      if (trimmedLine.startsWith('#')) {
        if (currentSection) {
          sections.push({
            id: `section-${sectionId++}`,
            type: 'paragraph',
            content: currentSection.trim()
          });
          currentSection = '';
        }

        const level = trimmedLine.match(/^#+/)?.[0].length || 1;
        sections.push({
          id: `section-${sectionId++}`,
          type: 'heading',
          content: trimmedLine.replace(/^#+\s*/, ''),
          level
        });
        continue;
      }

      // Detect code blocks
      if (trimmedLine.startsWith('```')) {
        if (currentSection) {
          sections.push({
            id: `section-${sectionId++}`,
            type: 'paragraph',
            content: currentSection.trim()
          });
          currentSection = '';
        }

        // Find end of code block
        let codeContent = '';
        let i = lines.indexOf(line) + 1;
        while (i < lines.length && !lines[i].trim().startsWith('```')) {
          codeContent += lines[i] + '\n';
          i++;
        }

        sections.push({
          id: `section-${sectionId++}`,
          type: 'code',
          content: codeContent.trim()
        });
        continue;
      }

      // Detect quotes
      if (trimmedLine.startsWith('>')) {
        if (currentSection) {
          sections.push({
            id: `section-${sectionId++}`,
            type: 'paragraph',
            content: currentSection.trim()
          });
          currentSection = '';
        }

        sections.push({
          id: `section-${sectionId++}`,
          type: 'quote',
          content: trimmedLine.replace(/^>\s*/, '')
        });
        continue;
      }

      // Regular content
      currentSection += line + '\n';
    }

    // Add remaining content
    if (currentSection) {
      sections.push({
        id: `section-${sectionId++}`,
        type: 'paragraph',
        content: currentSection.trim()
      });
    }

    return sections;
  };

  const handleScroll = useCallback((source: 'source' | 'target') => {
    if (!syncScroll || isScrollingRef.current) return;

    const sourceElement = sourceRef.current;
    const targetElement = targetRef.current;

    if (!sourceElement || !targetElement) return;

    isScrollingRef.current = true;

    if (scrollTimeoutRef.current) {
      clearTimeout(scrollTimeoutRef.current);
    }

    const activeElement = source === 'source' ? sourceElement : targetElement;
    const passiveElement = source === 'source' ? targetElement : sourceElement;

    // Calculate scroll percentage
    const scrollTop = activeElement.scrollTop;
    const scrollHeight = activeElement.scrollHeight - activeElement.clientHeight;
    const scrollPercentage = scrollHeight > 0 ? scrollTop / scrollHeight : 0;

    // Apply to passive element
    const passiveScrollHeight = passiveElement.scrollHeight - passiveElement.clientHeight;
    const targetScrollTop = passiveScrollHeight * scrollPercentage;

    passiveElement.scrollTop = targetScrollTop;

    scrollTimeoutRef.current = setTimeout(() => {
      isScrollingRef.current = false;
    }, 100);
  }, [syncScroll]);

  const handleLanguageSwap = () => {
    if (onLanguageSwitch) {
      onLanguageSwitch(effectiveTargetLanguage, sourceLanguage);
    }
  };

  const renderSection = (section: ContentSection, language: string) => {
    const isRTL = supportedLanguages.find(lang => lang.code === language)?.direction === 'rtl';
    const baseClasses = `content-section ${isRTL ? 'rtl' : 'ltr'}`;

    switch (section.type) {
      case 'heading':
        const HeadingTag = `h${Math.min(section.level || 1, 6)}` as keyof JSX.IntrinsicElements;
        return (
          <HeadingTag
            key={section.id}
            className={`${baseClasses} heading heading-${section.level}`}
            dir={isRTL ? 'rtl' : 'ltr'}
          >
            {section.content}
          </HeadingTag>
        );

      case 'code':
        return (
          <pre key={section.id} className={`${baseClasses} code-block`}>
            <code>{section.content}</code>
          </pre>
        );

      case 'quote':
        return (
          <blockquote
            key={section.id}
            className={`${baseClasses} quote`}
            dir={isRTL ? 'rtl' : 'ltr'}
          >
            {section.content}
          </blockquote>
        );

      case 'list':
        return (
          <ul key={section.id} className={`${baseClasses} list`} dir={isRTL ? 'rtl' : 'ltr'}>
            {section.content.split('\n').map((item, index) => (
              <li key={index}>{item.replace(/^[-*]\s*/, '')}</li>
            ))}
          </ul>
        );

      default:
        return (
          <p
            key={section.id}
            className={`${baseClasses} paragraph`}
            dir={isRTL ? 'rtl' : 'ltr'}
          >
            {section.content}
          </p>
        );
    }
  };

  const renderControls = () => (
    <div className="bilingual-controls">
      <div className="view-mode-controls">
        <button
          className={`control-btn ${viewMode === 'side-by-side' ? 'active' : ''}`}
          onClick={() => setViewMode('side-by-side')}
          title="Side by side view"
        >
          <svg width="20" height="20" viewBox="0 0 20 20">
            <rect x="1" y="3" width="8" height="14" fill="none" stroke="currentColor" strokeWidth="1.5"/>
            <rect x="11" y="3" width="8" height="14" fill="none" stroke="currentColor" strokeWidth="1.5"/>
          </svg>
        </button>
        
        <button
          className={`control-btn ${viewMode === 'single' ? 'active' : ''}`}
          onClick={() => setViewMode('single')}
          title="Single view"
        >
          <svg width="20" height="20" viewBox="0 0 20 20">
            <rect x="3" y="3" width="14" height="14" fill="none" stroke="currentColor" strokeWidth="1.5"/>
          </svg>
        </button>

        <button
          className={`control-btn ${viewMode === 'overlay' ? 'active' : ''}`}
          onClick={() => setViewMode('overlay')}
          title="Overlay view"
        >
          <svg width="20" height="20" viewBox="0 0 20 20">
            <rect x="2" y="2" width="12" height="12" fill="none" stroke="currentColor" strokeWidth="1.5"/>
            <rect x="6" y="6" width="12" height="12" fill="none" stroke="currentColor" strokeWidth="1.5"/>
          </svg>
        </button>
      </div>

      <div className="feature-controls">
        <label className="control-label">
          <input
            type="checkbox"
            checked={syncScroll}
            onChange={(e) => setSyncScroll(e.target.checked)}
          />
          Sync Scroll
        </label>

        <label className="control-label">
          <input
            type="checkbox"
            checked={highlightDifferences}
            onChange={(e) => setHighlightDifferences(e.target.checked)}
          />
          Highlight Differences
        </label>

        <button className="control-btn" onClick={handleLanguageSwap} title="Swap languages">
          <svg width="20" height="20" viewBox="0 0 20 20">
            <path d="M8 5l4 4-4 4M12 5l-4 4 4 4" stroke="currentColor" strokeWidth="1.5" fill="none"/>
          </svg>
        </button>
      </div>
    </div>
  );

  if (loading) {
    return (
      <div className={`bilingual-viewer loading ${className}`}>
        <div className="loading-spinner">
          <div className="spinner"></div>
          <p>Loading bilingual content...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={`bilingual-viewer error ${className}`}>
        <div className="error-message">
          <h3>Error Loading Content</h3>
          <p>{error}</p>
          <button onClick={loadBilingualContent} className="retry-btn">
            Retry
          </button>
        </div>
      </div>
    );
  }

  if (!content) {
    return (
      <div className={`bilingual-viewer empty ${className}`}>
        <p>No content available</p>
      </div>
    );
  }

  return (
    <div className={`bilingual-viewer ${viewMode} ${className}`}>
      {renderControls()}

      <div className="content-container">
        {viewMode === 'side-by-side' && (
          <>
            <div className="content-panel source-panel">
              <div className="panel-header">
                <h2>{content.source.title}</h2>
                <span className="language-tag">{content.source.language.toUpperCase()}</span>
              </div>
              <div
                ref={sourceRef}
                className="panel-content"
                onScroll={() => handleScroll('source')}
              >
                {content.source.sections.map(section => 
                  renderSection(section, content.source.language)
                )}
              </div>
            </div>

            <div className="content-panel target-panel">
              <div className="panel-header">
                <h2>{content.target.title}</h2>
                <span className="language-tag">{content.target.language.toUpperCase()}</span>
              </div>
              <div
                ref={targetRef}
                className="panel-content"
                onScroll={() => handleScroll('target')}
              >
                {content.target.sections.map(section => 
                  renderSection(section, content.target.language)
                )}
              </div>
            </div>
          </>
        )}

        {viewMode === 'single' && (
          <div className="single-content">
            <div className="language-switcher">
              <button
                className={`lang-btn ${activeLanguage === 'source' ? 'active' : ''}`}
                onClick={() => setActiveLanguage('source')}
              >
                {content.source.language.toUpperCase()}
              </button>
              <button
                className={`lang-btn ${activeLanguage === 'target' ? 'active' : ''}`}
                onClick={() => setActiveLanguage('target')}
              >
                {content.target.language.toUpperCase()}
              </button>
            </div>

            <div className="single-panel">
              <h2>
                {activeLanguage === 'source' ? content.source.title : content.target.title}
              </h2>
              <div className="panel-content">
                {(activeLanguage === 'source' ? content.source.sections : content.target.sections)
                  .map(section => renderSection(
                    section, 
                    activeLanguage === 'source' ? content.source.language : content.target.language
                  ))
                }
              </div>
            </div>
          </div>
        )}

        {viewMode === 'overlay' && (
          <div className="overlay-content">
            <div className="overlay-controls">
              <div className="opacity-slider">
                <label>Source Opacity:</label>
                <input type="range" min="0" max="100" defaultValue="70" />
              </div>
            </div>
            
            <div className="overlay-container">
              <div className="overlay-layer source-layer">
                <h2>{content.source.title}</h2>
                {content.source.sections.map(section => 
                  renderSection(section, content.source.language)
                )}
              </div>
              
              <div className="overlay-layer target-layer">
                <h2>{content.target.title}</h2>
                {content.target.sections.map(section => 
                  renderSection(section, content.target.language)
                )}
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default BilingualContentViewer;