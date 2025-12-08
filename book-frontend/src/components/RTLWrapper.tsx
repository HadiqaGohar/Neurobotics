/**
 * RTL Wrapper Component
 * Provides RTL context and styling for child components
 */

import React, { useEffect, ReactNode } from 'react';
import { useMultilingualContext } from '../contexts/MultilingualContext';
import { applyRTLConfig, getRTLFontFamily } from '../utils/rtlUtils';

interface RTLWrapperProps {
  children: ReactNode;
  className?: string;
  style?: React.CSSProperties;
  forceDirection?: 'ltr' | 'rtl';
  enableFontOptimization?: boolean;
}

const RTLWrapper: React.FC<RTLWrapperProps> = ({
  children,
  className = '',
  style = {},
  forceDirection,
  enableFontOptimization = true
}) => {
  const { currentLanguage, isRTL } = useMultilingualContext();
  
  const direction = forceDirection || (isRTL ? 'rtl' : 'ltr');
  const isRTLDirection = direction === 'rtl';

  useEffect(() => {
    // Apply RTL configuration to document
    applyRTLConfig({
      isRTL: isRTLDirection,
      language: currentLanguage,
      direction
    });
  }, [currentLanguage, isRTLDirection, direction]);

  const wrapperStyle: React.CSSProperties = {
    direction,
    textAlign: isRTLDirection ? 'right' : 'left',
    ...style
  };

  // Add font optimization for RTL languages
  if (enableFontOptimization && isRTLDirection) {
    wrapperStyle.fontFamily = getRTLFontFamily(currentLanguage);
  }

  const wrapperClassName = [
    className,
    isRTLDirection ? 'rtl-wrapper' : 'ltr-wrapper',
    `lang-${currentLanguage}`
  ].filter(Boolean).join(' ');

  return (
    <div 
      className={wrapperClassName}
      style={wrapperStyle}
      dir={direction}
      lang={currentLanguage}
    >
      {children}
    </div>
  );
};

export default RTLWrapper;