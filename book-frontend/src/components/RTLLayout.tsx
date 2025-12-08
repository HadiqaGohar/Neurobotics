/**
 * RTL Layout Components
 * Collection of layout components with RTL support
 */

import React, { ReactNode } from 'react';
import { useMultilingualContext } from '../contexts/MultilingualContext';
import { getRTLFlexDirection, getRTLTextAlign, getRTLSpacing } from '../utils/rtlUtils';

interface RTLFlexProps {
  children: ReactNode;
  direction?: 'row' | 'row-reverse' | 'column' | 'column-reverse';
  justify?: 'start' | 'end' | 'center' | 'between' | 'around' | 'evenly';
  align?: 'start' | 'end' | 'center' | 'stretch' | 'baseline';
  wrap?: boolean;
  gap?: number | string;
  className?: string;
  style?: React.CSSProperties;
}

export const RTLFlex: React.FC<RTLFlexProps> = ({
  children,
  direction = 'row',
  justify = 'start',
  align = 'start',
  wrap = false,
  gap = 0,
  className = '',
  style = {}
}) => {
  const { isRTL } = useMultilingualContext();
  
  const flexDirection = getRTLFlexDirection(direction, isRTL);
  
  const flexStyle: React.CSSProperties = {
    display: 'flex',
    flexDirection: direction,
    justifyContent: justify === 'start' ? (isRTL ? 'flex-end' : 'flex-start') :
                   justify === 'end' ? (isRTL ? 'flex-start' : 'flex-end') :
                   justify === 'between' ? 'space-between' :
                   justify === 'around' ? 'space-around' :
                   justify === 'evenly' ? 'space-evenly' : 'center',
    alignItems: align === 'start' ? 'flex-start' :
               align === 'end' ? 'flex-end' :
               align === 'center' ? 'center' :
               align === 'stretch' ? 'stretch' : 'baseline',
    flexWrap: wrap ? 'wrap' : 'nowrap',
    gap: typeof gap === 'number' ? `${gap}px` : gap,
    ...style
  };

  return (
    <div className={`rtl-flex ${flexDirection} ${className}`} style={flexStyle}>
      {children}
    </div>
  );
};

interface RTLGridProps {
  children: ReactNode;
  columns?: number | string;
  rows?: number | string;
  gap?: number | string;
  columnGap?: number | string;
  rowGap?: number | string;
  className?: string;
  style?: React.CSSProperties;
}

export const RTLGrid: React.FC<RTLGridProps> = ({
  children,
  columns = 'auto',
  rows = 'auto',
  gap = 0,
  columnGap,
  rowGap,
  className = '',
  style = {}
}) => {
  const { isRTL } = useMultilingualContext();
  
  const gridStyle: React.CSSProperties = {
    display: 'grid',
    gridTemplateColumns: typeof columns === 'number' ? `repeat(${columns}, 1fr)` : columns,
    gridTemplateRows: typeof rows === 'number' ? `repeat(${rows}, 1fr)` : rows,
    gap: typeof gap === 'number' ? `${gap}px` : gap,
    columnGap: columnGap ? (typeof columnGap === 'number' ? `${columnGap}px` : columnGap) : undefined,
    rowGap: rowGap ? (typeof rowGap === 'number' ? `${rowGap}px` : rowGap) : undefined,
    direction: isRTL ? 'rtl' : 'ltr',
    ...style
  };

  return (
    <div className={`rtl-grid ${isRTL ? 'rtl' : 'ltr'} ${className}`} style={gridStyle}>
      {children}
    </div>
  );
};

interface RTLContainerProps {
  children: ReactNode;
  maxWidth?: 'sm' | 'md' | 'lg' | 'xl' | '2xl' | 'full';
  padding?: number | string;
  margin?: number | string;
  centered?: boolean;
  className?: string;
  style?: React.CSSProperties;
}

export const RTLContainer: React.FC<RTLContainerProps> = ({
  children,
  maxWidth = 'lg',
  padding = '1rem',
  margin = '0 auto',
  centered = true,
  className = '',
  style = {}
}) => {
  const { isRTL } = useMultilingualContext();
  
  const maxWidthMap = {
    sm: '640px',
    md: '768px',
    lg: '1024px',
    xl: '1280px',
    '2xl': '1536px',
    full: '100%'
  };
  
  const containerStyle: React.CSSProperties = {
    maxWidth: maxWidthMap[maxWidth],
    padding: typeof padding === 'number' ? `${padding}px` : padding,
    margin: centered ? margin : undefined,
    direction: isRTL ? 'rtl' : 'ltr',
    ...style
  };

  return (
    <div className={`rtl-container ${isRTL ? 'rtl' : 'ltr'} ${className}`} style={containerStyle}>
      {children}
    </div>
  );
};

interface RTLTextProps {
  children: ReactNode;
  align?: 'left' | 'right' | 'center' | 'start' | 'end';
  size?: 'xs' | 'sm' | 'base' | 'lg' | 'xl' | '2xl' | '3xl';
  weight?: 'light' | 'normal' | 'medium' | 'semibold' | 'bold';
  color?: string;
  className?: string;
  style?: React.CSSProperties;
  tag?: keyof React.JSX.IntrinsicElements;
}

export const RTLText: React.FC<RTLTextProps> = ({
  children,
  align = 'start',
  size = 'base',
  weight = 'normal',
  color,
  className = '',
  style = {},
  tag: Tag = 'p'
}) => {
  const { isRTL } = useMultilingualContext();
  
  const textAlign = getRTLTextAlign(align, isRTL);
  
  const sizeMap = {
    xs: '0.75rem',
    sm: '0.875rem',
    base: '1rem',
    lg: '1.125rem',
    xl: '1.25rem',
    '2xl': '1.5rem',
    '3xl': '1.875rem'
  };
  
  const weightMap = {
    light: '300',
    normal: '400',
    medium: '500',
    semibold: '600',
    bold: '700'
  };
  
  const textStyle: React.CSSProperties = {
    fontSize: sizeMap[size],
    fontWeight: weightMap[weight],
    color,
    direction: isRTL ? 'rtl' : 'ltr',
    ...style
  };

  const Component = Tag as React.ElementType;

  return (
    <Component 
      className={`rtl-text ${textAlign} ${isRTL ? 'rtl' : 'ltr'} ${className}`} 
      style={textStyle}
    >
      {children}
    </Component>
  );
};

interface RTLCardProps {
  children: ReactNode;
  padding?: number | string;
  shadow?: boolean;
  border?: boolean;
  rounded?: boolean;
  className?: string;
  style?: React.CSSProperties;
}

export const RTLCard: React.FC<RTLCardProps> = ({
  children,
  padding = '1.5rem',
  shadow = true,
  border = true,
  rounded = true,
  className = '',
  style = {}
}) => {
  const { isRTL } = useMultilingualContext();
  
  const cardStyle: React.CSSProperties = {
    padding: typeof padding === 'number' ? `${padding}px` : padding,
    boxShadow: shadow ? '0 1px 3px 0 rgba(0, 0, 0, 0.1), 0 1px 2px 0 rgba(0, 0, 0, 0.06)' : 'none',
    border: border ? '1px solid #e5e7eb' : 'none',
    borderRadius: rounded ? '0.5rem' : '0',
    direction: isRTL ? 'rtl' : 'ltr',
    ...style
  };

  return (
    <div className={`rtl-card ${isRTL ? 'rtl' : 'ltr'} ${className}`} style={cardStyle}>
      {children}
    </div>
  );
};

interface RTLButtonProps {
  children: ReactNode;
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost';
  size?: 'sm' | 'md' | 'lg';
  disabled?: boolean;
  loading?: boolean;
  icon?: ReactNode;
  iconPosition?: 'start' | 'end';
  onClick?: () => void;
  className?: string;
  style?: React.CSSProperties;
  type?: 'button' | 'submit' | 'reset';
}

export const RTLButton: React.FC<RTLButtonProps> = ({
  children,
  variant = 'primary',
  size = 'md',
  disabled = false,
  loading = false,
  icon,
  iconPosition = 'start',
  onClick,
  className = '',
  style = {},
  type = 'button'
}) => {
  const { isRTL } = useMultilingualContext();
  
  const variantStyles = {
    primary: {
      backgroundColor: '#3b82f6',
      color: 'white',
      border: '1px solid #3b82f6'
    },
    secondary: {
      backgroundColor: '#6b7280',
      color: 'white',
      border: '1px solid #6b7280'
    },
    outline: {
      backgroundColor: 'transparent',
      color: '#3b82f6',
      border: '1px solid #3b82f6'
    },
    ghost: {
      backgroundColor: 'transparent',
      color: '#374151',
      border: '1px solid transparent'
    }
  };
  
  const sizeStyles = {
    sm: { padding: '0.5rem 1rem', fontSize: '0.875rem' },
    md: { padding: '0.75rem 1.5rem', fontSize: '1rem' },
    lg: { padding: '1rem 2rem', fontSize: '1.125rem' }
  };
  
  const buttonStyle: React.CSSProperties = {
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '0.5rem',
    borderRadius: '0.375rem',
    cursor: disabled || loading ? 'not-allowed' : 'pointer',
    opacity: disabled || loading ? 0.6 : 1,
    transition: 'all 0.2s',
    direction: isRTL ? 'rtl' : 'ltr',
    flexDirection: (isRTL && iconPosition === 'start') || (!isRTL && iconPosition === 'end') ? 'row-reverse' : 'row',
    ...variantStyles[variant],
    ...sizeStyles[size],
    ...style
  };

  return (
    <button
      type={type}
      onClick={onClick}
      disabled={disabled || loading}
      className={`rtl-button ${variant} ${size} ${isRTL ? 'rtl' : 'ltr'} ${className}`}
      style={buttonStyle}
    >
      {loading && (
        <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-current"></div>
      )}
      {!loading && icon && iconPosition === 'start' && icon}
      {children}
      {!loading && icon && iconPosition === 'end' && icon}
    </button>
  );
};

export default {
  RTLFlex,
  RTLGrid,
  RTLContainer,
  RTLText,
  RTLCard,
  RTLButton
};