import React from 'react';

interface FloatingIconProps {
  isOpen: boolean;
  onClick: () => void;
}

const FloatingIcon: React.FC<FloatingIconProps> = ({ isOpen, onClick }) => {
  return (
    <button
      className={`floating-chat-icon ${isOpen ? 'open' : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      aria-expanded={isOpen}
    >
      <div className="icon-content">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </div>
      
      {!isOpen && (
        <div className="notification-badge">
          <span className="pulse-dot"></span>
        </div>
      )}
    </button>
  );
};

export default FloatingIcon;