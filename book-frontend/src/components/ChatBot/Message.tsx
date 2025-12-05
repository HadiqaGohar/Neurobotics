import React, { useState } from 'react';
import { MessageProps } from './types';

const Message: React.FC<MessageProps> = ({ message, onCopy }) => {
  const [showCopyFeedback, setShowCopyFeedback] = useState(false);

  const handleCopy = () => {
    onCopy(message.content);
    setShowCopyFeedback(true);
    setTimeout(() => setShowCopyFeedback(false), 2000);
  };

  const formatTime = (timestamp: Date) => {
    return new Date(timestamp).toLocaleTimeString([], { 
      hour: '2-digit', 
      minute: '2-digit' 
    });
  };

  return (
    <div className={`message ${message.sender}`}>
      <div className="message-content">
        <div className="message-bubble">
          <div className="message-text">
            {message.content}
          </div>
          
          <div className="message-actions">
            <button
              className="copy-btn"
              onClick={handleCopy}
              aria-label="Copy message"
              title="Copy to clipboard"
            >
              {showCopyFeedback ? (
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="20,6 9,17 4,12"></polyline>
                </svg>
              ) : (
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
                  <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
                </svg>
              )}
            </button>
          </div>
        </div>
        
        <div className="message-meta">
          <span className="message-time">{formatTime(message.timestamp)}</span>
          {showCopyFeedback && (
            <span className="copy-feedback">Copied!</span>
          )}
        </div>
      </div>
      
      {message.sender === 'ai' && (
        <div className="message-avatar">
          <div className="avatar-icon">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="3"></circle>
              <path d="M12 1v6m0 6v6m11-7h-6m-6 0H1"></path>
            </svg>
          </div>
        </div>
      )}
    </div>
  );
};

export default Message;