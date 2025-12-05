import React, { useState } from 'react';

interface MessageInputProps {
  onSendMessage: (message: string, isAskAI?: boolean) => void;
  disabled?: boolean;
}

const MessageInput: React.FC<MessageInputProps> = ({ onSendMessage, disabled = false }) => {
  const [message, setMessage] = useState('');

  const handleSubmit = (e: React.FormEvent, isAskAI: boolean = false) => {
    e.preventDefault();
    if (message.trim() && !disabled) {
      onSendMessage(message, isAskAI);
      setMessage('');
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form onSubmit={(e) => handleSubmit(e)} className="message-input-form">
      <div className="input-wrapper">
        <textarea
          value={message}
          onChange={(e) => setMessage(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Type your message..."
          className="message-textarea"
          disabled={disabled}
          rows={1}
        />
        
        <div className="input-actions">
          <button
            type="button"
            onClick={(e) => handleSubmit(e, true)}
            disabled={!message.trim() || disabled}
            className="ask-ai-button"
            title="Ask AI for help"
          >
            ✨
          </button>
          
          <button
            type="submit"
            disabled={!message.trim() || disabled}
            className="send-button"
            title="Send message"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22,2 15,22 11,13 2,9 22,2"></polygon>
            </svg>
          </button>
        </div>
      </div>
    </form>
  );
};

export default MessageInput;