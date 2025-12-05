import React, { useState, useRef, useEffect } from 'react';
import MessageList from './MessageList';
import MessageInput from './MessageInput';
import VoiceInput from './VoiceInput';
import FileUpload from './FileUpload';
import { ChatWindowProps } from './types';

const ChatWindow: React.FC<ChatWindowProps> = ({
  messages,
  onSendMessage,
  onVoiceInput,
  onFileUpload,
  onCopyMessage,
  onClose,
  onToggleSize,
  onShowHistory,
  windowSize,
  isLoading
}) => {
  const [inputValue, setInputValue] = useState('');
  const [isVoiceRecording, setIsVoiceRecording] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSendMessage = (isAskAI: boolean = false) => {
    if (inputValue.trim()) {
      onSendMessage(inputValue, isAskAI);
      setInputValue('');
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className={`chat-window ${windowSize}`}>
      {/* Header */}
      <div className="chat-header">
        <div className="header-left">
          <div className="chat-title">
            <h3>AI Assistant</h3>
            <span className="status-indicator online">Online</span>
          </div>
        </div>
        
        <div className="header-controls">
          <button
            className="control-btn history-btn"
            onClick={onShowHistory}
            aria-label="Chat history"
            title="Chat History"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M3 3v5h5"></path>
              <path d="M3.05 13A9 9 0 1 0 6 5.3L3 8"></path>
              <path d="M12 7v5l4 2"></path>
            </svg>
          </button>
          
          <button
            className="control-btn resize-btn"
            onClick={onToggleSize}
            aria-label={windowSize === 'small' ? 'Fullscreen' : 'Window mode'}
            title={windowSize === 'small' ? 'Fullscreen' : 'Window mode'}
          >
            {windowSize === 'small' ? (
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="15,3 21,3 21,9"></polyline>
                <polyline points="9,21 3,21 3,15"></polyline>
                <line x1="21" y1="3" x2="14" y2="10"></line>
                <line x1="3" y1="21" x2="10" y2="14"></line>
              </svg>
            ) : (
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="4,14 10,14 10,20"></polyline>
                <polyline points="20,10 14,10 14,4"></polyline>
                <line x1="14" y1="10" x2="21" y2="3"></line>
                <line x1="3" y1="21" x2="10" y2="14"></line>
              </svg>
            )}
          </button>
          
          <button
            className="control-btn close-btn"
            onClick={onClose}
            aria-label="Close chat"
            title="Close"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>
      </div>

      {/* Messages Area */}
      <div className="chat-messages">
        <MessageList messages={messages} onCopyMessage={onCopyMessage} />
        {isLoading && (
          <div className="typing-indicator">
            <div className="typing-dots">
              <span></span>
              <span></span>
              <span></span>
            </div>
            <span className="typing-text">AI is typing...</span>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div className="chat-input-area">
        <div className="input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your message..."
            className="message-input"
            rows={1}
            disabled={isLoading}
          />
          
          <div className="input-controls">
            <FileUpload
              onFileUpload={onFileUpload}
              disabled={isLoading}
            />
            
            <VoiceInput
              onVoiceInput={onVoiceInput}
              isRecording={isVoiceRecording}
              onToggleRecording={() => setIsVoiceRecording(!isVoiceRecording)}
            />
            
            <button
              className="control-btn ask-ai-btn"
              onClick={() => handleSendMessage(true)}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Ask AI for help"
              title="Ask AI"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="12" cy="12" r="3"></circle>
                <path d="M12 1v6m0 6v6m11-7h-6m-6 0H1"></path>
              </svg>
            </button>
            
            <button
              className="control-btn send-btn"
              onClick={() => handleSendMessage()}
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
              title="Send"
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22,2 15,22 11,13 2,9 22,2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChatWindow;