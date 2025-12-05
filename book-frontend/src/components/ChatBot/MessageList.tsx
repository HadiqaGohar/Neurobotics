import React from 'react';
import Message from './Message';
import { ChatMessage } from './types';

interface MessageListProps {
  messages: ChatMessage[];
  onCopyMessage: (text: string) => void;
}

const MessageList: React.FC<MessageListProps> = ({ messages, onCopyMessage }) => {
  if (messages.length === 0) {
    return (
      <div className="empty-chat">
        <div className="welcome-message">
          <div className="welcome-icon">
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
              <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
            </svg>
          </div>
          <h4>Welcome to AI Assistant!</h4>
          <p>I'm here to help you with any questions or tasks. Feel free to:</p>
          <ul>
            <li>Ask me anything</li>
            <li>Use voice input 🎤</li>
            <li>Request AI help with the ✨ button</li>
            <li>Copy responses to clipboard</li>
          </ul>
          <p>How can I assist you today?</p>
        </div>
      </div>
    );
  }

  return (
    <div className="message-list">
      {messages.map((message) => (
        <Message
          key={message.id}
          message={message}
          onCopy={onCopyMessage}
        />
      ))}
    </div>
  );
};

export default MessageList;