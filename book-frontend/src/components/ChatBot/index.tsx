import React, { useState, useEffect, useRef } from 'react';
import ChatWindow from './ChatWindow';
import FloatingIcon from './FloatingIcon';
import { ChatMessage, ChatSession } from './types';
import { chatAPI } from './api';
import './styles.css';

const ChatBot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [sessionId, setSessionId] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const [windowSize, setWindowSize] = useState<'small' | 'large'>('small');
  const [showHistory, setShowHistory] = useState(false);

  // Initialize session on mount
  useEffect(() => {
    const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    setSessionId(newSessionId);
    
    // Load chat history if exists
    loadChatHistory(newSessionId);
  }, []);

  const loadChatHistory = async (sessionId: string) => {
    try {
      const history = await chatAPI.getChatHistory(sessionId);
      setMessages(history.messages || []);
    } catch (error) {
      console.error('Failed to load chat history:', error);
    }
  };

  const sendMessage = async (content: string, isAskAI: boolean = false) => {
    if (!content.trim()) return;

    // Add user message immediately
    const userMessage: ChatMessage = {
      id: `user_${Date.now()}`,
      content,
      sender: 'user',
      timestamp: new Date(),
      sessionId
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = isAskAI 
        ? await chatAPI.askAI(content, sessionId)
        : await chatAPI.sendMessage(content, sessionId);

      setMessages(prev => [...prev, response.message]);
    } catch (error) {
      console.error('Failed to send message:', error);
      // Add error message
      const errorMessage: ChatMessage = {
        id: `error_${Date.now()}`,
        content: 'Sorry, I encountered an error. Please try again.',
        sender: 'ai',
        timestamp: new Date(),
        sessionId
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleVoiceInput = async (audioBlob: Blob) => {
    setIsLoading(true);
    try {
      // Convert blob to base64
      const reader = new FileReader();
      reader.onloadend = async () => {
        const base64Audio = reader.result as string;
        const response = await chatAPI.processVoice(base64Audio, sessionId);
        setMessages(prev => [...prev, response.message]);
        setIsLoading(false);
      };
      reader.readAsDataURL(audioBlob);
    } catch (error) {
      console.error('Failed to process voice input:', error);
      setIsLoading(false);
    }
  };

  const handleFileUpload = async (file: File) => {
    setIsLoading(true);
    try {
      // Add user message showing file upload
      const userMessage: ChatMessage = {
        id: `file_${Date.now()}`,
        content: `📎 Uploaded file: ${file.name} (${(file.size / 1024).toFixed(1)} KB)`,
        sender: 'user',
        timestamp: new Date(),
        sessionId
      };
      
      setMessages(prev => [...prev, userMessage]);
      
      // Process file (mock implementation - you can integrate with your backend)
      const response = await chatAPI.processFile(file, sessionId);
      setMessages(prev => [...prev, response.message]);
    } catch (error) {
      console.error('Failed to process file upload:', error);
      // Add error message
      const errorMessage: ChatMessage = {
        id: `error_${Date.now()}`,
        content: 'Sorry, I encountered an error processing your file. Please try again.',
        sender: 'ai',
        timestamp: new Date(),
        sessionId
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const copyToClipboard = (text: string) => {
    navigator.clipboard.writeText(text).then(() => {
      // You could add a toast notification here
      console.log('Text copied to clipboard');
    }).catch(err => {
      console.error('Failed to copy text: ', err);
    });
  };

  const toggleSize = () => {
    setWindowSize(prev => prev === 'small' ? 'large' : 'small');
  };

  const handleShowHistory = () => {
    setShowHistory(true);
    // You can implement history modal or sidebar here
    console.log('Show chat history');
  };

  return (
    <div className="chatbot-container">
      <FloatingIcon 
        isOpen={isOpen}
        onClick={() => setIsOpen(!isOpen)}
      />
      
      {isOpen && (
        <>
          <ChatWindow
            messages={messages}
            onSendMessage={sendMessage}
            onVoiceInput={handleVoiceInput}
            onFileUpload={handleFileUpload}
            onCopyMessage={copyToClipboard}
            onClose={() => setIsOpen(false)}
            onToggleSize={toggleSize}
            onShowHistory={handleShowHistory}
            windowSize={windowSize}
            isLoading={isLoading}
          />
          
          {/* Floating icon inside chat for easy access */}
          <div className="chat-floating-icon">
            <button
              className="mini-chat-icon"
              onClick={() => setIsOpen(false)}
              aria-label="Minimize chat"
              title="Minimize"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
              </svg>
            </button>
          </div>
        </>
      )}
    </div>
  );
};

export default ChatBot;