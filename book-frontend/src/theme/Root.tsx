import React from 'react';
import { AuthProvider } from '../auth/AuthContext';
import ChatBotWrapper from '../components/ChatBot/ChatBotWrapper';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatBotWrapper />
    </AuthProvider>
  );
}