import React from 'react';
import ChatBotWrapper from '../components/ChatBot/ChatBotWrapper';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatBotWrapper />
    </>
  );
}