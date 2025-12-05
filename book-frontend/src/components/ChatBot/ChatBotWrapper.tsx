import React from 'react';
import ChatBot from './index';
import BrowserOnly from '@docusaurus/BrowserOnly';

/**
 * Wrapper component for ChatBot that only renders on the client side
 * This is necessary for Docusaurus SSR compatibility
 */
const ChatBotWrapper: React.FC = () => {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ChatBot />}
    </BrowserOnly>
  );
};

export default ChatBotWrapper;