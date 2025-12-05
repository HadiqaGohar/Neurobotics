import React from 'react';
import ReactDOM from 'react-dom/client';
import ChatBotWrapper from '../components/ChatBot/ChatBotWrapper';

export default {
  onRouteDidUpdate() {
    const chatbotContainer = document.getElementById('docusaurus-chatbot-root');
    if (chatbotContainer && !chatbotContainer.hasChildNodes()) {
      ReactDOM.createRoot(chatbotContainer).render(<ChatBotWrapper />);
    }
  },
};
