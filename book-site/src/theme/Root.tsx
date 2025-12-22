import React from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

// Root wrapper for Docusaurus theme
// This adds the chatbot widget to every page
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
