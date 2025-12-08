import React, { useState, useEffect } from 'react';

interface TextSelectionProps {
  onAskAI: (selectedText: string, action: 'ask' | 'summary') => void;
}

const TextSelection: React.FC<TextSelectionProps> = ({ onAskAI }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showPopup, setShowPopup] = useState(false);
  const [popupPosition, setPopupPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      
      if (text && text.length > 10) {
        setSelectedText(text);
        
        // Get selection position
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();
        
        if (rect) {
          setPopupPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
          });
          setShowPopup(true);
        }
      } else {
        setShowPopup(false);
      }
    };

    const handleClickOutside = () => {
      setShowPopup(false);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  if (!showPopup) return null;

  return (
    <div
      style={{
        position: 'fixed',
        left: popupPosition.x,
        top: popupPosition.y,
        transform: 'translate(-50%, -100%)',
        background: 'white',
        border: '1px solid #e5e7eb',
        borderRadius: '8px',
        boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        padding: '0.5rem',
        display: 'flex',
        gap: '0.5rem',
        zIndex: 10000,
        animation: 'fadeIn 0.2s ease',
      }}
    >
      <button
        onClick={() => {
          onAskAI(selectedText, 'ask');
          setShowPopup(false);
        }}
        style={{
          background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
          color: 'white',
          border: 'none',
          padding: '0.5rem 0.75rem',
          borderRadius: '4px',
          fontSize: '0.75rem',
          cursor: 'pointer',
          whiteSpace: 'nowrap',
        }}
      >
        🤖 Ask AI
      </button>
      
      <button
        onClick={() => {
          onAskAI(selectedText, 'summary');
          setShowPopup(false);
        }}
        style={{
          background: '#f3f4f6',
          color: '#374151',
          border: '1px solid #d1d5db',
          padding: '0.5rem 0.75rem',
          borderRadius: '4px',
          fontSize: '0.75rem',
          cursor: 'pointer',
          whiteSpace: 'nowrap',
        }}
      >
        📋 Summarize
      </button>
    </div>
  );
};

export default TextSelection;