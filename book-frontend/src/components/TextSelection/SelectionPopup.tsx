import React, { useEffect, useRef } from 'react';

interface SelectionPopupProps {
  x: number;
  y: number;
  onAskAI: () => void;
  onSummary: () => void;
  onClose: () => void;
}

const SelectionPopup: React.FC<SelectionPopupProps> = ({
  x,
  y,
  onAskAI,
  onSummary,
  onClose
}) => {
  const popupRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const popup = popupRef.current;
    if (!popup) return;

    // Adjust position to keep popup within viewport
    const rect = popup.getBoundingClientRect();
    const viewportWidth = window.innerWidth;
    const viewportHeight = window.innerHeight;

    let adjustedX = x;
    let adjustedY = y;

    // Adjust horizontal position
    if (x + rect.width / 2 > viewportWidth - 20) {
      adjustedX = viewportWidth - rect.width / 2 - 20;
    } else if (x - rect.width / 2 < 20) {
      adjustedX = rect.width / 2 + 20;
    }

    // Adjust vertical position
    if (y < rect.height + 20) {
      adjustedY = y + 40; // Position below selection instead
    }

    popup.style.left = `${adjustedX - rect.width / 2}px`;
    popup.style.top = `${adjustedY}px`;
  }, [x, y]);

  return (
    <div
      ref={popupRef}
      className="text-selection-popup"
      style={{
        position: 'fixed',
        left: `${x}px`,
        top: `${y}px`,
        zIndex: 9999,
      }}
    >
      <div className="popup-content">
        <button
          className="popup-button ask-button"
          onClick={onAskAI}
          title="Ask Neurobotics AI about this text"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <circle cx="12" cy="12" r="3"/>
            <path d="M12 1v6m0 6v6m11-7h-6m-6 0H1"/>
          </svg>
          Ask Neurobotics
        </button>
        
        <button
          className="popup-button summary-button"
          onClick={onSummary}
          title="Get a summary of this text"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/>
            <polyline points="14,2 14,8 20,8"/>
            <line x1="16" y1="13" x2="8" y2="13"/>
            <line x1="16" y1="17" x2="8" y2="17"/>
            <polyline points="10,9 9,9 8,9"/>
          </svg>
          Summary
        </button>
      </div>
      
      <div className="popup-arrow"></div>
    </div>
  );
};

export default SelectionPopup;