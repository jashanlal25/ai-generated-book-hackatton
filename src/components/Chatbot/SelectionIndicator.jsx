import React from 'react';

const SelectionIndicator = ({ selectedText, onClearSelection }) => {
  if (!selectedText) {
    return null;
  }

  return (
    <div className="selection-indicator" role="status" aria-live="polite">
      <div className="selected-text-preview">
        <strong>Selected:</strong> {selectedText.substring(0, 100)}...
        {selectedText.length > 100 && '...'}
      </div>
      <button
        onClick={onClearSelection}
        className="clear-selection-button"
        aria-label="Clear selected text"
      >
        Clear Selection
      </button>
    </div>
  );
};

export default SelectionIndicator;