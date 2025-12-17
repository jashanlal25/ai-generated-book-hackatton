import React, { useState } from 'react';

const InputArea = ({ onSendMessage, isLoading, selectedText }) => {
  const [inputValue, setInputValue] = useState('');
  const [selectedTextOnly, setSelectedTextOnly] = useState(false);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue, selectedTextOnly && selectedText);
      setInputValue('');
    }
  };

  return (
    <form className="input-area" onSubmit={handleSubmit} role="form">
      {selectedText && (
        <div className="selected-text-indicator" role="region" aria-labelledby="selection-label">
          <div id="selection-label">
            <strong>Selected Text Mode:</strong> {selectedText.substring(0, 50)}...
          </div>
          <label htmlFor="selected-text-only">
            <input
              id="selected-text-only"
              type="checkbox"
              checked={selectedTextOnly}
              onChange={(e) => setSelectedTextOnly(e.target.checked)}
            />
            Answer only from selected text
          </label>
        </div>
      )}
      <div className="input-container">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the book..."
          disabled={isLoading}
          className="chat-input"
          aria-label="Type your question here"
          autoComplete="off"
        />
        <button
          type="submit"
          disabled={isLoading || !inputValue.trim()}
          className="send-button"
          aria-label={isLoading ? "Sending message" : "Send message"}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </form>
  );
};

export default InputArea;