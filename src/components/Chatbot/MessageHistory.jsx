import React from 'react';

const MessageHistory = ({ messages }) => {
  return (
    <div className="message-history" aria-live="polite" aria-label="Chat messages">
      {messages.map((message) => (
        <div
          key={message.id}
          className={`message ${message.sender}`}
          role="log"
          aria-label={`${message.sender} message`}
        >
          <div className="message-text">{message.text}</div>
          <div className="message-timestamp" aria-label="Message time">
            {message.timestamp.toLocaleTimeString()}
          </div>
        </div>
      ))}
      {messages.length === 0 && (
        <div className="welcome-message" role="status" aria-live="polite">
          Ask me anything about the book! Select text to constrain answers to that content.
        </div>
      )}
    </div>
  );
};

export default MessageHistory;