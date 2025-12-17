import React, { useState, useEffect } from 'react';
import MessageHistory from './MessageHistory';
import InputArea from './InputArea';
import SelectionIndicator from './SelectionIndicator';
import ChatbotAPI from '../../services/chatbot-api';
import './chatbot.css';

const ChatbotContainer = () => {
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Add a message to the history
  const addMessage = (message) => {
    setMessages(prev => [...prev, message]);
  };

  // Initialize session
  useEffect(() => {
    const initializeSession = async () => {
      // Try to get existing session from sessionStorage
      let id = sessionStorage.getItem('chatbot-session-id');

      if (!id) {
        // Create a new session if none exists
        try {
          id = await ChatbotAPI.createSession();
        } catch (err) {
          // Fallback to generating a local session ID if API fails
          id = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
        }
      }

      setSessionId(id);
      sessionStorage.setItem('chatbot-session-id', id);

      // Load existing conversation history if available
      try {
        const history = await ChatbotAPI.getConversationHistory(id);
        if (history && history.length > 0) {
          setMessages(history);
        }
      } catch (err) {
        // If history fetch fails, continue with empty messages
        console.warn('Could not load conversation history:', err.message);
      }
    };

    initializeSession();
  }, []);

  const handleSendMessage = async (question, selectedTextOnly = false) => {
    if (!question.trim()) return;

    // Add user message to history
    const userMessage = {
      id: Date.now(),
      text: question,
      sender: 'user',
      timestamp: new Date()
    };

    addMessage(userMessage);
    setIsLoading(true);
    setError(null);

    try {
      // Limit the selected text length to prevent performance issues
      const processedSelectedText = selectedText && selectedTextOnly
        ? selectedText.substring(0, 2000) // Limit to 2000 characters
        : undefined;

      // Use the API service to communicate with FastAPI backend
      const data = await ChatbotAPI.sendQuestion(
        question,
        sessionId,
        processedSelectedText
      );

      // Add AI response to history
      const aiMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'ai',
        timestamp: new Date()
      };

      addMessage(aiMessage);

      // Update session ID if provided by backend
      if (data.sessionId) {
        setSessionId(data.sessionId);
        sessionStorage.setItem('chatbot-session-id', data.sessionId);
      }
    } catch (err) {
      setError(err.message);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'system',
        timestamp: new Date()
      };
      addMessage(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to capture text selection using the browser Selection API
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection.rangeCount > 0) {
        const selectedText = selection.toString().trim();
        if (selectedText) {
          setSelectedText(selectedText);
        }
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  return (
    <div className="chatbot-container" role="region" aria-label="Chatbot interface">
      {isLoading && (
        <div className="loading-indicator" role="status" aria-live="polite">
          <div className="loading-spinner" aria-hidden="true"></div>
          <span>Processing your question...</span>
        </div>
      )}
      <SelectionIndicator
        selectedText={selectedText}
        onClearSelection={() => setSelectedText('')}
      />
      <MessageHistory messages={messages} />
      <InputArea
        onSendMessage={handleSendMessage}
        isLoading={isLoading}
        selectedText={selectedText}
      />
      {error && (
        <div className="error-message" role="alert" aria-live="assertive">
          <strong>Error:</strong> {error}
        </div>
      )}
    </div>
  );
};

export default ChatbotContainer;