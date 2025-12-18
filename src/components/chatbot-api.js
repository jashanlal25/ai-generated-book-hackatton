/**
 * API service for chatbot communication with FastAPI backend
 */

// const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';  // Local development
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'https://jashanlal-rag-chat-boy.hf.space';  // Production (HF Space)

// Rate limiting configuration
const RATE_LIMIT_CONFIG = {
  maxRequests: 10,        // Maximum requests
  timeWindow: 60000,      // Time window in milliseconds (1 minute)
};

// Track request timestamps for rate limiting
let requestTimestamps = [];

class ChatbotAPI {
  /**
   * Check if the client is within the rate limit
   * @returns {boolean} True if within rate limit, false otherwise
   */
  static isWithinRateLimit() {
    const now = Date.now();
    // Remove timestamps outside the time window
    requestTimestamps = requestTimestamps.filter(timestamp =>
      now - timestamp < RATE_LIMIT_CONFIG.timeWindow
    );

    // Check if we're under the limit
    if (requestTimestamps.length < RATE_LIMIT_CONFIG.maxRequests) {
      // Add current timestamp
      requestTimestamps.push(now);
      return true;
    }

    return false;
  }

  /**
   * Send a question to the FastAPI /chat endpoint
   * @param {string} question - The user's question
   * @param {string} [sessionId] - Optional session identifier
   * @param {string} [selectedText] - Optional selected text to constrain the answer
   * @returns {Promise<{response: string, sessionId: string}>} The AI response and session ID
   */
  static async sendQuestion(question, sessionId, selectedText) {
    // Check rate limit
    if (!this.isWithinRateLimit()) {
      throw new Error(`Rate limit exceeded. Maximum ${RATE_LIMIT_CONFIG.maxRequests} requests per minute.`);
    }

    const payload = {
      question: question,
      ...(sessionId && { conversation_id: sessionId }),
      ...(selectedText && { selected_text: selectedText })
    };

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      // Map backend response to expected frontend format
      return {
        response: data.response,
        sessionId: data.conversation_id,
        sources: data.sources
      };
    } catch (error) {
      console.error('Error communicating with FastAPI /chat endpoint:', error);
      throw error;
    }
  }

  /**
   * Initialize a new chat session
   * @returns {Promise<string>} A new session ID
   */
  static async createSession() {
    try {
      const response = await fetch(`${API_BASE_URL}/session`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.sessionId;
    } catch (error) {
      console.error('Error creating chat session:', error);
      throw error;
    }
  }

  /**
   * Get conversation history for a session
   * @param {string} sessionId - The session identifier
   * @returns {Promise<Array>} Array of conversation messages
   */
  static async getConversationHistory(sessionId) {
    try {
      const response = await fetch(`${API_BASE_URL}/session/${sessionId}/history`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.history || [];
    } catch (error) {
      console.error('Error fetching conversation history:', error);
      throw error;
    }
  }
}

export default ChatbotAPI;