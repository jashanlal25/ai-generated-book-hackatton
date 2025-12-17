/**
 * Type definitions for chatbot objects and API responses
 * (Using JSDoc-style comments for type documentation)
 */

/**
 * @typedef {Object} Message
 * @property {number} id - Unique identifier for the message
 * @property {string} text - The content of the message
 * @property {'user'|'ai'|'system'} sender - Who sent the message
 * @property {Date} timestamp - When the message was created
 */

/**
 * @typedef {Object} ChatRequest
 * @property {string} question - The user's question
 * @property {string} [sessionId] - Optional session identifier
 * @property {string} [selectedText] - Optional selected text to constrain the answer
 */

/**
 * @typedef {Object} ChatResponse
 * @property {string} response - The AI-generated response
 * @property {string} sessionId - The session identifier (for maintaining context)
 */

/**
 * @typedef {Object} SessionInfo
 * @property {string} sessionId - The unique session identifier
 * @property {Array<Message>} history - The conversation history
 * @property {Date} createdAt - When the session was created
 * @property {Date} lastActive - When the session was last used
 */

// Export empty object to make this a valid ES module
export {};