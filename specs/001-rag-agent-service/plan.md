# Implementation Plan: RAG Agent and Backend Service

## Technical Context

This plan outlines the implementation of a backend RAG service using FastAPI and OpenAI Agents/ChatKit SDKs that can answer questions about book content by retrieving relevant information from Qdrant and optionally restricting answers to user-selected text.

### Architecture Overview

- **Backend Framework**: FastAPI for the web service
- **AI Agent**: OpenAI Agents/ChatKit SDK for processing questions
- **Vector Store**: Qdrant Cloud collection for book content retrieval
- **Database**: Neon Serverless Postgres for conversation storage
- **Mode Support**: Standard RAG mode and "selected-text-only" mode

### Dependencies

- FastAPI
- OpenAI Agents/ChatKit SDK
- Qdrant client library
- Neon Postgres connector
- Python 3.10+

## Constitution Check

- [ ] Performance: API responses under 5 seconds (from spec: SC-001)
- [ ] Accuracy: 90% of responses from book content (from spec: SC-002)
- [ ] Concurrency: Support 100 concurrent users (from spec: SC-005)
- [ ] Data Integrity: 95% conversation retrieval success (from spec: SC-006)
- [ ] Security: Proper API key management for external services
- [ ] Maintainability: Clear separation of concerns between components

## Phase 0: Research & Analysis

### Research Tasks

1. **FastAPI with OpenAI Integration**
   - Best practices for async API endpoints
   - Error handling for external API calls
   - Rate limiting and request management

2. **Qdrant Vector Search Configuration**
   - Similarity search algorithms
   - Query optimization for book content
   - Performance tuning parameters

3. **OpenAI Agent Configuration**
   - Retrieval tool integration with Qdrant
   - System prompt engineering for book content
   - Response validation to prevent hallucination

4. **PostgreSQL Schema Design**
   - Conversation and message structure
   - Indexing strategies for performance
   - Data retention policies

## Phase 1: Design & Contracts

### Data Model

- **Conversation**:
  - id (UUID): Unique identifier
  - created_at (timestamp): When conversation started
  - updated_at (timestamp): Last activity
  - metadata (JSON): Additional conversation data

- **Message**:
  - id (UUID): Unique identifier
  - conversation_id (UUID): Reference to parent conversation
  - role (string): "user" or "assistant"
  - content (text): Message content
  - timestamp (timestamp): When message was created
  - metadata (JSON): Additional message data

### API Contracts

- **POST /chat**
  - Request: `{question: string, selected_text?: string, conversation_id?: string}`
  - Response: `{response: string, conversation_id: string, sources: array}`
  - Error: `{error: string, code: number}`

### Quickstart Guide

1. Set up environment variables for OpenAI, Qdrant, and Neon Postgres
2. Run database migrations
3. Start the FastAPI server
4. Send requests to the /chat endpoint

## Phase 2: Implementation Tasks

1. Initialize FastAPI application
2. Configure environment variables and settings
3. Implement OpenAI Agent integration
4. Connect to Qdrant vector store
5. Design and implement Postgres schema
6. Create chat endpoint with retrieval functionality
7. Add "selected-text-only" mode support
8. Implement conversation persistence
9. Add error handling and validation
10. Write tests for core functionality
11. Document API endpoints
12. Performance testing and optimization

## Risk Assessment

- **External Service Reliability**: Dependence on OpenAI and Qdrant APIs
- **Data Privacy**: Book content handling and user queries
- **Performance**: Response time requirements under load
- **Cost Management**: API usage costs for OpenAI and Qdrant