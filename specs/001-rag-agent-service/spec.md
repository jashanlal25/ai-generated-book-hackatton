# Feature Specification: RAG Agent and Backend Service

**Feature Branch**: `001-rag-agent-service`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Spec-3: RAG Agent and backend service implementation

Goal:
Build a backend RAG service using FastAPI and the OpenAI Agents/ChatKit SDKs that can answer questions about the book by retrieving relevant content from Qdrant and optionally restricting answers to user-selected text.

Success criteria:

FastAPI service exposes a chat endpoint for the book.

OpenAI Agent is configured with retrieval tools backed by Qdrant.

Agent answers questions using retrieved book content only.

Supports "selected-text-only" mode where responses are constrained to user-provided text.

Conversation state and message history stored in Neon Serverless Postgres.

Constraints:

Agent framework: OpenAI Agents / ChatKit SDKs only.

Vector store: existing Qdrant Cloud collection from Spec-1.

Database: Neon Serverless Postgres.

Backend only; no frontend integration in this spec.

Output: working FastAPI backend + minimal documentation.

Not building:

Frontend chat UI or book embedding.

Website deployment or styling.

Re-embedding or ingestion logic.

Advanced analytics or monitoring.

Deliverables:

FastAPI backend with RAG chat endpoint.

OpenAI Agent configuration with retrieval tools.

Neon Postgres schema for conversations/messages.

Concise Markdown describing architecture and usage."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Question Answering (Priority: P1)

As a user, I want to ask questions about the book content and receive accurate answers based on the book's information, so that I can quickly find relevant information without reading the entire book.

**Why this priority**: This is the core functionality of the RAG system - enabling users to get answers from book content through natural language queries.

**Independent Test**: Can be fully tested by submitting a question to the chat endpoint and verifying that the response contains information from the book content, demonstrating the core RAG functionality.

**Acceptance Scenarios**:

1. **Given** a user has access to the chat endpoint, **When** they submit a question about book content, **Then** they receive an accurate response based on the book's information
2. **Given** the system has access to book content in Qdrant, **When** a relevant question is asked, **Then** the system retrieves relevant book passages and formulates an answer

---

### User Story 2 - Selected Text Constrained Responses (Priority: P2)

As a user, I want to provide specific text selections and receive responses that are constrained only to that text, so that I can get focused answers on particular content sections.

**Why this priority**: This provides an advanced feature that allows users to narrow down the scope of answers to specific content they're interested in.

**Independent Test**: Can be tested by providing user-selected text along with a question and verifying that the response is based only on the provided text rather than the entire book collection.

**Acceptance Scenarios**:

1. **Given** a user provides specific text content with their question, **When** they submit the query, **Then** the response is constrained to only the provided text content
2. **Given** a user enables "selected-text-only" mode, **When** they submit a question, **Then** the system ignores the general book content and only uses the selected text

---

### User Story 3 - Conversation History Management (Priority: P3)

As a user, I want my conversation history to be preserved across interactions, so that I can have contextual conversations with the system and reference previous exchanges.

**Why this priority**: This enhances the user experience by providing context-aware responses and maintaining conversation flow.

**Independent Test**: Can be tested by conducting a multi-turn conversation and verifying that the system remembers previous questions and responses in the conversation context.

**Acceptance Scenarios**:

1. **Given** a user has started a conversation, **When** they ask follow-up questions, **Then** the system maintains context from previous exchanges
2. **Given** a conversation session exists, **When** the user returns later, **Then** they can access their conversation history

---

### Edge Cases

- What happens when the system cannot find relevant information in the book content for a given question?
- How does the system handle very long user-provided text selections in "selected-text-only" mode?
- What occurs when Qdrant vector store is temporarily unavailable?
- How does the system respond to malicious or inappropriate questions?
- What happens when conversation history exceeds database storage limits?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a chat endpoint that accepts user questions about book content
- **FR-002**: System MUST integrate with Qdrant vector store to retrieve relevant book content for user questions
- **FR-003**: System MUST use OpenAI Agents/ChatKit SDKs to process questions and generate responses
- **FR-004**: System MUST ensure responses are based only on retrieved book content and not hallucinated information
- **FR-005**: System MUST support a "selected-text-only" mode where responses are constrained to user-provided text
- **FR-006**: System MUST store conversation history and message data in Neon Serverless Postgres database
- **FR-007**: System MUST maintain conversation context across multiple exchanges with the same user
- **FR-008**: System MUST validate that responses contain information sourced from the book content
- **FR-009**: System MUST handle error conditions gracefully when external services (Qdrant, OpenAI) are unavailable
- **FR-010**: System MUST support concurrent users without data cross-contamination between conversations

### Key Entities *(include if feature involves data)*

- **Conversation**: Represents a single user's interaction session with the system, containing metadata like creation time, user ID, and conversation status
- **Message**: Represents an individual exchange within a conversation, including the user's input, system's response, timestamp, and message type (user/assistant)
- **User**: Represents the person interacting with the system, with potential authentication and session management
- **BookContent**: Represents the source material from which answers are derived, stored in Qdrant vector store with metadata about source location

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can submit questions and receive relevant answers based on book content within 5 seconds
- **SC-002**: 90% of user questions receive responses that are directly sourced from book content without hallucination
- **SC-003**: System maintains conversation context across 10+ exchanges without degradation in response quality
- **SC-004**: The selected-text-only mode correctly constrains responses to user-provided text 95% of the time
- **SC-005**: System handles 100 concurrent users without performance degradation or data cross-contamination
- **SC-006**: 95% of conversations can be successfully retrieved from Neon Postgres database with complete message history
- **SC-007**: Users rate the relevance and accuracy of responses 4.0 or higher on a 5-point scale