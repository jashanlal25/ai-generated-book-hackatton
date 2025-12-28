# ADR-0002: RAG Chatbot Frontend Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** RAG Chatbot Frontend Integration
- **Context:** Need to integrate a RAG chatbot into the Docusaurus book to allow users to ask questions about book content with optional text selection constraints. The solution must maintain conversation context, handle selected text appropriately, and integrate seamlessly with the existing Docusaurus framework.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Frontend Framework**: React component integrated directly into Docusaurus
- **Backend Communication**: REST API calls to FastAPI /chat endpoint using standard HTTP requests
- **Session Management**: Client-side session tracking with backend coordination via session ID
- **Text Selection**: Browser Selection API to capture user-selected text from book content
- **Component Structure**: Modular React components (ChatbotContainer, MessageHistory, InputArea, SelectionIndicator)
- **API Format**: Request `{question: string, selectedText?: string, sessionId?: string}`, Response `{response: string, sessionId: string}`

## Consequences

### Positive

- Seamless integration with existing Docusaurus infrastructure without requiring separate UI framework
- Simple REST API communication pattern that is well-understood and debuggable
- Client-side session management allows for persistent conversation context without server storage requirements
- Browser Selection API provides reliable access to user-selected text across different browsers
- Modular component architecture promotes reusability and maintainability
- Standard API format allows for easy backend modifications if needed

### Negative

- Client-side session management may be lost on page refresh unless enhanced with localStorage
- REST API may have higher latency compared to WebSocket for real-time interactions
- Browser Selection API has some limitations in handling complex text selections or special formatting
- Tight coupling with Docusaurus may make future framework migrations more difficult
- Potential security concerns with exposing backend API endpoint directly from frontend

## Alternatives Considered

**Alternative A: Standalone Chat Widget with iFrame Integration**
- Approach: Host chatbot as separate React application embedded via iFrame
- Why rejected: Would create additional complexity, potential styling inconsistencies, and cross-origin communication challenges

**Alternative B: WebSocket-based Real-time Communication**
- Approach: Use WebSocket connection for real-time chat experience
- Why rejected: Unnecessary complexity for question-answer use case, additional server infrastructure needed, more complex error handling

**Alternative C: Server-Side Session Management with Database Storage**
- Approach: Store conversation sessions in a database with server-side session management
- Why rejected: Adds unnecessary backend complexity and infrastructure requirements for a client-side feature

**Alternative D: Alternative Frontend Framework (Vue, Angular)**
- Approach: Use different JavaScript framework for the chatbot component
- Why rejected: Would create framework inconsistency with existing Docusaurus (React-based) implementation

## References

- Feature Spec: ../specs/003-rag-chatbot-frontend/spec.md
- Implementation Plan: ../specs/003-rag-chatbot-frontend/plan.md
- Related ADRs: ADR-0001: RAG System Architecture
- Evaluator Evidence: ../history/prompts/003-rag-chatbot-frontend/0001-rag-chatbot-frontend-spec.spec.prompt.md
