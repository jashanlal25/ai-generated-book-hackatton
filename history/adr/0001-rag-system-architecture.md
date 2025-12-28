# ADR-0001: RAG System Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** rag-agent-service
- **Context:** Need to build a backend RAG service that can answer questions about book content by retrieving relevant information from Qdrant and optionally restricting answers to user-selected text. The system must be scalable, maintainable, and provide accurate responses based only on book content without hallucination.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Backend Framework**: FastAPI (Python 3.10+) - for building the REST API with excellent async support and automatic API documentation
- **AI Agent Framework**: OpenAI Agents/ChatKit SDK - for processing natural language queries and generating responses
- **Vector Store**: Qdrant Cloud - for storing and retrieving book content embeddings with efficient similarity search
- **Database**: Neon Serverless Postgres - for storing conversation history and message data with automatic scaling
- **Retrieval Architecture**: Retrieval-Augmented Generation (RAG) pattern with optional constrained mode for user-selected text
- **API Design**: Single /chat endpoint supporting both standard RAG and selected-text-only modes

## Consequences

### Positive

- FastAPI provides excellent developer experience with automatic OpenAPI documentation and strong type validation
- OpenAI Agents/ChatKit SDK offers mature tools for RAG implementations with built-in retrieval capabilities
- Qdrant Cloud provides managed vector database with high-performance similarity search algorithms
- Neon Postgres Serverless offers automatic scaling and familiar SQL interface with minimal operational overhead
- Single endpoint design simplifies client integration while supporting both RAG modes
- Clear separation between retrieval and generation components enables independent optimization
- Built-in async support allows handling many concurrent requests efficiently

### Negative

- Dependency on external managed services (OpenAI, Qdrant Cloud, Neon) introduces potential vendor lock-in
- API costs for OpenAI and Qdrant may scale with usage, creating variable operational expenses
- Network latency between services may impact response times, especially under high load
- Limited control over vector store internals and retrieval algorithms compared to self-hosted solutions
- Potential data privacy concerns when sending book content to external AI services
- Reliance on multiple external APIs creates multiple failure points

## Alternatives Considered

Alternative Stack A: Flask + LangChain + Pinecone + PostgreSQL
- Why rejected: Less modern framework, more complex integration, higher operational overhead

Alternative Stack B: Node.js/Express + OpenAI API directly + Supabase Vector + Supabase DB
- Why rejected: Less suitable for ML/AI workloads, potentially slower for async operations compared to FastAPI

Alternative Stack C: Self-hosted solutions (FastAPI + pgvector + Hugging Face models)
- Why rejected: Higher operational complexity, requires managing model hosting and vector database infrastructure

## References

- Feature Spec: ../specs/001-rag-agent-service/spec.md
- Implementation Plan: ../specs/001-rag-agent-service/plan.md
- Related ADRs: none
- Evaluator Evidence: history/prompts/001-rag-agent-service/0001-create-rag-agent-service-spec.spec.prompt.md
