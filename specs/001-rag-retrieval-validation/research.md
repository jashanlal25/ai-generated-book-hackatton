# Research Summary: RAG Retrieval Pipeline Validation

## Decision: Identified Current Architecture
**Rationale**: Analysis of backend/main.py reveals the existing RAG system architecture with clear separation of concerns.
**Alternatives considered**:
- Building from scratch vs. extending existing pipeline
- Using different embedding models vs. Cohere embed-english-v3.0
- Different vector databases vs. Qdrant

## Decision: Qdrant Configuration Location
**Rationale**: Configuration is stored in environment variables as per backend/.env.example:
- QDRANT_URL="https://your-qdrant-cluster-url.cloud.qdrant.io:6333"
- QDRANT_API_KEY="your-qdrant-api-key-here"
- Collection name: "hackathon_aibook" (hardcoded in main.py:470, 538)

## Decision: Qdrant Collection Name
**Rationale**: The existing pipeline uses "hackathon_aibook" as the collection name in multiple places in main.py
**Alternatives considered**: Using a different collection name for validation vs. using the same one

## Decision: Cohere Embedding Model
**Rationale**: The existing pipeline uses "embed-english-v3.0" model with "search_document" input type as seen in main.py:393-394
**Alternatives considered**: Different Cohere models, other embedding providers

## Decision: Embedding Dimensions
**Rationale**: Cohere embeddings are 1024-dimensional vectors as validated in main.py:457
**Alternatives considered**: Different embedding dimensions/models

## Decision: Metadata Schema
**Rationale**: From main.py:519-534, the payload contains:
- chunk_id: Unique identifier for the chunk
- url: Source URL of the content
- title: Page title
- content: The actual text content
- chunk_index: Position of chunk in document
- document_id: ID of the source document
- created_at: Timestamp
- content_hash: For incremental updates

## Decision: Test Query Strategy
**Rationale**: Test queries should be related to the book content about Physical AI & Humanoid Robotics as specified in the constitution
**Alternatives considered**: Generic queries vs. domain-specific queries

## Decision: Validation Approach
**Rationale**: Need to validate both semantic similarity retrieval and metadata accuracy as per spec requirements
**Alternatives considered**: Only content validation vs. comprehensive validation including metadata