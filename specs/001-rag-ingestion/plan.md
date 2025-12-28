# Implementation Plan: RAG Content Ingestion Pipeline

**Branch**: `001-rag-ingestion` | **Date**: 2025-12-13 | **Spec**: [link to spec](spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an end-to-end ingestion pipeline that crawls the deployed Docusaurus book URLs (https://ai-generated-book-hackatton-82u7.vercel.app/), extracts page-level clean text, generates embeddings using Cohere's embedding models, and stores the resulting vectors and metadata inside a Qdrant Cloud collection. The system will be implemented as a single Python script (main.py) with functions for URL discovery, text extraction, content chunking, embedding generation, and vector storage.

## System Architecture

### High-Level Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Docusaurus     │───▶│  Text Processing │───▶│  Embedding      │
│  Website        │    │  Pipeline        │    │  Generation     │
│ (Source)        │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌──────────────────┐    ┌─────────────────┐
                    │  Chunking        │───▶│  Qdrant         │
                    │  Engine          │    │  Storage        │
                    │                  │    │                 │
                    └──────────────────┘    └─────────────────┘
```

### Component Flow
1. **URL Discovery**: Crawl Docusaurus website to identify all accessible pages
2. **Text Extraction & Cleaning**: Extract clean text content, removing HTML, navigation, scripts, and styling
3. **Text Processing & Chunking**: Process text into 800-1200 token chunks with overlap
4. **Embedding Generation**: Generate embeddings using Cohere's embedding model
5. **Vector Storage**: Store embeddings and metadata in Qdrant Cloud collection named "hackathon_aibook"

### Data Flow Architecture
```
Raw HTML Pages → Clean Text → Chunks → Embeddings → Qdrant Payloads
     ↓              ↓           ↓          ↓             ↓
   Requests    BeautifulSoup  Recursive  Cohere API   Upsert API
   Parsing        Cleaning    Splitting   Vectors      Storage
```

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, tiktoken
**Storage**: Qdrant Cloud (vector database)
**Testing**: N/A (single script implementation)
**Target Platform**: Linux server environment
**Project Type**: Single script backend tool
**Performance Goals**: Process 95% of public pages within 30 minutes
**Constraints**: Must support incremental updates (upsert), use only Cohere for embeddings, and Qdrant Cloud for storage
**Scale/Scope**: Handle multiple documents with 800-1200 token chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation aligns with the project constitution by:
- Using minimal dependencies to reduce complexity
- Implementing clean, maintainable code
- Following security best practices (using environment variables for API keys)
- Maintaining proper error handling and logging
- Supporting incremental updates to avoid unnecessary processing

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Single script implementation with all functions
├── requirements.txt     # Python dependencies
└── .env.example         # Example environment variables file
```

**Structure Decision**: Single script implementation chosen for simplicity and ease of deployment. The main.py file will contain all required functions: get_all_urls, extract_text_from_urls, chunk_text, embed, create_collection named hackathon_aibook, save_chunk_to_qdrant, and execute in main function.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |