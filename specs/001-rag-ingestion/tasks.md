# Tasks: RAG Content Ingestion Pipeline

**Feature**: RAG Content Ingestion Pipeline
**Created**: 2025-12-13
**Status**: Draft
**Input**: Feature specification from `/specs/001-rag-ingestion/spec.md`

## Implementation Strategy

This feature will be implemented using a phased approach following the multi-stage pipeline architecture:
1. **Phase 1**: Setup - Project structure and dependencies
2. **Phase 2**: Foundational - Core utilities and configuration
3. **Phase 3**: User Story 1 - Content Indexing (P1 priority)
4. **Phase 4**: User Story 2 - Embedding Generation (P1 priority)
5. **Phase 5**: User Story 3 - Vector Storage and Management (P1 priority)
6. **Phase 6**: User Story 4 - Reproducible Pipeline Execution (P2 priority)
7. **Phase 7**: Polish & Cross-cutting concerns

**MVP Scope**: Phases 1-3 (Content Indexing) will deliver a working system that can crawl and extract content from Docusaurus sites.

## Phase 1: Setup
**Goal**: Create project structure and initialize dependencies

- [x] T001 Create backend directory structure at backend/
- [x] T002 Initialize Python project with uv in backend/ directory
- [x] T003 Create requirements.txt with dependencies: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, tiktoken
- [x] T004 Create .env.example with template for API keys and configuration
- [x] T005 [P] Install all required dependencies using uv

## Phase 2: Foundational Components
**Goal**: Implement core utilities and configuration needed by all user stories

- [x] T006 Create main.py file with proper imports and structure
- [x] T007 Implement configuration loading from environment variables
- [x] T008 [P] Create utility function for token counting using tiktoken
- [x] T009 [P] Create logging setup with appropriate levels and formatting
- [x] T010 [P] Implement retry mechanism for network requests with exponential backoff

## Phase 3: User Story 1 - Content Indexing (P1)
**Goal**: As a content administrator, I want to automatically crawl and index all pages from my Docusaurus documentation site so that the content becomes searchable through a RAG system.

**Independent Test**: Can be fully tested by running the ingestion pipeline against a sample Docusaurus site and verifying that all pages are successfully crawled and stored in the vector database.

- [x] T011 [US1] Implement get_all_urls function to discover Docusaurus site URLs from sitemap
- [x] T012 [US1] Implement recursive crawling fallback if sitemap is not available
- [x] T013 [US1] Implement extract_text_from_urls function to fetch and parse HTML
- [x] T014 [US1] Implement HTML cleaning to remove navigation, scripts, and styling using BeautifulSoup
- [x] T015 [US1] Extract page titles and content with proper metadata
- [x] T016 [US1] Add error handling for network timeouts and blocked pages
- [x] T017 [US1] Test crawling functionality with the target Docusaurus site

## Phase 4: User Story 2 - Embedding Generation (P1)
**Goal**: As a system administrator, I want to generate high-quality embeddings for indexed content so that semantic search and retrieval can be performed effectively.

**Independent Test**: Can be fully tested by running the embedding generation process and verifying that vectors are created with appropriate dimensionality and quality.

- [x] T018 [US2] Implement embed function to generate Cohere embeddings for text chunks
- [x] T019 [US2] Set up Cohere API client with proper authentication
- [x] T020 [US2] Implement batch processing for efficient embedding generation
- [x] T021 [US2] Add embedding validation to ensure quality and correct dimensions
- [x] T022 [US2] Handle rate limiting and API errors gracefully
- [x] T023 [US2] Test embedding generation with sample text chunks

## Phase 5: User Story 3 - Vector Storage and Management (P1)
**Goal**: As a developer, I want to store embeddings in a vector database with proper metadata so that content can be efficiently retrieved later.

**Independent Test**: Can be fully tested by storing vectors in Qdrant Cloud and verifying that they can be queried and retrieved with appropriate metadata.

- [x] T024 [US3] Implement Qdrant client setup with proper authentication
- [x] T025 [US3] Create collection named "hackathon_aibook" with appropriate schema
- [x] T026 [US3] Implement save_chunk_to_qdrant function with upsert functionality
- [x] T027 [US3] Format metadata payload with chunk_id, url, title, content, and chunk_index
- [x] T028 [US3] Implement vector storage with proper error handling
- [x] T029 [US3] Test storage functionality with sample embeddings and metadata

## Phase 6: User Story 4 - Reproducible Pipeline Execution (P2)
**Goal**: As an operations engineer, I want to run the ingestion pipeline reproducibly so that I can update the vector store as content changes.

**Independent Test**: Can be fully tested by running the complete pipeline multiple times and verifying consistent results and proper incremental updates.

- [x] T030 [US4] Implement main function orchestrating the complete pipeline
- [x] T031 [US4] Add comprehensive logging for indexing summary (documents, tokens, vector count)
- [x] T032 [US4] Implement incremental update logic to process only changed content
- [x] T033 [US4] Add pipeline progress tracking and status reporting
- [x] T034 [US4] Test full pipeline execution with the target Docusaurus site
- [x] T035 [US4] Validate upsert behavior for repeated runs

## Phase 7: Polish & Cross-cutting Concerns
**Goal**: Complete the implementation with proper error handling, validation, and documentation

- [x] T036 Add comprehensive error handling and graceful degradation across all functions
- [x] T037 Implement content quality validation before embedding generation
- [x] T038 Add validation for URL format and content requirements
- [x] T039 [P] Create comprehensive README with setup and usage instructions
- [x] T040 [P] Add unit tests for critical functions
- [x] T041 [P] Perform end-to-end testing of the complete pipeline
- [x] T042 [P] Optimize performance based on requirements (process 95% of pages within 30 minutes)

## Dependencies

**User Story 1 Dependencies**: Phase 1 (Setup), Phase 2 (Foundational)
**User Story 2 Dependencies**: Phase 1 (Setup), Phase 2 (Foundational), User Story 1 (Content extraction)
**User Story 3 Dependencies**: Phase 1 (Setup), Phase 2 (Foundational), User Story 2 (Embeddings)
**User Story 4 Dependencies**: All previous phases

## Parallel Execution Examples

**Within User Story 1**:
- T011 and T012 can run in parallel [P] - sitemap discovery and recursive crawling
- T013 and T014 can run in parallel [P] - fetching and cleaning different URLs

**Within User Story 2**:
- T018 and T019 can run in parallel [P] - embedding function and API setup

**Within User Story 3**:
- T024 and T025 can run in parallel [P] - client setup and collection creation

**Within Polish phase**:
- T039, T040, T041, T042 can all run in parallel [P] - documentation, testing, optimization