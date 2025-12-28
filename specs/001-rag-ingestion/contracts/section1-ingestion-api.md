# Contract: Ingestion Pipeline API

## Overview
This contract defines the interface for the RAG content ingestion pipeline. The system implements a multi-stage pipeline architecture with distinct functions for each processing stage: URL discovery, text extraction & cleaning, text processing & chunking, embedding generation, and vector storage.

## System Architecture Interface

### Stage 1: URL Discovery
### `get_all_urls(target_url: str) -> List[str]`
**Purpose**: Discover all URLs from the deployed Docusaurus site using sitemap and recursive crawling
**Input**:
- `target_url`: The base URL of the Docusaurus site to crawl
**Output**: List of all discovered URLs
**Error cases**: Network errors, invalid URL format, empty sitemap, robots.txt restrictions
**Performance**: Should complete within 5 minutes for sites with <1000 pages

### Stage 2: Text Extraction & Cleaning
### `extract_text_from_urls(urls: List[str]) -> List[dict]`
**Purpose**: Fetch each URL and extract clean text content, removing HTML, navigation, scripts, and styling
**Input**:
- `urls`: List of URLs to extract content from
**Output**: List of dictionaries containing {url, title, clean_content, extracted_at}
**Error cases**: Network timeouts, invalid HTML, blocked by robots.txt, empty content after cleaning
**Performance**: Should handle 100 pages within 10 minutes

### Stage 3: Text Processing & Chunking
### `chunk_text(content_list: List[dict], chunk_size: int = 1000, overlap: int = 200) -> List[dict]`
**Purpose**: Process extracted text into 800-1200 token chunks with overlap, using tiktoken for accurate token counting
**Input**:
- `content_list`: List of content dictionaries from extraction step
- `chunk_size`: Target size for chunks in tokens (default 1000)
- `overlap`: Overlap between chunks in tokens (default 200)
**Output**: List of chunk dictionaries with metadata {document_id, url, title, chunk_id, chunk_text, chunk_index, token_count}
**Error cases**: Empty content, invalid chunk parameters, token counting errors
**Performance**: Should process 1000 chunks within 2 minutes

### Stage 4: Embedding Generation
### `embed(text_chunks: List[dict]) -> List[dict]`
**Purpose**: Generate embeddings using Cohere's embedding model with batch processing
**Input**:
- `text_chunks`: List of text chunks to embed
**Output**: List of dictionaries containing original chunk data plus embedding_vector
**Error cases**: Invalid API key, rate limiting, network errors, invalid text for embedding
**Performance**: Should process 100 chunks within 5 minutes (with batching)

### Stage 5: Vector Storage
### `create_collection(collection_name: str = "hackathon_aibook") -> bool`
**Purpose**: Create a Qdrant collection with appropriate schema for storing embeddings and metadata
**Input**:
- `collection_name`: Name for the Qdrant collection (default "hackathon_aibook")
**Output**: Boolean indicating success
**Error cases**: Invalid collection name, insufficient permissions, Qdrant connection issues
**Performance**: Should complete within 30 seconds

### `save_chunk_to_qdrant(chunk_data: dict) -> bool`
**Purpose**: Store a single chunk with its embedding in Qdrant Cloud with upsert functionality
**Input**:
- `chunk_data`: Dictionary containing chunk text, embedding, and metadata {chunk_id, url, title, content, chunk_index, document_id}
**Output**: Boolean indicating successful storage
**Error cases**: Invalid vector format, network errors, rate limiting, Qdrant connection issues
**Performance**: Should complete within 2 seconds per chunk

### Pipeline Execution
### `main() -> None`
**Purpose**: Execute the complete multi-stage ingestion pipeline from URL discovery to Qdrant storage
**Input**: None (reads configuration from environment variables)
**Output**: None (writes results to Qdrant and logs progress for each stage)
**Error cases**: Any of the above functions failing, with appropriate error handling and logging
**Performance**: Should complete full pipeline within 30 minutes for typical site

## Data Flow Contract
Each stage transforms data according to the data model:
- Stage 1: Raw HTML Pages → Stage 2: Clean Content → Stage 3: Chunks → Stage 4: Embeddings → Stage 5: Qdrant Records
- Each transformation maintains data integrity and includes appropriate metadata
- Error handling at each stage with retry logic where appropriate