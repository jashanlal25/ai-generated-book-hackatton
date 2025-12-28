# Data Model: RAG Content Ingestion Pipeline

## System Data Flow

### Stage 1: Raw HTML Pages
- **source_url**: string (URL of the Docusaurus page)
- **raw_html**: string (unprocessed HTML content)
- **crawl_timestamp**: datetime (when the page was crawled)

### Stage 2: Text Extraction & Cleaning
- **url**: string (source URL of the page)
- **title**: string (page title extracted from HTML)
- **clean_content**: string (text content with HTML, navigation, scripts, and styling removed)
- **extracted_at**: datetime (timestamp of extraction)
- **processing_metadata**: object (info about cleaning process)

### Stage 3: Text Processing & Chunking
- **document_id**: string (identifier for the original document)
- **url**: string (source URL)
- **title**: string (document title)
- **chunk_id**: string (unique identifier for this chunk)
- **chunk_text**: string (the specific text chunk content)
- **chunk_index**: integer (position of chunk in original document)
- **token_count**: integer (number of tokens in this chunk)
- **processed_at**: datetime (timestamp of chunking)

### Stage 4: Embedding Generation
- **chunk_id**: string (unique identifier for this chunk)
- **document_id**: string (identifier for the original document)
- **url**: string (source URL)
- **title**: string (document title)
- **chunk_text**: string (the specific text chunk that was embedded)
- **chunk_index**: integer (position of chunk in original document)
- **embedding_vector**: list[float] (embedding vector from Cohere)
- **embedded_at**: datetime (timestamp of embedding)

### Stage 5: Qdrant Storage
- **point_id**: string (Qdrant point ID - auto-generated or provided)
- **vector**: list[float] (embedding vector from Cohere)
- **payload**: object (metadata stored with the vector)
  - **chunk_id**: string (unique identifier for this chunk)
  - **url**: string (source URL of the original page)
  - **title**: string (title of the original page)
  - **content**: string (the text of this specific chunk)
  - **chunk_index**: integer (position of this chunk in the original document)
  - **document_id**: string (identifier for the original document)
  - **created_at**: datetime (timestamp of when this record was created)
- **stored_at**: datetime (timestamp of storage in Qdrant)

## Architecture Relationships

- One Raw HTML Page → One Clean Content → Many Chunks → Many Embeddings → Many Qdrant Records
- Each stage transforms data for the next stage in the pipeline

## Validation Rules

- URL must be a valid HTTP/HTTPS URL
- Clean content must not be empty after HTML removal
- Chunk size must be between 800-1200 tokens
- Chunk overlap should not exceed 25% of chunk size
- Embedding vector must have the correct dimensionality for the Cohere model
- Qdrant payload must include required metadata fields (url, title, content)

## Processing State Transitions

- Raw HTML: CRAWLED → EXTRACTED → CHUNKED → EMBEDDED → STORED
- Each stage has success/failure states for error handling and retry logic

