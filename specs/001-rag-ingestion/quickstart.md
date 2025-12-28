# Quickstart: RAG Content Ingestion Pipeline

## System Architecture Overview

The pipeline follows a multi-stage architecture:
```
Docusaurus Website → URL Discovery → HTML Fetching → Text Extraction & Cleaning →
Text Processing & Chunking → Embedding Generation → Qdrant Storage
```

## Prerequisites

- Python 3.10+
- `uv` package manager
- Cohere API key
- Qdrant Cloud cluster URL and API key

## Setup

1. Create the backend directory:
```bash
mkdir backend
cd backend
```

2. Initialize Python project with uv:
```bash
uv init
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv tiktoken
```

3. Create environment file:
```bash
cp .env.example .env
```

4. Add your API keys to `.env`:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
SITEMAP_URL=https://ai-generated-book-hackatton-82u7.vercel.app/sitemap.xml
TARGET_URL=https://ai-generated-book-hackatton-82u7.vercel.app/
```

## Pipeline Stages

### Stage 1: URL Discovery
- Discovers all accessible pages from the Docusaurus website
- Uses both sitemap.xml and recursive crawling

### Stage 2: Text Extraction & Cleaning
- Fetches each page's HTML content
- Extracts clean text, removing navigation, scripts, and styling
- Preserves document structure and hierarchy

### Stage 3: Text Processing & Chunking
- Processes text into 800-1200 token chunks
- Implements overlap to maintain context
- Uses tiktoken for accurate token counting

### Stage 4: Embedding Generation
- Generates embeddings using Cohere's embedding model
- Processes in batches for efficiency

### Stage 5: Qdrant Storage
- Stores embeddings in "hackathon_aibook" collection
- Includes metadata (URL, title, chunk_id) with each vector
- Supports incremental updates via upsert operations

## Usage

Run the complete ingestion pipeline:
```bash
python main.py
```

## Configuration

The script can be configured via environment variables in the `.env` file:
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL of your Qdrant Cloud cluster
- `QDRANT_API_KEY`: API key for Qdrant Cloud access
- `TARGET_URL`: The Docusaurus site URL to crawl (defaults to https://ai-generated-book-hackatton-82u7.vercel.app/)
`SITEMAP_URL`=https://ai-generated-book-hackatton-82u7.vercel.app/sitemap.xml
- `CHUNK_SIZE`: Size of text chunks in tokens (defaults to 1000)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (defaults to 200)
- `BATCH_SIZE`: Number of chunks to process in each embedding batch (defaults to 10)