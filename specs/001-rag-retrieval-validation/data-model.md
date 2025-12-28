# Data Model: RAG Retrieval Pipeline Validation

## Core Entities

### RetrievedChunk
- **chunk_id**: Unique identifier for the chunk (string)
- **url**: Source URL of the content (string, required)
- **title**: Page title (string, required)
- **content**: The actual text content (string, required)
- **chunk_index**: Position of chunk in document (integer)
- **document_id**: ID of the source document (string)
- **similarity_score**: Semantic similarity score (float, 0.0-1.0)
- **retrieval_rank**: Rank in retrieval results (integer)

### ValidationResult
- **query**: The test query used (string, required)
- **retrieved_chunks**: List of RetrievedChunk objects (array, required)
- **total_chunks_found**: Number of chunks returned (integer)
- **metadata_accuracy**: Percentage of accurate metadata (float, 0.0-100.0)
- **content_relevance**: Percentage of relevant content (float, 0.0-100.0)
- **query_successful**: Whether the query executed without errors (boolean)
- **execution_time**: Time taken to execute query (float, seconds)

### ValidationReport
- **validation_date**: When validation was performed (timestamp)
- **total_queries_executed**: Number of test queries run (integer)
- **success_rate**: Percentage of successful queries (float, 0.0-100.0)
- **average_relevance**: Average relevance score across all queries (float, 0.0-100.0)
- **metadata_accuracy**: Overall metadata accuracy (float, 0.0-100.0)
- **individual_results**: Array of ValidationResult objects (array)
- **status**: Overall validation status (string: "pass", "fail", "partial")