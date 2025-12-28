# Validation Script Contract

## Purpose
This contract defines the interface and behavior for the RAG retrieval validation script.

## Interface Definition

### Script Entry Point
- **Command**: `python validate_retrieval.py [options]`
- **Options**:
  - `--queries-file`: Path to file containing test queries (optional, defaults to built-in queries)
  - `--top-k`: Number of results to retrieve per query (optional, defaults to 5)
  - `--output`: Output format (markdown, json, csv - defaults to markdown)
  - `--output-file`: Path to save validation report (optional, defaults to stdout)

### Input Format
- **Test Queries**: A list of semantic search queries related to Physical AI & Humanoid Robotics content
- **Validation Parameters**:
  - top_k: Number of chunks to retrieve per query (typically 3-10)
  - relevance_threshold: Minimum similarity score for relevance (typically 0.7)

### Output Format
- **ValidationReport** in specified format (Markdown by default)
- Contains query results, metadata validation, and overall metrics

## Behavior Specification

### Success Path
1. Connect to Qdrant using environment configuration
2. Execute each test query against the "hackathon_aibook" collection
3. Retrieve top-k most similar chunks for each query
4. Validate metadata accuracy (URL, title, etc.)
5. Assess content relevance to query
6. Generate comprehensive validation report

### Error Handling
- Handle Qdrant connection failures gracefully
- Handle Cohere API errors during query embedding
- Log validation errors without stopping entire process
- Report individual query failures in summary

## Validation Rules
- All retrieved chunks must have complete metadata (URL, title, content)
- Content relevance determined by semantic similarity to query
- Metadata accuracy verified by comparing to source expectations
- Performance measured by execution time and success rate