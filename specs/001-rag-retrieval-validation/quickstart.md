# Quickstart Guide: RAG Retrieval Validation

## Prerequisites
- Python 3.10+
- Valid Qdrant Cloud credentials in environment
- Valid Cohere API key in environment
- Existing "hackathon_aibook" collection with embedded content

## Setup
1. Copy `.env.example` to `.env` and fill in your credentials:
   ```bash
   QDRANT_URL="https://your-qdrant-cluster-url.cloud.qdrant.io:6333"
   QDRANT_API_KEY="your-qdrant-api-key-here"
   COHERE_API_KEY="your-cohere-api-key-here"
   ```

2. Install dependencies:
   ```bash
   pip install requests beautifulsoup4 cohere qdrant-client python-dotenv tiktoken
   ```

## Running Validation
Execute the validation script:
```bash
python validate_retrieval.py
```

Or with specific options:
```bash
python validate_retrieval.py --top-k 5 --output markdown --output-file validation_report.md
```

## Expected Output
The script will generate a validation report showing:
- Query success rate
- Metadata accuracy percentage
- Content relevance scores
- Individual query results with retrieved chunks
- Overall validation status

## Sample Output
```
RAG Retrieval Validation Report
===============================

Validation Date: 2025-12-13
Total Queries: 5
Success Rate: 100%
Average Relevance: 87.5%
Metadata Accuracy: 100%
Overall Status: PASS

Query Results:
1. Query: "What are the key components of humanoid robot navigation?"
   - Retrieved 5 chunks in 1.23s
   - Relevance: 92%
   - Metadata: Accurate ✓

2. Query: "How does SLAM work in robotics?"
   - Retrieved 5 chunks in 0.98s
   - Relevance: 85%
   - Metadata: Accurate ✓
...
```