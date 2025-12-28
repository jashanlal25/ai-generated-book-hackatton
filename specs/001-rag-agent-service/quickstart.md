# Quickstart Guide: RAG Agent and Backend Service

## Prerequisites

- Python 3.10+
- pip package manager
- Access to OpenAI API key
- Access to Qdrant Cloud collection
- Access to Neon Postgres database

## Setup

### 1. Clone and Navigate to Project

```bash
git clone <repository-url>
cd <project-directory>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install fastapi openai qdrant-client asyncpg python-dotenv uvicorn
```

### 4. Set Up Environment Variables

Create a `.env` file in the project root with the following:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_neon_postgres_connection_string_here
```

## Running the Service

### 1. Database Setup

Run database migrations to set up the conversation and message tables:

```bash
# This would be implemented as a migration script
python -m backend.db.migrate
```

### 2. Start the Server

```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

### 3. API Usage

#### Chat Endpoint

Send a question to the RAG agent:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main concept discussed in chapter 3?",
    "conversation_id": "optional-conversation-id",
    "selected_text": "optional text to constrain responses to"
  }'
```

Response format:
```json
{
  "response": "The main concept in chapter 3 is...",
  "conversation_id": "unique-conversation-id",
  "sources": ["source1", "source2"]
}
```

## Testing

### Unit Tests

```bash
python -m pytest backend/tests/unit/
```

### Integration Tests

```bash
python -m pytest backend/tests/integration/
```

## Validation

### Basic Functionality Test

1. Start the service
2. Send a test question: `curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"question": "What is this book about?"}'`
3. Verify you receive a relevant response based on book content
4. Test selected-text-only mode by including the `selected_text` parameter

### Performance Test

Verify that responses are returned within 5 seconds for typical queries.

### Persistence Test

Start a conversation, send multiple messages, and verify that conversation history is maintained.