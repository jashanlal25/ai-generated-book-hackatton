# Research: RAG Content Ingestion Pipeline

## Decision: Backend Project Structure
**Rationale**: Creating a dedicated backend folder with a single Python script (main.py) to house the entire ingestion pipeline. This approach provides clear separation of concerns while maintaining simplicity for a focused task.
**Alternatives considered**:
- Adding to existing project structure (would complicate the codebase)
- Creating multiple modules (would add unnecessary complexity for a single-purpose tool)

## Decision: System Architecture Pattern
**Rationale**: Implementing a pipeline architecture with distinct stages (URL discovery → text extraction → chunking → embedding → storage) to ensure clear separation of concerns and maintainability. This approach allows for easy debugging and testing of individual components.
**Alternatives considered**:
- Monolithic function approach (harder to debug and maintain)
- Microservices architecture (overkill for this single-purpose tool)

## Decision: URL Discovery Method
**Rationale**: Using both sitemap.xml and recursive crawling to ensure comprehensive URL discovery from the Docusaurus site. Sitemap provides a reliable list of URLs, while recursive crawling catches any missed pages.
**Alternatives considered**:
- Only sitemap.xml (might miss dynamically generated or newly added pages)
- Only recursive crawling (might miss pages or be blocked by robots.txt)

## Decision: Text Extraction & Cleaning Library
**Rationale**: Using BeautifulSoup4 for HTML parsing and text extraction as it's robust, well-maintained, and handles malformed HTML gracefully. It also provides fine-grained control over element selection and cleaning, which is essential for removing navigation, scripts, and styling elements from Docusaurus pages.
**Alternatives considered**:
- scrapy (overkill for this simple extraction task)
- regex parsing (unreliable for HTML parsing)
- lxml (similar functionality but less Python-native)

## Decision: Text Processing & Chunking Strategy
**Rationale**: Using 800-1200 token chunks with small overlap (200 tokens) to balance context preservation and processing efficiency. Using tiktoken for accurate token counting to ensure chunks are within the specified range. This aligns with the requirements and ensures semantic coherence.
**Alternatives considered**:
- Fixed character length chunks (doesn't account for semantic boundaries)
- Sentence-based chunks (might be too short for context)
- Paragraph-based chunks (might be too long and context-heavy)

## Decision: Embedding Model
**Rationale**: Using Cohere's latest embedding model (embed-english-v3.0 or equivalent) as required by the constraints. This model offers good performance for text similarity tasks and meets the requirement of using only Cohere for embeddings.
**Alternatives considered**:
- OpenAI embeddings (violates constraint of using only Cohere)
- Sentence Transformers (violates constraint of using only Cohere)

## Decision: Vector Database Collection Name
**Rationale**: Using "hackathon_aibook" as the collection name as specified in the requirements. This provides a clear, descriptive name for the vector store.
**Alternatives considered**:
- Generic names like "documents" (less descriptive)
- Other naming conventions (doesn't follow requirement)

## Decision: Environment Configuration
**Rationale**: Using python-dotenv for managing environment variables (API keys, Qdrant credentials) to ensure security and easy configuration across environments.
**Alternatives considered**:
- Hardcoded values (security risk)
- Command line arguments (inconvenient and potentially insecure)

## Decision: Qdrant Storage Design
**Rationale**: Using Qdrant Cloud for vector storage as required by constraints. Implementing upsert functionality to support incremental updates. Storing metadata (URL, title, chunk_id) alongside embeddings to enable proper retrieval and attribution.
**Alternatives considered**:
- Other vector databases (violates constraint of using only Qdrant Cloud)
- Local storage solutions (doesn't meet cloud requirement)