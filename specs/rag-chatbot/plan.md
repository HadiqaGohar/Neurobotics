# RAG Chatbot Implementation Plan for AI Book Creation Project

## 1. Scope and Dependencies

### In Scope:
- Integration of a RAG (Retrieval-Augmented Generation) chatbot within the Docusaurus-based book.
- Utilizing OpenAI Agents/ChatKit SDKs with Gemini API for LLM interactions.
- Implementation of FastAPI for backend services.
- Integration with Neon Serverless Postgres for chat history, session, and document metadata persistence.
- Integration with Qdrant Cloud Free Tier for vector storage and similarity search.
- Document ingestion pipeline for book content (PDF, EPUB, TXT, Markdown).
- Text chunking, embedding generation using `sentence-transformers` (`all-MiniLM-L6-v2`).
- Semantic search and context retrieval logic.
- Enhanced chat endpoints to leverage RAG.
- Frontend (Docusaurus + React) integration with the RAG backend.
- Text selection-based query functionality on the frontend.
- Real voice processing (speech-to-text) for chat input.
- File upload processing for temporary knowledge base augmentation.
- Secure user authentication using Better-Auth (as per Constitution).
- User-centric content personalization based on background (as per Constitution).
- Multi-lingual support (Urdu translation) for chapter content (as per Constitution).

### Out of Scope:
- Full-fledged content management system beyond Docusaurus.
- Advanced AI model fine-tuning (initially, will rely on Gemini API and retrieved context).
- Complex multi-modal RAG beyond text and voice.
- Detailed UI/UX design changes beyond integrating RAG features.
- In-depth content authoring (focus is on the RAG system infrastructure).

### External Dependencies:
- **Neon Serverless Postgres**: Managed database service for relational data.
- **Qdrant Cloud Free Tier**: Managed vector database for embeddings.
- **Gemini API**: Large Language Model for response generation.
- **Better-Auth Platform**: External service for user authentication.
- **Docusaurus**: Frontend static site generator.
- **OpenAI Agents/ChatKit SDKs**: For interacting with the Gemini API.
- **Python Libraries**: FastAPI, Uvicorn, python-dotenv, openai-agents, qdrant-client, sentence-transformers, asyncpg, sqlalchemy, alembic, psycopg2-binary, speechrecognition, pydub, openai-whisper, PyPDF2, python-docx, ebooklib.
- **Frontend Libraries**: Existing Docusaurus/React dependencies.

## 2. Key Decisions and Rationale

| Decision                               | Options Considered                  | Rationale                                                                                                                                                                                                                                                                                                |
| :------------------------------------- | :---------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Vector Database**                    | Qdrant, Pinecone, Weaviate          | **Qdrant Cloud Free Tier** selected as per project constitution and provides a robust, scalable solution for vector search with a generous free tier suitable for initial development. Its Python client is well-integrated.                                                                            |
| **Relational Database**                | Neon Postgres, Supabase, SQLite     | **Neon Serverless Postgres** chosen as per project constitution. Offers serverless scalability, good integration with Python/FastAPI, and robust relational capabilities for chat history, user sessions, and document metadata.                                                                            |
| **Backend Framework**                  | FastAPI, Django, Flask              | **FastAPI** is lightweight, performant, and well-suited for building APIs. Its async capabilities are beneficial for I/O-bound operations like database calls and external API integrations (Gemini, Qdrant).                                                                                             |
| **LLM Integration**                    | OpenAI Agents/ChatKit SDKs + Gemini | **OpenAI Agents/ChatKit SDKs with Gemini API** mandated by project constitution. This provides a flexible and powerful interface for interacting with advanced LLMs, leveraging existing abstractions.                                                                                              |
| **Embedding Model**                    | `all-MiniLM-L6-v2`, OpenAI Embeddings | `all-MiniLM-L6-v2` from `sentence-transformers` provides a good balance of performance and accuracy for initial content embeddings, suitable for local processing and reducing external API dependencies for this core task. It's a standard choice for quick prototyping. |
| **Frontend Framework**                 | Docusaurus + React                  | **Docusaurus + React** is already the established frontend. The RAG chatbot will be integrated as a React component within the existing Docusaurus structure.                                                                                                                                    |
| **Authentication Platform**            | Better-Auth, Auth0, Firebase Auth   | **Better-Auth** is explicitly required by the project constitution, ensuring a consistent and standardized approach to user authentication.                                                                                                                                                              |
| **Document Chunking Strategy**         | Fixed-size, Recursive, Semantic     | **Fixed-size chunking with overlap** (e.g., 512 tokens) provides a straightforward starting point. It's easy to implement and manage. Will evaluate more advanced methods (e.g., recursive character text splitter) during optimization phases if context quality is an issue.                               |

### Principles:
- **Measurable**: All key components (latency, retrieval accuracy) should be measurable.
- **Reversible**: Database migrations and major architectural changes should be designed for reversibility where feasible.
- **Smallest Viable Change**: Each feature increment will be implemented with the minimum necessary changes, avoiding over-engineering.

## 3. Interfaces and API Contracts

### Public APIs (Backend - FastAPI)

#### 3.1 Chat Endpoints
-   **POST `/chat`**: Standard chatbot interaction (existing, to be enhanced).
    -   **Inputs**: `ChatRequest { message: str, session_id: str }`
    -   **Outputs**: `ChatResponse { response: str, session_id: str, context: Optional[List[str]] }`
    -   **Errors**: 400 (Invalid input), 500 (Internal server error, LLM failure, DB error)
-   **POST `/chat/rag`**: RAG-enhanced chatbot interaction.
    -   **Inputs**: `ChatRequest { message: str, session_id: str }`
    -   **Outputs**: `ChatResponse { response: str, session_id: str, context: List[str] }`
    -   **Errors**: 400 (Invalid input), 500 (RAG pipeline failure, LLM failure)
-   **POST `/chat/history`**: Retrieve chat history (existing).
    -   **Inputs**: `HistoryRequest { session_id: str }`
    -   **Outputs**: `List[ChatMessage { sender: str, content: str, timestamp: datetime }]`
    -   **Errors**: 404 (Session not found), 500

#### 3.2 Ingestion & Processing Endpoints
-   **POST `/ingest/book`**: Ingest the main book content.
    -   **Inputs**: `IngestBookRequest { book_path: str }` (or potentially file upload)
    -   **Outputs**: `IngestResponse { status: str, documents_indexed: int }`
    -   **Errors**: 400 (Invalid path/format), 500 (Processing/DB error)
-   **POST `/ingest/markdown`**: Ingest Docusaurus markdown documentation.
    -   **Inputs**: `IngestDocsRequest { docs_path: str }`
    -   **Outputs**: `IngestResponse { status: str, documents_indexed: int }`
    -   **Errors**: 400 (Invalid path), 500
-   **POST `/chat/file`**: Process uploaded files for temporary knowledge base.
    -   **Inputs**: `UploadFile, session_id: str`
    -   **Outputs**: `UploadResponse { status: str, message: str, context_added: bool }`
    -   **Errors**: 400 (Invalid file type/size), 500

#### 3.3 Advanced Feature Endpoints
-   **POST `/chat/voice`**: Process real voice input.
    -   **Inputs**: `VoiceRequest { audio_base64: str, session_id: str, language: Optional[str] }`
    -   **Outputs**: `ChatResponse { response: str, session_id: str, transcript: str }`
    -   **Errors**: 400 (Invalid audio), 500 (Speech-to-text failure, RAG failure)

### Versioning Strategy:
- API versioning will follow a `v1`, `v2` in the URL prefix if significant breaking changes are introduced. For initial development, all endpoints will be under a base `/api` or `/` path.

### Idempotency, Timeouts, Retries:
- **Idempotency**: Ingestion endpoints should be designed with idempotency in mind where possible (e.g., check for existing document hashes before re-indexing). Chat endpoints are inherently non-idempotent.
- **Timeouts**: Implement appropriate timeouts for external API calls (Gemini, Qdrant) and database operations to prevent long-running requests.
- **Retries**: Implement exponential backoff for transient errors when calling external services.

### Error Taxonomy:
- **400 Bad Request**: Malformed input, missing parameters.
- **401 Unauthorized**: Missing or invalid authentication token (for authenticated endpoints).
- **403 Forbidden**: User not authorized to access resource.
- **404 Not Found**: Resource (e.g., session, document) not found.
- **429 Too Many Requests**: Rate limiting applied.
- **500 Internal Server Error**: Unhandled exceptions, internal service failures, external API errors.
- **502 Bad Gateway**: Upstream service (e.g., Qdrant, Gemini) unresponsive.
- **503 Service Unavailable**: Temporary overload or maintenance.

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance:
-   **p95 Latency**: RAG-enhanced chat responses should be delivered within 3-5 seconds for 95% of requests.
-   **Throughput**: Backend should handle at least 100 concurrent RAG chat requests.
-   **Resource Caps**: Qdrant and Neon Postgres to be monitored for resource usage, initially within free tier limits, scaling as needed.
-   **Embedding Generation**: Batch processing and asynchronous operations for ingestion to minimize latency impact.

### Reliability:
-   **SLOs**: 99.9% availability for core chat and retrieval services.
-   **Error Budgets**: Define and track error rates for critical API endpoints.
-   **Degradation Strategy**: If RAG components fail, fallback to basic LLM response (without retrieved context) where possible.
-   **Data Durability**: Neon Postgres for persistent storage, Qdrant for vector durability.

### Security:
-   **AuthN/AuthZ**: User authentication via Better-Auth. Authorization checks for protected resources (e.g., personalized content, file uploads).
-   **Data Handling**: Sensitive chat history and user personalization data encrypted at rest and in transit.
-   **Secrets Management**: API keys (Gemini, Qdrant, Neon, Better-Auth) stored securely using environment variables (`.env` files in `book-backend`). NEVER hardcoded.
-   **Auditing**: Log security-relevant events (e.g., failed logins, data access attempts).
-   **Input Validation**: Strict input validation and sanitization on all API endpoints to prevent injection attacks (SQL, command, XSS).
-   **Rate Limiting**: Implement rate limiting on public API endpoints to prevent abuse and DoS attacks.

### Cost:
-   Initial development will leverage free tiers (Qdrant, Neon).
-   Monitor API usage (Gemini) to manage costs.
-   Optimize storage for vector embeddings and chat history.

## 5. Data Management and Migration

### Source of Truth:
-   **Neon Postgres**: Primary source of truth for user sessions, chat messages, and document metadata (e.g., document title, chunk index, original source).
-   **Qdrant**: Primary source of truth for vector embeddings. Cross-referencing with Postgres document metadata via `embedding_id` or similar.

### Schema Evolution:
-   **Alembic**: Will be used for database schema migrations in Neon Postgres to manage changes in a controlled and versioned manner.
-   **Versioned Collections**: Qdrant collections can be versioned or recreated as embedding models evolve, with appropriate re-indexing pipelines.

### Migration and Rollback:
-   Alembic migration scripts will be reviewed and tested before deployment.
-   Rollback procedures for database migrations will be documented.

### Data Retention:
-   Define retention policies for chat history and temporary uploaded document data.

### Database Schema (Neon Postgres):
```sql
-- Sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE, -- Assuming a 'users' table from Better-Auth integration
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES sessions(id) ON DELETE CASCADE,
    content TEXT NOT NULL,
    sender VARCHAR(10) NOT NULL CHECK (sender IN ('user', 'ai')),
    timestamp TIMESTAMP DEFAULT NOW(),
    metadata JSONB, -- Store additional context, e.g., retrieved document IDs
    PRIMARY KEY (id, session_id) -- Composite primary key for partitioning if needed
);

-- Documents table (for metadata about original sources and chunks)
CREATE TABLE documents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    title VARCHAR(255) NOT NULL,
    source_uri TEXT NOT NULL, -- e.g., URL, file path, book chapter
    total_chunks INTEGER,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Document Chunks table (metadata for each chunk, linked to vectors in Qdrant)
CREATE TABLE document_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    document_id UUID REFERENCES documents(id) ON DELETE CASCADE,
    chunk_index INTEGER NOT NULL,
    content TEXT NOT NULL, -- Storing original chunk text here for retrieval/reconstruction
    embedding_id TEXT UNIQUE, -- ID used in Qdrant
    metadata JSONB, -- Any additional chunk-specific info
    created_at TIMESTAMP DEFAULT NOW()
);
```

## 6. Operational Readiness

### Observability:
-   **Logs**: Standard FastAPI logging, structured logging for RAG pipeline steps, errors, and warnings.
-   **Metrics**: Prometheus/Grafana integration (if applicable) to track:
    -   API request rates, latencies, error rates.
    -   Qdrant search latencies and hit rates.
    -   Embedding generation times.
    -   Database query performance.
-   **Traces**: OpenTelemetry (or similar) to trace requests across services (FastAPI, Qdrant, Gemini).

### Alerting:
-   Alerts configured for critical thresholds (e.g., high error rates, service downtime, slow response times).
-   On-call rotations and runbooks for addressing common issues.

### Runbooks for common tasks:
-   Database migration (Alembic).
-   Qdrant collection management (creation, re-indexing).
-   Application deployment and rollback.
-   Troubleshooting RAG retrieval failures.

### Deployment and Rollback strategies:
-   **Docker**: Containerize the FastAPI backend for consistent deployment environments.
-   **CI/CD**: Automate build, test, and deployment processes.
-   **Canary Deployments/Blue-Green**: Implement phased rollouts where possible to minimize risk.
-   **Rollback**: Ability to quickly revert to previous stable versions.

### Feature Flags and compatibility:
-   Consider using feature flags for new RAG features (e.g., voice processing, file upload) to enable/disable them independently.

## 7. Risk Analysis and Mitigation

| Risk                                     | Blast Radius                                        | Mitigation Strategy                                                                                                                                                                                                                                                                                                                                                                                                                       |
| :--------------------------------------- | :-------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Low Retrieval Accuracy**               | Poor chatbot responses, user dissatisfaction        | **Mitigation**: Experiment with different chunking strategies and sizes. Evaluate alternative embedding models if `all-MiniLM-L6-v2` is insufficient. Implement evaluation metrics (e.g., RAGAS) and manual review of retrieved contexts. Feedback loop for continuous improvement.                                                                                                                                     |
| **High Latency for RAG Queries**         | Slow user experience, timeouts                      | **Mitigation**: Optimize Qdrant indexing and query parameters. Implement caching for frequently accessed embeddings and search results. Optimize network calls to external APIs. Utilize asynchronous programming effectively in FastAPI.                                                                                                                                                                                           |
| **Data Inconsistency (Postgres/Qdrant)** | Incorrect RAG responses, stale data                 | **Mitigation**: Design robust ingestion pipelines with transactional integrity where possible. Implement reconciliation mechanisms for ensuring data synchronization. Use unique identifiers (e.g., `embedding_id`) to link records across databases. Monitor data integrity.                                                                                                                                    |
| **Security Vulnerabilities**             | Data breaches, unauthorized access, system compromise | **Mitigation**: Adhere to OWASP Top 10. Implement input validation, authentication (Better-Auth) and authorization. Secure API key management (environment variables). Regularly audit code for vulnerabilities. Implement rate limiting and WAF (Web Application Firewall). Encrypt sensitive data.                                                                                                            |
| **Cost Overruns (Gemini, Qdrant, Neon)** | Exceeding budget, unsustainable operations          | **Mitigation**: Monitor API usage closely. Leverage free tiers judiciously. Optimize embedding storage and retrieval to minimize Qdrant calls. Implement caching to reduce redundant LLM calls. Review and optimize database queries to reduce Neon compute usage.                                                                                                                                             |
| **Complex Deployment/Maintenance**       | Operational overhead, increased downtime            | **Mitigation**: Automate deployment with Docker and CI/CD. Document clear runbooks. Use infrastructure-as-code where appropriate. Ensure observability tools are in place for quick troubleshooting. Design for modularity to isolate failures.                                                                                                                                                                           |
| **Integration with Better-Auth**         | Failed authentication, user access issues           | **Mitigation**: Thoroughly test Better-Auth integration. Follow Better-Auth documentation precisely. Handle authentication tokens securely (e.g., HttpOnly cookies, secure storage). Implement robust error handling for authentication failures.                                                                                                                                                                 |
| **Limited Multi-Lingual Support**        | Poor Urdu translations, limited RAG functionality   | **Mitigation**: Research pre-trained multi-lingual embedding models if `all-MiniLM-L6-v2` struggles with Urdu. Ensure translation services (if external) are robust. Evaluate prompt engineering for multi-lingual LLM responses. Clearly define scope for initial Urdu support (e.g., chapter content only).                                                                                                        |

## 8. Evaluation and Validation

### Definition of Done:
-   All features outlined in the "In Scope" section are implemented and meet functional requirements.
-   Unit, integration, and end-to-end tests pass with adequate code coverage.
-   Performance NFRs (latency, throughput) are met under load testing.
-   Security audits (static analysis, penetration testing) are conducted and critical findings addressed.
-   Deployment to production environment is successful and stable.
-   Documentation (API, deployment, runbooks) is complete and up-to-date.
-   PHRs are created for all significant development activities.
-   ADRs are created for architecturally significant decisions.

### Output Validation:
-   **Chat Responses**: LLM responses are coherent, relevant to the query, and grounded in retrieved context (for RAG queries).
-   **Retrieval**: Retrieved documents are semantically relevant to the user's query.
-   **Database Operations**: Data is stored and retrieved correctly from Neon Postgres.
-   **Embeddings**: Embeddings are generated consistently and accurately.
-   **Voice Processing**: Speech-to-text accuracy is acceptable.
-   **File Processing**: Files are correctly parsed, chunked, and indexed.

## 9. Architectural Decision Record (ADR) - Intelligent Suggestion

Based on this detailed plan, several architecturally significant decisions have been made, particularly regarding the choice and integration of the vector database (Qdrant), relational database (Neon Postgres), backend framework (FastAPI), and LLM integration (OpenAI Agents + Gemini API). These decisions have long-term consequences, involved considering alternatives, and influence the overall system design.

📋 Architectural decision detected: Core RAG Technology Stack Selection (Qdrant, Neon Postgres, FastAPI, Gemini API). Document reasoning and tradeoffs? Run `/sp.adr "Core RAG Technology Stack"`

## Implementation Phases (Detailed from RAG.md)

### Phase 1: Database Setup
-   **Duration**: 2-3 hours
-   **Tasks**:
    1.  Install Python dependencies: `asyncpg`, `sqlalchemy`, `alembic`, `psycopg2-binary`.
    2.  Define SQLAlchemy models for `sessions`, `messages`, `documents`, and `document_chunks` based on the schema above.
    3.  Initialize Alembic (`uv run alembic init alembic`).
    4.  Create initial migration (`uv run alembic revision --autogenerate -m "Initial RAG schema"`).
    5.  Apply migration to Neon Postgres (`uv run alembic upgrade head`).
    6.  Configure database connection string (`NEON_DATABASE_URL`) in `.env`.

### Phase 2: Vector Database Integration
-   **Duration**: 3-4 hours
-   **Tasks**:
    1.  Install Python dependencies: `qdrant-client`, `sentence-transformers`.
    2.  Configure Qdrant client connection using `QDRANT_URL` and `QDRANT_API_KEY` from `.env`.
    3.  Implement `EmbeddingService` class with methods for:
        -   Loading `all-MiniLM-L6-v2` model.
        -   `generate_embeddings(texts: List[str]) -> List[List[float]]`.
        -   `chunk_document(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]`.
    4.  Create Qdrant collection (e.g., "book_content") with appropriate vector parameters (dimension, distance metric).

### Phase 3: RAG Core Implementation
-   **Duration**: 4-5 hours
-   **Tasks**:
    1.  Implement `RAGService` class:
        -   Constructor takes `qdrant_client` and `embedding_service`.
        -   `retrieve_context(query: str, top_k: int = 5) -> List[str]`: Generates query embedding, searches Qdrant, and retrieves chunk content from Postgres using `embedding_id` linkage.
        -   `generate_rag_response(query: str, context: List[str]) -> str`: Combines context with query, constructs a prompt, and uses existing Gemini integration for response.
    2.  Enhance FastAPI `/chat` endpoint or create new `/chat/rag` endpoint to:
        -   Call `rag_service.retrieve_context`.
        -   Call `rag_service.generate_rag_response`.
        -   Store user query, RAG response, and retrieved context in `messages` table.
    3.  Integrate session management to persist chat history with `sessions` table.

### Phase 4: Book Content Integration
-   **Duration**: 2-3 hours
-   **Tasks**:
    1.  Install Python dependencies for document parsing: `PyPDF2`, `python-docx`, `ebooklib`.
    2.  Implement `BookIngestionService` class:
        -   `ingest_book_content(book_path: str)`: Reads various file types, chunks content, generates embeddings, stores metadata in Postgres (`documents`, `document_chunks`), and vectors in Qdrant.
        -   `ingest_markdown_docs(docs_path: str)`: Processes Docusaurus markdown files similarly.
    3.  Create an admin endpoint or a CLI script to trigger content ingestion.
    4.  Frontend: Implement text selection handler (`handleTextSelection` in TypeScript) to detect selected text and offer "Ask about this" functionality, sending selected text as an implicit query to the RAG backend.

### Phase 5: Advanced Features & Refinements
-   **Duration**: 3-4 hours
-   **Tasks**:
    1.  Install Python dependencies for voice processing: `speechrecognition`, `pydub`, `openai-whisper`.
    2.  Implement `/chat/voice` endpoint: Decode base64 audio, use Whisper for speech-to-text, process transcript with RAG, return response.
    3.  Implement `/chat/file` endpoint: Extract text from uploaded files (PDF, DOCX, TXT), chunk, generate temporary embeddings (or add to a dedicated user-specific Qdrant collection), process queries against this content.
    4.  Implement Frontend integration for voice input and file upload UI components.
    5.  Integrate Better-Auth for user signup/signin.
    6.  Implement logic for user-centric content personalization based on background.
    7.  Implement multi-lingual support (Urdu translation) for chapter content.

### Phase 6: Testing & Polish
-   **Duration**: 2-3 hours
-   **Tasks**:
    1.  Develop comprehensive unit tests for `EmbeddingService`, `RAGService`, and database models.
    2.  Develop integration tests for the full RAG pipeline (FastAPI endpoints, Qdrant interaction, Postgres operations).
    3.  Develop end-to-end tests for critical user flows (chat, voice, file upload, text selection).
    4.  Perform load testing to validate performance NFRs.
    5.  Address any identified bugs, performance bottlenecks, or security vulnerabilities.
    6.  Update documentation (API, deployment, runbooks).

---

## Required Dependencies

### Backend Dependencies (`pyproject.toml`)
```toml
[project]
dependencies = [
    # Existing
    "python-dotenv>=1.2.1",
    "fastapi>=0.104.1",
    "uvicorn>=0.24.0",
    "python-multipart>=0.0.6",
    "openai-agents>=0.6.2",

    # New for RAG & Advanced Features
    "qdrant-client>=1.7.0",
    "sentence-transformers>=2.2.2",
    "asyncpg>=0.29.0",
    "sqlalchemy>=2.0.0",
    "alembic>=1.13.0",
    "psycopg2-binary>=2.9.0",
    "speechrecognition>=3.10.0",
    "pydub>=0.25.1",
    "openai-whisper>=20231117",
    "PyPDF2>=3.0.1",
    "python-docx>=1.1.0",
    "ebooklib>=0.18",
]
```

### Frontend Dependencies (`package.json`)
Existing dependencies are sufficient; AI SDK is already installed.

## Environment Variables Required (`.env` in `book-backend`)
```env
# Existing
GEMINI_API_KEY=your_gemini_api_key

# New Required
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=postgresql://user:password@host/database
OPENAI_API_KEY=your_openai_api_key  # For Whisper if using OpenAI models
```

## Deployment Configuration

### Docker Setup (`book-backend/Dockerfile`)
```dockerfile
# Update book-backend/Dockerfile
FROM python:3.13-slim

WORKDIR /app
COPY pyproject.toml uv.lock ./.specify/ /app/.specify/ # Copy .specify for scripts
RUN pip install uv && uv sync --frozen

COPY . .
EXPOSE 8000

CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Database Migration (Alembic)
```bash
# Initialize Alembic (if not already done)
cd book-backend
uv run alembic init alembic

# Create migration
uv run alembic revision --autogenerate -m "Initial RAG schema"

# Apply migration
uv run alembic upgrade head
```

## Constitution Check

### I. Docusaurus & Spec-Kit Plus for Book Development
- **Compliance**: ✅ Fully compliant. Plan leverages Docusaurus for frontend, and Spec-Kit Plus (via `/sp.plan` workflow) for managing specifications, plans, and tasks.

### II. Integrated RAG Chatbot Development
- **Compliance**: ✅ Fully compliant. The core of this plan is the implementation of the RAG chatbot using OpenAI Agents/ChatKit SDKs with Gemini API, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier. It includes answering questions about book content and user-selected text.

### III. Reusable Intelligence via Claude Code Subagents & Skills
- **Compliance**: ✅ Compliant. The plan implicitly supports the use of Claude Code Subagents and Agent Skills throughout the development workflow, as this entire planning process is an example of leveraging such capabilities. Specific implementation details for *creating* new subagents/skills are out of scope for *this* plan, but the architecture enables their future use.

### IV. Secure User Authentication (Better-Auth)
- **Compliance**: ✅ Fully compliant. The plan explicitly incorporates Better-Auth for user signup and signin functionalities.

### V. User-Centric Content Personalization
- **Compliance**: ✅ Fully compliant. The plan includes integrating personalization logic based on user background collected during signup.

### VI. Multi-Lingual Content Support (Urdu Translation)
- **Compliance**: ✅ Fully compliant. The plan incorporates the ability for logged-in users to translate chapter content into Urdu.

### Overall Compliance: ✅ All core principles of the Constitution are addressed and adhered to in this implementation plan.
