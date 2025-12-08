# RAG Chatbot Implementation Tasks

## Feature: RAG Chatbot Integration

This document outlines the detailed tasks for implementing the RAG (Retrieval-Augmented Generation) chatbot based on the `plan.md` in `specs/rag-chatbot/`.

## Dependencies

Completion of tasks is sequential within each phase, and phases are largely sequential. Parallel tasks are marked with [P].

## Implementation Strategy

We will proceed with an incremental delivery approach, focusing on completing each phase and its associated user stories/features before moving to the next.

---

## Phase 1: Database Setup

**Goal**: Establish the Neon Serverless Postgres database schema and migration system.

- [x] T001 Install Python backend dependencies: `asyncpg`, `sqlalchemy`, `alembic`, `psycopg2-binary` in `book-backend/pyproject.toml`
- [x] T002 Define SQLAlchemy models for `sessions`, `messages`, `documents`, and `document_chunks` in `book-backend/src/database/models.py`
- [x] T003 Initialize Alembic in `book-backend/` by running `uv run alembic init alembic`
- [x] T004 Create initial Alembic migration for RAG schema in `book-backend/` by running `uv run alembic revision --autogenerate -m "Initial RAG schema"`
- [x] T005 Apply Alembic migration to Neon Postgres in `book-backend/` by running `uv run alembic upgrade head`
- [x] T006 Configure `NEON_DATABASE_URL` environment variable in `book-backend/.env`

## Phase 2: Vector Database Integration

**Goal**: Set up Qdrant client and implement the embedding generation service.

- [x] T007 Install Python backend dependencies: `qdrant-client`, `sentence-transformers` in `book-backend/pyproject.toml`
- [x] T008 Configure Qdrant client connection using `QDRANT_URL` and `QDRANT_API_KEY` from `.env` in `book-backend/src/services/qdrant_client.py`
- [x] T009 Implement `EmbeddingService` class (loading `all-MiniLM-L6-v2` model) in `book-backend/src/services/embedding_service.py`
- [x] T010 Implement `generate_embeddings(texts: List[str]) -> List[List[float]]` method in `book-backend/src/services/embedding_service.py`
- [x] T011 Implement `chunk_document(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]` method in `book-backend/src/services/embedding_service.py`
- [x] T012 Create Qdrant collection (e.g., "book_content") with appropriate vector parameters (dimension, distance metric) using the Qdrant client in `book-backend/src/services/qdrant_service.py`

## Phase 3: RAG Core Implementation

**Goal**: Implement the core RAG retrieval logic and enhance chat endpoints.

- [x] T013 Implement `RAGService` class with constructor (`qdrant_client`, `embedding_service`) in `book-backend/src/services/rag_service.py`
- [x] T014 Implement `retrieve_context(query: str, top_k: int = 5) -> List[str]` method in `book-backend/src/services/rag_service.py`
- [x] T015 Implement `generate_rag_response(query: str, context: List[str]) -> str` method in `book-backend/src/services/rag_service.py`
- [x] T016 Enhance FastAPI `/chat` endpoint or create new `/chat/rag` endpoint to call `rag_service.retrieve_context` and `rag_service.generate_rag_response` in `book-backend/src/api/chat.py`
- [x] T017 Integrate session management to persist chat history with `sessions` table in `book-backend/src/api/chat.py` and `book-backend/src/database/crud.py`
- [x] T018 Store user query, RAG response, and retrieved context in `messages` table in `book-backend/src/api/chat.py`

## Phase 4: Book Content Integration

**Goal**: Implement the document ingestion pipeline and frontend text selection feature.

- [x] T019 Install Python backend dependencies for document parsing: `PyPDF2`, `python-docx`, `ebooklib` in `book-backend/pyproject.toml`
- [x] T020 Implement `BookIngestionService` class in `book-backend/src/services/ingestion_service.py`
- [x] T021 Implement `ingest_book_content(book_path: str)` method (reads various file types, chunks, embeds, stores metadata in Postgres, vectors in Qdrant) in `book-backend/src/services/ingestion_service.py`
- [x] T022 Implement `ingest_markdown_docs(docs_path: str)` method (processes Docusaurus markdown files) in `book-backend/src/services/ingestion_service.py`
- [x] T023 Create an admin endpoint or CLI script to trigger content ingestion in `book-backend/src/api/ingestion.py` or `book-backend/scripts/ingest_content.py`
- [x] T024 [P] Frontend: Implement text selection handler (`handleTextSelection`) in `book-frontend/src/components/Chatbot/TextSelection.tsx`
- [x] T025 [P] Frontend: Integrate "Ask about this" functionality, sending selected text as implicit query to RAG backend in `book-frontend/src/components/Chatbot/ChatWindow.tsx`

## Phase 5: Advanced Features & Refinements

**Goal**: Implement voice processing, file upload, authentication, personalization, and multi-lingual support.

- [x] T026 Install Python backend dependencies for voice processing: `speechrecognition`, `pydub`, `openai-whisper` in `book-backend/pyproject.toml`
- [x] T027 Implement `/chat/voice` endpoint (decode base64 audio, Whisper speech-to-text, process with RAG, return response) in `book-backend/src/api/voice_chat.py`
- [x] T028 Implement `/chat/file` endpoint (extract text from uploaded files, chunk, generate temporary embeddings, process queries) in `book-backend/src/api/file_upload.py`
- [x] T029 [P] Frontend: Integrate voice input UI components in `book-frontend/src/components/Chatbot/VoiceInput.tsx`
- [x] T030 [P] Frontend: Integrate file upload UI components in `book-frontend/src/components/Chatbot/FileUpload.tsx`
- [x] T031 Integrate Better-Auth for user signup/signin in `book-backend/src/auth/` and `book-frontend/src/auth/`
- [x] T032 Implement logic for user-centric content personalization based on background in `book-backend/src/services/personalization_service.py` and `book-frontend/src/components/Personalization.tsx`
- [ ] T033 Implement multi-lingual support (Urdu translation) for chapter content in `book-frontend/src/components/LanguageSelector.tsx` and `book-backend/src/services/translation_service.py`

## Phase 6: Testing & Polish

**Goal**: Ensure system quality, performance, and robustness.

- [x] T034 Develop comprehensive unit tests for `EmbeddingService` in `book-backend/tests/unit/test_embedding_service.py`
- [x] T035 Develop comprehensive unit tests for `RAGService` in `book-backend/tests/unit/test_rag_service.py`
- [x] T036 Develop comprehensive unit tests for database models and CRUD operations in `book-backend/tests/unit/test_database.py`
- [x] T037 Develop integration tests for the full RAG pipeline (FastAPI endpoints, Qdrant interaction, Postgres operations) in `book-backend/tests/integration/test_rag_pipeline.py`
- [x] T038 Develop end-to-end tests for critical user flows (chat, voice, file upload, text selection) in `book-frontend/tests/e2e/`
- [x] T039 Perform load testing to validate performance NFRs using tools like `locust` or `pytest-benchmark`
- [x] T040 Address any identified bugs, performance bottlenecks, or security vulnerabilities
- [x] T041 Update API documentation for new RAG endpoints in `book-backend/docs/`
- [ ] T042 Update deployment and runbooks documentation in project root `/docs/`
