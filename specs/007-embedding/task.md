# Tasks: Book Content Embedding for Chatbot

This document lists the actionable tasks for implementing the book content embedding and RAG chatbot, derived from the `plan.md`.

## Phase 1: Environment Setup and Tool Definition

- [ ] **Task 1.1: `text_loader` Tool Implementation**
    - [ ] Develop a tool to read various file types (`.txt`, `.md`, `.pdf`).
    - [ ] Extract `module`, `chapter`, and clean text.
- [ ] **Task 1.2: `chunk_text` Tool Implementation**
    - [ ] Develop a tool to split text into chunks with specified `chunk_size` (1500) and `overlap` (200).
- [ ] **Task 1.3: `make_embeddings` Tool Implementation**
    - [ ] Develop a tool to generate embeddings using `gemini-embedding-001` (dimension 1536).
- [ ] **Task 1.4: `qdrant_upsert` Tool Implementation**
    - [ ] Develop a tool to connect to Qdrant and upsert data into the `book_chunks` collection.
    - [ ] Ensure proper payload structure and collection auto-creation.
- [ ] **Task 1.5: `neon_insert` Tool Implementation**
    - [ ] Develop a tool to connect to Neon Postgres and insert data into the `book_chunks` table.
    - [ ] Handle `VECTOR` extension check and creation.
- [ ] **Task 1.6: `search_qdrant` Tool Implementation**
    - [ ] Develop a tool to embed a query and retrieve top 5 results from Qdrant.
- [ ] **Task 1.7: `rag_answer` Tool Implementation**
    - [ ] Develop a tool orchestrating the RAG process: query embedding, Qdrant search, Neon fetch, prompt assembly, and Gemini model call.
    - [ ] Implement the "NEVER answer outside book" constraint.

## Phase 2: Ingestion Workflow Implementation

- [ ] **Task 2.1: Ingestion Script Development**
    - [ ] Create a script that orchestrates the `INGEST WORKFLOW` steps.
    - [ ] Utilize `text_loader`, `chunk_text`, `make_embeddings`, `qdrant_upsert`, and `neon_insert` tools.
- [ ] **Task 2.2: Module Loading**
    - [ ] Implement logic to traverse `/modules` directory and identify book files.
- [ ] **Task 2.3: Data Processing Loop**
    - [ ] Integrate the chunking, embedding, Qdrant upsert, and Neon insert for each chunk.
- [ ] **Task 2.4: Error Handling & Logging**
    - [ ] Incorporate error rules (embedding dimension, Qdrant/Neon checks, invalid characters) and logging for chunk/embedding counts.

## Phase 3: Chatbot Workflow Implementation

- [ ] **Task 3.1: Chatbot Interface (Conceptual)**
    - [ ] Define how the user will interact with the chatbot.
- [ ] **Task 3.2: Query Processing**
    - [ ] Implement the `CHATBOT WORKFLOW` steps using `search_qdrant` and fetching from Neon.
- [ ] **Task 3.3: RAG Prompt Construction**
    - [ ] Ensure effective prompt engineering for `gemini-2.0-flash` with retrieved context.
- [ ] **Task 3.4: Response Generation**
    - [ ] Call `gemini-2.0-flash` and enforce the "book-only" answer constraint.

