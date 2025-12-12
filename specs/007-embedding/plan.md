# Implementation Plan: Book Content Embedding for Chatbot

## 1. Overall Approach
The implementation will follow an iterative development process, focusing on building and testing each component (tools, ingestion, chatbot workflow) in sequence. We will leverage the provided `CHATBOT_README.md` and `spec.md` as the authoritative source for requirements.

## 2. Phases and Sub-tasks

### Phase 1: Environment Setup and Tool Definition

**Objective:** Create the necessary tools as specified in `spec.md` and `CHATBOT_README.md`.

*   **Task 1.1: `text_loader` Tool Implementation**
    *   Develop a tool to read various file types (`.txt`, `.md`, `.pdf`).
    *   Extract `module`, `chapter`, and clean text.
*   **Task 1.2: `chunk_text` Tool Implementation**
    *   Develop a tool to split text into chunks with specified `chunk_size` (1500) and `overlap` (200).
*   **Task 1.3: `make_embeddings` Tool Implementation**
    *   Develop a tool to generate embeddings using `gemini-embedding-001` (dimension 1536).
*   **Task 1.4: `qdrant_upsert` Tool Implementation**
    *   Develop a tool to connect to Qdrant and upsert data into the `book_chunks` collection.
    *   Ensure proper payload structure and collection auto-creation.
*   **Task 1.5: `neon_insert` Tool Implementation**
    *   Develop a tool to connect to Neon Postgres and insert data into the `book_chunks` table.
    *   Handle `VECTOR` extension check and creation.
*   **Task 1.6: `search_qdrant` Tool Implementation**
    *   Develop a tool to embed a query and retrieve top 5 results from Qdrant.
*   **Task 1.7: `rag_answer` Tool Implementation**
    *   Develop a tool orchestrating the RAG process: query embedding, Qdrant search, Neon fetch, prompt assembly, and Gemini model call.
    *   Implement the "NEVER answer outside book" constraint.

### Phase 2: Ingestion Workflow Implementation

**Objective:** Develop and execute the script for ingesting book content.

*   **Task 2.1: Ingestion Script Development**
    *   Create a script that orchestrates the `INGEST WORKFLOW` steps.
    *   Utilize `text_loader`, `chunk_text`, `make_embeddings`, `qdrant_upsert`, and `neon_insert` tools.
*   **Task 2.2: Module Loading**
    *   Implement logic to traverse `/modules` directory and identify book files.
*   **Task 2.3: Data Processing Loop**
    *   Integrate the chunking, embedding, Qdrant upsert, and Neon insert for each chunk.
*   **Task 2.4: Error Handling & Logging**
    *   Incorporate error rules (embedding dimension, Qdrant/Neon checks, invalid characters) and logging for chunk/embedding counts.

### Phase 3: Chatbot Workflow Implementation

**Objective:** Integrate the RAG components to create a functional chatbot.

*   **Task 3.1: Chatbot Interface (Conceptual)**
    *   Define how the user will interact with the chatbot (e.g., CLI, simple web interface). (Note: This is outside current scope unless specified by user, mainly for conceptualizing flow)
*   **Task 3.2: Query Processing**
    *   Implement the `CHATBOT WORKFLOW` steps using `search_qdrant` and fetching from Neon.
*   **Task 3.3: RAG Prompt Construction**
    *   Ensure effective prompt engineering for `gemini-2.0-flash` with retrieved context.
*   **Task 3.4: Response Generation**
    *   Call `gemini-2.0-flash` and enforce the "book-only" answer constraint.

## 3. Milestones and Completion Criteria
*   **Milestone 1:** All required tools are implemented and individually tested.
*   **Milestone 2:** The ingestion script successfully processes all book modules, populating Qdrant and Neon.
*   **Milestone 3:** The chatbot can receive queries, perform RAG, and return accurate, book-constrained answers.
*   **Final Completion:** All criteria specified in `spec.md` (Section 5) are met.

## 4. Dependencies
*   Access to Gemini API for embedding and chatbot models.
*   Access to Qdrant instance (URL and API Key).
*   Access to Neon Postgres instance (DB URL).
*   Existence of book content files in the `/modules` directory.
