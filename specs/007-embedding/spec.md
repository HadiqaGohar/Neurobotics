# Specification: Book Content Embedding for Chatbot

## 1. Introduction
This document specifies the technical requirements for embedding book content and integrating it into a Retrieval-Augmented Generation (RAG) chatbot system. The primary goal is to enable the chatbot to answer user queries using only the knowledge derived from the provided book modules.

## 2. Components

### 2.1. Embedding Model
-   **Model:** `gemini-embedding-001`
-   **Dimension:** `1536`

### 2.2. Vector Database (Qdrant)
-   **Purpose:** Store text embeddings for fast similarity search.
-   **Environment Variables:** `QDRANT_URL`, `QDRANT_API_KEY`
-   **Collection Name:** `book_chunks`
-   **Vector Size:** `1536`
-   **Distance Metric:** `Cosine`
-   **Payload Fields (per point):**
    -   `text`: Original text chunk
    -   `module`: Name of the book module
    -   `chapter`: Name of the chapter
    -   `chunk_id`: Unique identifier for the chunk

### 2.3. Relational Database (Neon - pgvector)
-   **Purpose:** Store original text, metadata, and embeddings for retrieval and persistence.
-   **Environment Variable:** `NEON_DB_URL`
-   **Table Name:** `book_chunks`
-   **Schema:**
    -   `id`: SERIAL PRIMARY KEY
    -   `qdrant_id`: TEXT (Reference to Qdrant point ID)
    -   `module`: TEXT
    -   `chapter`: TEXT
    -   `text`: TEXT (Original text chunk)
    -   `embedding`: VECTOR(1536)
    -   `created_at`: TIMESTAMPTZ DEFAULT now()
-   **Required Extension:** `VECTOR` (must be enabled if missing)

### 2.4. RAG Chatbot Model
-   **Model:** `gemini-2.0-flash`
-   **Constraint:** Must **NEVER** answer outside the book's knowledge.
    -   If information is not found: "This detail is not available in the book content."

## 3. Workflow Specifications

### 3.1. Ingestion Workflow

#### Step 1: Load Modules
-   Iterate through `/modules/<module_name>/*.txt`, `*.md`, `*.pdf` files.
-   Use `text_loader` tool to read and return clean text with `module` and `chapter` metadata.

#### Step 2: Chunk Text
-   Use `chunk_text` tool.
-   **Chunk Size:** `1500` characters
-   **Overlap:** `200` characters

#### Step 3: Generate Embeddings
-   Use `make_embeddings` tool.
-   Input: List of text chunks.
-   Output: List of `1536`-dimension vectors.

#### Step 4: Push to Qdrant
-   Use `qdrant_upsert` tool.
-   Insert chunks and corresponding vectors into the `book_chunks` collection.
-   Ensure `text`, `module`, `chapter` are included in the payload.

#### Step 5: Push to Neon
-   Use `neon_insert` tool.
-   Insert `qdrant_id`, `module`, `chapter`, `text`, and `embedding` into the `book_chunks` table.

### 3.2. Chatbot Workflow

#### Step 1: Convert User Query to Embedding
-   Embed the user's input query using `gemini-embedding-001`.

#### Step 2: Search Qdrant
-   Search the `book_chunks` collection in Qdrant with the query embedding.
-   Retrieve top 5 similar results (chunk IDs).

#### Step 3: Pull Full Text from Neon
-   Fetch the full original text and metadata from the Neon `book_chunks` table using the `qdrant_id`s obtained from Qdrant.

#### Step 4: Build Final RAG Prompt
-   Assemble a prompt for the `gemini-2.0-flash` model, incorporating the user's query and the retrieved relevant text as context.

#### Step 5: Answer using Gemini Model
-   Generate a response using the `gemini-2.0-flash` model, strictly adhering to the provided context.

## 4. Error Handling and Validation
-   **Embedding Dimension Mismatch:** Prevent any mismatch in embedding dimensions.
-   **Qdrant Collection:** If `book_chunks` collection is missing, it should be auto-created.
-   **Neon Extension:** Ensure `CREATE EXTENSION VECTOR;` is run if the `VECTOR` extension is missing in Neon.
-   **Invalid Characters:** Clean invalid characters from text before embedding.
-   **Logging:** Log chunk counts and embedding counts during ingestion.

## 5. Completion Criteria
The work is considered complete when:
1.  All specified book modules are ingested into Qdrant.
2.  All metadata and embeddings are inserted into Neon.
3.  The chatbot responds accurately and *only* from book knowledge.
4.  All ingestion and retrieval steps execute without errors.
