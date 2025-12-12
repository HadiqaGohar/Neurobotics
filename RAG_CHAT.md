# RAG-Embedding Chatbot — Detailed Specification (for Claude)

**Goal:** Build a Retrieval-Augmented Generation (RAG) chatbot that answers only from the book/module content. The system will ingest book files, convert them into embeddings, store them in a vector DB (Qdrant) + relational metadata store (Neon Postgres), and use Claude to generate paraphrased, source-backed answers.

---

# 1. Short Overview

* Ingest book/module text (PDF/MD/TXT), clean and chunk it.
* Create embeddings for chunks and store vectors in Qdrant with metadata.
* Store chunk metadata & original text in Neon Postgres (pgvector).
* When the user asks a question: embed the query → search Qdrant → fetch best chunks from Neon → construct a prompt → call Claude → return a paraphrased, source-referenced answer.

---

# 2. Goals / Deliverables

1. **Ingestion pipeline**: parser → chunker → embedder → Qdrant + Neon.
2. **Backend (FastAPI)**: endpoints for upload/ingest/chat/feedback/status.
3. **RAG chat flow**: retrieve relevant context and produce Claude-based answers.
4. **Prompt templates** that instruct Claude to only use provided contexts and to paraphrase.
5. **Minimal admin UI**: upload files, view ingestion status, inspect retrieved chunks.
6. **Tests & demo**: a reproducible demo with sample queries.

---

# 3. User Stories

* As a user, I ask questions and receive answers derived only from the book.
* As a user, answers can be in Urdu or English per preference.
* As an admin, I upload books and trigger (re)indexing.
* As an admin, I can see which chunks were used for an answer.

---

# 4. High-level Architecture (components)

* **Ingestion Service**: file parsers (PDF/MD/TXT), preprocessor, chunker.
* **Embedding Service**: Gemini/OpenAI embeddings or local model.
* **Vector DB**: Qdrant (stores vectors + payload metadata).
* **Relational DB**: Neon Postgres for storing chunk text & metadata (and sessions/messages).
* **Backend**: FastAPI to orchestrate ingestion, retrieval, and chat.
* **Claude**: the LLM that composes final answers from retrieved context.
* **Frontend**: chat UI (Docusaurus/Next/React) with upload and chat features.

---

# 5. Data Model & Metadata

Each chunk stored in Qdrant and Neon should include:

* `id` (uuid)
* `doc_id` (book/module id)
* `module`, `chapter`
* `chunk_id` (position)
* `text` (raw chunk)
* `start_pos`, `end_pos` (optional)
* `embedding` (vector)
* `language`
* `created_at`

---

# 6. Chunking & Embeddings

* **Chunk size**: ~400–800 tokens (or ~1500 chars if using char-based rule); use overlap (e.g., 100–200 tokens / 200 chars) to preserve context.
* **Preprocessing**: remove headers/footers/page numbers, normalize whitespace, keep paragraph boundaries when possible.
* **Embeddings**: use a high-quality embedding model (Gemini/OpenAI or sentence-transformers) producing matching dimensionality (e.g., 1536).
* **Deduplication**: skip near-identical chunks (cosine similarity threshold).

---

# 7. Retrieval & Prompting

* **Search**: cosine similarity search in Qdrant, return top_k chunks (default k=5).
* **Prompt construction**:

  * System instruction: restrict to provided sources, do not hallucinate, paraphrase and cite chunk ids.
  * Attach retrieved chunks labeled with `--SOURCE [doc:chunk_id]--`.
  * Ask Claude to produce: 1-line summary, brief explanation, and a source list.
* **Fallback**: if top similarity score is below a threshold, include a disclaimer: “Not found in the book.”

**Example system instruction:**

> “You are a helpful assistant. Use ONLY the provided context passages to answer. If the context does not contain an answer, say ‘This detail is not available in the book.’ Paraphrase answers and list the source chunk ids at the end.”

---

# 8. API Endpoints (suggested)

* `POST /upload` — upload file (title, language) → returns `job_id`.
* `POST /ingest` — trigger ingestion for an uploaded file or existing doc id.
* `GET /ingest/status/{job_id}` — ingestion progress.
* `POST /chat` — `{session_id, query, lang_pref, top_k=5}` → returns `{answer, sources:[{doc_id,chunk_id,score}], prompt}`.
* `POST /feedback` — collect user feedback for each response.

---

# 9. Claude Skill Design (recommended)

Create 3 small Claude skills (atomic):

**Skill A — ingest_file**

* Input: file path / file bytes, metadata (module, chapter).
* Action: parse → chunk → embed → qdrant_upsert → neon_insert.
* Output: `job_id`, chunk count, ingest status.

**Skill B — retrieve_context**

* Input: query, top_k.
* Action: embed query → qdrant.search(top_k) → return results with payload.
* Output: list of chunks + similarity scores.

**Skill C — answer_question**

* Input: user query, retrieved chunks, lang_pref.
* Action: build prompt (system + user + context) → call Claude completion → return answer, sources used.
* Output: `{answer, sources, prompt_sent}`.

---

# 10. Safety & Hallucination Controls

* Explicitly instruct Claude to not invent facts and only use the retrieved context.
* If evidence is weak (low similarity), return a conservative reply that clearly states the limitation.
* Store and surface the chunk ids used for audit.

---

# 11. Logging, Metrics & Feedback

* Log each query: query text, top_k hits, similarity scores, prompt, response, response_time.
* Feedback: thumbs-up/thumbs-down and optional comment. Use feedback to reprocess flagged chunks.

---

# 12. Testing & Acceptance Criteria

* **Unit tests**: chunker, embedder, Qdrant upsert, search.
* **Integration tests**: ingest → search → Claude answer uses expected chunks.
* **Manual QA**: 20 sample questions across chapters verifying answers reference book content.
* **Acceptance**: answers correctly reference book content for ≥90% of sample set; no major hallucinations.

---

# 13. Minimal Implementation Plan (MVP)

1. Implement ingestion pipeline & Qdrant upsert (embedding + metadata).
2. Implement `POST /chat` to perform retrieval + Claude prompt + response.
3. Add database tables for chunks & chat history in Neon.
4. Simple UI for upload and chat.
5. Tests & demo.

---

# 14. Minimal Required Environment Variables (do NOT store secrets in repo)

* `QDRANT_URL`, `QDRANT_API_KEY`
* `NEON_DATABASE_URL` (or separate Neon envs)
* `CLAUDE_API_KEY` / whatever credentials required by your Claude agent setup
* `EMBEDDING_MODEL` identifier (if using Gemini/OpenAI embeddings)

> **Do not commit .env to source control.** Use secret management in CI or cloud provider.

---

# 15. Example Prompt Template (final)

```
SYSTEM: You are an assistant. Use ONLY the provided context passages to answer. Paraphrase the content, do not hallucinate, and list source chunk IDs. If there is no answer in the context, say "This detail is not available in the book."

USER: {user_query}

CONTEXT:
--SOURCE [doc:{doc_id} chunk:{chunk_id} score:{score}]--
{chunk_text}
--SOURCE [doc:{doc_id2} chunk:{chunk_id2} score:{score2}]--
{chunk_text2}

INSTRUCTIONS:
1) Give a one-line summary.
2) Provide a short explanation.
3) At the end, list the sources used in the format doc:chunk_id.
4) Answer in {lang_pref}.
```

---

# 16. Advanced & Optional Features

* **Text selection queries**: allow users to select text in the UI and ask questions about that snippet (treat selection as extra context).
* **Voice (real)**: speech-to-text via Whisper or other model, then feed text to RAG.
* **Multi-language**: tag chunks with language and route queries to language-aware embeddings or translate as needed.
* **Caching**: cache embeddings and repeated search results; use Redis for session caching.

---

# 17. Success Criteria

* Bot answers questions using only book content, lists sources used, and does not hallucinate.
* Chunk & embedding store in Qdrant + Neon are consistent.
* Admin can upload and re-index books.
* Persistent chat history stored in Neon.

---

# 18. Next Steps I can do now (pick any, I’ll produce directly)

* Convert the spec into **Claude skill definitions** (JSON: inputs, outputs, sample payloads).
* Generate ready-made `ingest.py`, `tools.py`, and `qdrant_client.py` starter code.
* Produce a **FastAPI** `chat/rag` endpoint implementation.
* Create a minimal **skills.json** for Gemini/Gemini CLI if you decide to use Gemini instead of Claude.

Tell me which of the next steps you want and I’ll create it right away (e.g., “Make Claude skills JSON” or “Give ingest.py now”).
