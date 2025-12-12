
---

# ⭐ **FINAL READY-TO-USE INSTRUCTIONS FOR GEMINI CLI AGENT**

👇 Ye instructions EXACT isi format me Gemini CLI ko do:

---

# 📘 **Instructions for My Book Knowledge System (Gemini CLI)**

## **1. GOAL**

I want you (Gemini CLI agent) to build a complete book-knowledge chatbot using:

* Gemini Embeddings
* Qdrant Vector Database
* Neon Postgres (pgvector)
* Custom Tools / Skills
* My book text stored inside multiple modules

When I chat with the bot, it must answer only from my book's knowledge using Retrieval-Augmented Generation (RAG).

---

# **2. REQUIRED BEHAVIOR**

Gemini CLI must:

### ✔ Read all book module files

Location:

```
/modules/<module_name>/*.txt  
/modules/<module_name>/*.md  
/modules/<module_name>/*.pdf
```

### ✔ Convert book text into embeddings

Use:

```
model: gemini-embedding-001
dimension: 1536
```

### ✔ Store embeddings in Qdrant

Use these env variables:

```
QDRANT_URL=...
QDRANT_API_KEY=...
```

Store:

* vector
* text
* module
* chapter
* chunk_id

### ✔ Store metadata & original text in Neon (pgvector)

Use:

```
NEON_DB_URL=...
```

Store:

* qdrant_id
* module
* chapter
* text
* embedding (vector 1536)
* created_at

---

# **3. REQUIRED TOOLS (Gemini CLI must create these)**

## 🔧 **Tool 1: text_loader**

Reads files → returns clean text.

**Tool requirements:**

* supports .txt, .md, .pdf
* returns `{module, chapter, text}`

---

## 🔧 **Tool 2: chunk_text**

Splits text into usable chunks for vectors.

**Rules:**

```
chunk_size = 1500 chars
overlap = 200 chars
```

---

## 🔧 **Tool 3: make_embeddings**

Uses Gemini API:

```
model: gemini-embedding-001
input: list of chunks
output: list of vectors (1536)
```

---

## 🔧 **Tool 4: qdrant_upsert**

Insert data inside Qdrant collection:

```
collection_name = "book_chunks"
vector_size = 1536
distance = "Cosine"
```

Payload must include:

```
{
  "text": chunk_text,
  "module": module_name,
  "chapter": chapter_name
}
```

---

## 🔧 **Tool 5: neon_insert**

Insert into Neon table:

```
book_chunks(id SERIAL,
  qdrant_id TEXT,
  module TEXT,
  chapter TEXT,
  text TEXT,
  embedding VECTOR(1536),
  created_at TIMESTAMPTZ DEFAULT now()
)
```

---

## 🔧 **Tool 6: search_qdrant**

On query → embed the query → get top 5 results.

---

## 🔧 **Tool 7: rag_answer**

Steps:

1. embed user query
2. search qdrant
3. fetch metadata from Neon
4. assemble context
5. call Gemini model
6. reply with book-accurate answer

---

# **4. INGEST WORKFLOW (Gemini CLI must execute this)**

### **Step 1 — Load all modules**

```
for each module folder:
  for each file:
    call text_loader
```

### **Step 2 — Chunk text**

```
chunks = chunk_text(text)
```

### **Step 3 — Generate embeddings**

```
vectors = make_embeddings(chunks)
```

### **Step 4 — Push to Qdrant**

```
qdrant_upsert(chunks, vectors, metadata)
```

### **Step 5 — Push to Neon**

```
neon_insert(qdrant_id, module, chapter, text, vector)
```

---

# **5. CHATBOT WORKFLOW**

When user asks something:

### ✔ Step 1 — Convert query → embedding

### ✔ Step 2 — Get top results from Qdrant

### ✔ Step 3 — Pull full text from Neon

### ✔ Step 4 — Build final RAG prompt

### ✔ Step 5 — Answer using Gemini model:

```
model: gemini-2.0-flash 
```

### ✔ Step 6 — NEVER answer outside my book

If information is not found in the book:
→ Say: **“This detail is not available in the book content.”**

---

# **6. ERROR RULES**

Gemini CLI must ensure:

✔ NEVER mismatch embedding dimension
✔ If Qdrant collection missing → auto create
✔ If Neon EXTENSION missing → run

```
CREATE EXTENSION VECTOR;
```

✔ Clean invalid characters before embedding
✔ Log chunk counts + embedding count

---

# **7. COMPLETION CONDITION**

Work is considered **COMPLETE** when:

1. All modules are inserted into Qdrant
2. All metadata + embeddings are inserted into Neon
3. Chatbot responds only using book knowledge
4. No step fails or throws an error

---

# ⭐ FINAL MESSAGE FOR GEMINI CLI

**Follow all above instructions EXACTLY.
Create all tools & skills.
Perform ingestion.
Then start answering queries using RAG.**

---

# 🔥 If you want

I can also generate for you:

✅ Ready-made `skills.json` for Gemini CLI
✅ Ready-made `tools.py`
✅ Ready-made `ingest.py` script
✅ Ready-made FastAPI RAG backend
✅ Ready-made Chatbot UI (Next.js + Tailwind)

Just tell me: **“Ready configs do”**
