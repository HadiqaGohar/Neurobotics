# RAG Chatbot Implementation Plan

## Current Status: ❌ NOT IMPLEMENTED
The current system is a basic chatbot with UI components but lacks the core RAG (Retrieval-Augmented Generation) functionality.

## What's Currently Implemented ✅

### Frontend (Docusaurus + React)
- ✅ Complete chatbot UI with floating icon
- ✅ Chat window with resizable interface
- ✅ Voice input functionality (UI only)
- ✅ File upload interface
- ✅ Message copying functionality
- ✅ Responsive design for mobile/desktop
- ✅ TypeScript types and API integration

### Backend (FastAPI + Python)
- ✅ Basic FastAPI server setup
- ✅ Gemini AI integration via OpenAI Agents
- ✅ CORS configuration for frontend
- ✅ Basic chat endpoints (/chat, /chat/history)
- ✅ Session management (in-memory)
- ✅ Environment configuration

## What's Missing for Full RAG Implementation ❌

### 1. Vector Database (Qdrant Cloud)
- ❌ Qdrant client setup and configuration
- ❌ Vector embeddings generation
- ❌ Document indexing pipeline
- ❌ Similarity search implementation

I already generate 
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.xNGlJ5_zq2q9v76wZMdCsn0sq5kEVaHI-4OhgL-lcn8
QDRANT_URL=https://0b43ae9c-4396-4cb0-baef-984149e44bb0.europe-west3-0.gcp.cloud.qdrant.io
all key are inside .env file present inside book-backend

### 2. Database Layer (Neon Serverless Postgres)
- ❌ Database schema design
- ❌ User session persistence
- ❌ Chat history storage
- ❌ Document metadata storage
- ❌ Database connection and ORM setup

# Neon Auth environment variables for Next.js
NEXT_PUBLIC_STACK_PROJECT_ID=c3ea287a-cdcf-4e60-9a5a-d3c3fa5c8204
NEXT_PUBLIC_STACK_PUBLISHABLE_CLIENT_KEY=pck_gdpmhyrtqa3xs9ajy6qqkcr1zfeg67cjs1rma5vbw1nq0
STACK_SECRET_SERVER_KEY=ssk_prp1teb7tzxq94x38fmhr36azns8dxr9xjhede7n3xdeg

# Database owner connection string
DATABASE_URL=postgresql://neondb_owner:npg_NVYLqfh90crD@ep-dark-fire-ahmpx1jc-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require
NEXT_PUBLIC_STACK_PROJECT_ID=c3ea287a-cdcf-4e60-9a5a-d3c3fa5c8204
NEXT_PUBLIC_STACK_PUBLISHABLE_CLIENT_KEY=pck_gdpmhyrtqa3xs9ajy6qqkcr1zfeg67cjs1rma5vbw1nq0
STACK_SECRET_SERVER_KEY=ssk_prp1teb7tzxq94x38fmhr36azns8dxr9xjhede7n3xdeg

# Database owner connection string
DATABASE_URL=postgresql://neondb_owner:npg_NVYLqfh90crD@ep-dark-fire-ahmpx1jc-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require
STACK_SECRET_SERVER_KEY=ssk_prp1teb7tzxq94x38fmhr36azns8dxr9xjhede7n3xdeg

# Database owner connection string
DATABASE_URL=postgresql://neondb_owner:npg_NVYLqfh90crD@ep-dark-fire-ahmpx1jc-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require

### 3. Document Processing Pipeline
- ❌ Book content ingestion
- ❌ Text chunking and preprocessing
- ❌ Embedding generation for book content
- ❌ Document indexing workflow

### 4. RAG Retrieval Logic
- ❌ Query embedding generation
- ❌ Semantic search in vector database
- ❌ Context retrieval and ranking
- ❌ Retrieved context integration with LLM

### 5. Advanced Features
- ❌ Text selection-based queries
- ❌ Real voice processing (currently mock)
- ❌ File upload processing for documents
- ❌ Multi-language support

## Implementation Plan

### Phase 1: Database Setup (2-3 hours)

#### 1.1 Neon Postgres Setup
```bash
# Install dependencies
cd book-backend
uv add asyncpg sqlalchemy alembic psycopg2-binary

# Create database models
# - User sessions
# - Chat messages
# - Document metadata
# - Vector embeddings metadata
```

#### 1.2 Database Schema
```sql
-- Sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES sessions(id),
    content TEXT NOT NULL,
    sender VARCHAR(10) NOT NULL CHECK (sender IN ('user', 'ai')),
    timestamp TIMESTAMP DEFAULT NOW(),
    metadata JSONB
);

-- Documents table
CREATE TABLE documents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    title VARCHAR(255) NOT NULL,
    content TEXT NOT NULL,
    chunk_index INTEGER,
    embedding_id VARCHAR(255),
    created_at TIMESTAMP DEFAULT NOW()
);
```

### Phase 2: Vector Database Integration (3-4 hours)

#### 2.1 Qdrant Setup
```python
# Install Qdrant client
uv add qdrant-client sentence-transformers

# Configure Qdrant connection
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(
    url="https://your-cluster-url.qdrant.io",
    api_key="your-api-key"
)
```

#### 2.2 Embedding Pipeline
```python
# Document processing and embedding
from sentence_transformers import SentenceTransformer

class EmbeddingService:
    def __init__(self):
        self.model = SentenceTransformer('all-MiniLM-L6-v2')
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        return self.model.encode(texts).tolist()
    
    def chunk_document(self, text: str, chunk_size: int = 512) -> List[str]:
        # Implement text chunking logic
        pass
```

### Phase 3: RAG Implementation (4-5 hours)

#### 3.1 Retrieval Service
```python
class RAGService:
    def __init__(self, qdrant_client, embedding_service):
        self.qdrant = qdrant_client
        self.embeddings = embedding_service
    
    async def retrieve_context(self, query: str, top_k: int = 5) -> List[str]:
        # Generate query embedding
        query_embedding = self.embeddings.generate_embeddings([query])[0]
        
        # Search in Qdrant
        search_results = self.qdrant.search(
            collection_name="book_content",
            query_vector=query_embedding,
            limit=top_k
        )
        
        return [result.payload["content"] for result in search_results]
    
    async def generate_rag_response(self, query: str, context: List[str]) -> str:
        # Combine context with query for LLM
        context_text = "\n\n".join(context)
        prompt = f"""
        Context from the book:
        {context_text}
        
        User question: {query}
        
        Please answer the question based on the provided context from the book.
        """
        
        # Use existing Gemini integration
        return await generate_ai_response(prompt)
```

#### 3.2 Enhanced Chat Endpoints
```python
@app.post("/chat/rag")
async def chat_with_rag(request: ChatRequest):
    # Retrieve relevant context
    context = await rag_service.retrieve_context(request.message)
    
    # Generate RAG response
    response = await rag_service.generate_rag_response(request.message, context)
    
    # Store in database
    # Return response
```

### Phase 4: Book Content Integration (2-3 hours)

#### 4.1 Content Ingestion
```python
class BookIngestionService:
    def __init__(self, rag_service):
        self.rag_service = rag_service
    
    async def ingest_book_content(self, book_path: str):
        # Read book content (PDF, EPUB, TXT)
        # Chunk into manageable pieces
        # Generate embeddings
        # Store in Qdrant and Postgres
        pass
    
    async def ingest_markdown_docs(self, docs_path: str):
        # Process existing Docusaurus docs
        # Extract content from markdown files
        # Index for RAG retrieval
        pass
```

#### 4.2 Text Selection Feature
```typescript
// Frontend: Text selection handler
const handleTextSelection = () => {
  const selection = window.getSelection();
  const selectedText = selection?.toString();
  
  if (selectedText && selectedText.length > 10) {
    // Show "Ask about this" button
    // Send selected text as context to RAG
  }
};
```

### Phase 5: Advanced Features (3-4 hours)

#### 5.1 Voice Processing
```python
# Install speech recognition
uv add speechrecognition pydub openai-whisper

@app.post("/chat/voice")
async def process_voice_real(request: VoiceRequest):
    # Decode base64 audio
    # Use Whisper for speech-to-text
    # Process with RAG
    # Return response
```

#### 5.2 File Upload Processing
```python
@app.post("/chat/file")
async def process_file_upload(file: UploadFile, session_id: str):
    # Extract text from uploaded file
    # Add to knowledge base temporarily
    # Process query against uploaded content
    pass
```

## Required Dependencies

### Backend Dependencies
```toml
[project]
dependencies = [
    # Existing
    "python-dotenv>=1.2.1",
    "fastapi>=0.104.1",
    "uvicorn>=0.24.0",
    "python-multipart>=0.0.6",
    "openai-agents>=0.6.2",
    
    # New for RAG
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

### Frontend Dependencies
```json
{
  "dependencies": {
    // Existing dependencies are sufficient
    // AI SDK is already installed
  }
}
```

## Environment Variables Required

```env
# Existing
GEMINI_API_KEY=your_gemini_api_key

# New Required
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=postgresql://user:password@host/database
GEMINI_API_KEY=your_gemini_api_key  # For Whisper if using OpenAI
```

## Deployment Configuration

### Docker Setup
```dockerfile
# Update book-backend/Dockerfile
FROM python:3.13-slim

WORKDIR /app
COPY pyproject.toml uv.lock ./
RUN pip install uv && uv sync --frozen

COPY . .
EXPOSE 8000

CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Database Migration
```bash
# Initialize Alembic
cd book-backend
uv run alembic init alembic

# Create migration
uv run alembic revision --autogenerate -m "Initial schema"

# Apply migration
uv run alembic upgrade head
```

## Testing Strategy

### 1. Unit Tests
- Test embedding generation
- Test vector search
- Test RAG response generation

### 2. Integration Tests
- Test full RAG pipeline
- Test database operations
- Test API endpoints

### 3. End-to-End Tests
- Test complete user flow
- Test voice input processing
- Test file upload functionality

## Performance Considerations

### 1. Caching
- Cache embeddings for frequently asked questions
- Cache search results for similar queries
- Implement Redis for session caching

### 2. Optimization
- Batch embedding generation
- Async processing for file uploads
- Connection pooling for databases

### 3. Monitoring
- Track response times
- Monitor embedding quality
- Log search relevance scores

## Security Considerations

### 1. API Security
- Rate limiting on chat endpoints
- Input validation and sanitization
- Authentication for file uploads

### 2. Data Privacy
- Encrypt sensitive data in database
- Secure API key management
- GDPR compliance for chat history

## Estimated Timeline

- **Phase 1 (Database)**: 2-3 hours
- **Phase 2 (Vector DB)**: 3-4 hours  
- **Phase 3 (RAG Core)**: 4-5 hours
- **Phase 4 (Content)**: 2-3 hours
- **Phase 5 (Advanced)**: 3-4 hours
- **Testing & Polish**: 2-3 hours

**Total Estimated Time**: 16-22 hours

## Success Criteria

✅ **Functional RAG System**
- Users can ask questions about book content
- System retrieves relevant context from vector database
- AI provides accurate answers based on retrieved content

✅ **Text Selection Feature**
- Users can select text and ask questions about it
- System uses selected text as additional context

✅ **Persistent Storage**
- Chat history stored in Neon Postgres
- Sessions persist across browser refreshes

✅ **Voice & File Processing**
- Real voice-to-text conversion
- File upload and processing for temporary knowledge

✅ **Performance**
- Response time under 3 seconds for most queries
- Accurate retrieval of relevant content
- Scalable architecture for production use

## Next Steps

1. **Set up Neon Postgres database** and get connection string
2. **Create Qdrant Cloud account** and get API credentials  
3. **Implement database models** and migrations
4. **Build embedding and retrieval services**
5. **Integrate RAG logic** with existing chat endpoints
6. **Test with actual book content**
7. **Deploy and optimize**

This plan provides a complete roadmap for implementing a production-ready RAG chatbot system that meets all the specified requirements.