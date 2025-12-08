# RAG Chatbot API Documentation

## Overview

The RAG (Retrieval-Augmented Generation) Chatbot API provides intelligent conversational capabilities enhanced with document retrieval and context-aware responses. This API supports text chat, voice processing, file uploads, and document ingestion.

## Base URL

```
Development: http://localhost:8000/api
Production: https://your-domain.com/api
```

## Authentication

Currently, the API uses session-based authentication. Future versions will include JWT-based authentication with Better-Auth integration.

## Rate Limiting

- **Default Limit**: 100 requests per hour per IP
- **Chat Endpoints**: 50 requests per hour per IP
- **File Upload**: 10 requests per hour per IP

## Error Handling

All endpoints return consistent error responses:

```json
{
  "detail": "Error description",
  "error_code": "ERROR_CODE",
  "timestamp": "2024-01-01T00:00:00Z"
}
```

### Common HTTP Status Codes

- `200` - Success
- `400` - Bad Request (validation error)
- `401` - Unauthorized
- `404` - Not Found
- `429` - Rate Limit Exceeded
- `500` - Internal Server Error

---

## Chat Endpoints

### POST /chat/rag

Enhanced chat endpoint with RAG (Retrieval-Augmented Generation) capabilities.

**Request Body:**
```json
{
  "message": "What is machine learning?",
  "session_id": "optional-uuid-string"
}
```

**Response:**
```json
{
  "response": "Machine learning is a subset of artificial intelligence...",
  "session_id": "uuid-string",
  "context": [
    "Retrieved context chunk 1",
    "Retrieved context chunk 2",
    "Retrieved context chunk 3"
  ]
}
```

**Features:**
- Context retrieval from vector database
- Session management
- Response caching
- Security validation

**Example:**
```bash
curl -X POST "http://localhost:8000/api/chat/rag" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain neural networks",
    "session_id": null
  }'
```

### POST /chat/history

Retrieve chat history for a session.

**Request Body:**
```json
{
  "session_id": "uuid-string"
}
```

**Response:**
```json
{
  "messages": [
    {
      "id": "msg-uuid",
      "content": "User message",
      "sender": "user",
      "timestamp": "2024-01-01T00:00:00Z",
      "session_id": "session-uuid"
    },
    {
      "id": "msg-uuid",
      "content": "AI response",
      "sender": "ai",
      "timestamp": "2024-01-01T00:00:00Z",
      "session_id": "session-uuid",
      "metadata": {
        "context": ["context chunks"]
      }
    }
  ]
}
```

---

## Voice Processing Endpoints

### POST /chat/voice

Process voice input and return text response.

**Request Body:**
```json
{
  "audio_data": "base64-encoded-audio-data",
  "session_id": "optional-uuid-string"
}
```

**Response:**
```json
{
  "response": "AI response to transcribed audio",
  "session_id": "uuid-string",
  "transcription": "Transcribed text from audio",
  "context": ["Retrieved context chunks"]
}
```

**Supported Audio Formats:**
- WAV
- MP3
- M4A
- FLAC

**Limitations:**
- Maximum file size: 10MB
- Maximum duration: 5 minutes
- Minimum quality: 16kHz sample rate

**Example:**
```bash
curl -X POST "http://localhost:8000/api/chat/voice" \
  -H "Content-Type: application/json" \
  -d '{
    "audio_data": "UklGRnoGAABXQVZFZm10...",
    "session_id": null
  }'
```

---

## File Upload Endpoints

### POST /chat/file

Upload and process documents for temporary context.

**Request:**
- Content-Type: `multipart/form-data`
- File field: `file`

**Supported File Types:**
- PDF (`.pdf`)
- Word Documents (`.docx`)
- EPUB (`.epub`)
- Text Files (`.txt`)
- Markdown (`.md`)

**Response:**
```json
{
  "message": "File processed successfully",
  "file_id": "temp-file-uuid",
  "chunks_processed": 15,
  "processing_time": 2.34
}
```

**Query Uploaded File:**
```json
{
  "message": "What does this document say about AI?",
  "file_id": "temp-file-uuid"
}
```

**Example:**
```bash
curl -X POST "http://localhost:8000/api/chat/file" \
  -F "file=@document.pdf"
```

---

## Document Ingestion Endpoints

### POST /ingestion/ingest

Ingest documents into the permanent knowledge base.

**Request Body:**
```json
{
  "file_path": "/path/to/document.pdf",
  "document_type": "book|article|manual",
  "metadata": {
    "title": "Document Title",
    "author": "Author Name",
    "category": "Category"
  }
}
```

**Response:**
```json
{
  "message": "Document ingested successfully",
  "document_id": "doc-uuid",
  "chunks_created": 25,
  "embeddings_generated": 25,
  "processing_time": 15.67
}
```

### POST /ingestion/batch

Batch ingest multiple documents.

**Request Body:**
```json
{
  "documents": [
    {
      "file_path": "/path/to/doc1.pdf",
      "document_type": "book",
      "metadata": {"title": "Book 1"}
    },
    {
      "file_path": "/path/to/doc2.pdf",
      "document_type": "article",
      "metadata": {"title": "Article 1"}
    }
  ]
}
```

**Response:**
```json
{
  "message": "Batch ingestion completed",
  "processed": 2,
  "failed": 0,
  "total_chunks": 50,
  "processing_time": 45.23,
  "results": [
    {
      "file_path": "/path/to/doc1.pdf",
      "status": "success",
      "document_id": "doc-uuid-1",
      "chunks": 25
    },
    {
      "file_path": "/path/to/doc2.pdf",
      "status": "success",
      "document_id": "doc-uuid-2",
      "chunks": 25
    }
  ]
}
```

---

## Health and Monitoring

### GET /health

System health check endpoint.

**Response:**
```json
{
  "status": "healthy",
  "environment": "development",
  "version": "1.0.0",
  "features": [
    "RAG Chat",
    "Voice Processing",
    "File Upload",
    "Document Ingestion",
    "Text Selection"
  ]
}
```

### GET /metrics

Performance metrics (development only).

**Response:**
```json
{
  "metrics": {
    "chat_response_time": {
      "count": 100,
      "mean": 1.23,
      "min": 0.45,
      "max": 3.21
    },
    "context_retrieval_time": {
      "count": 100,
      "mean": 0.34,
      "min": 0.12,
      "max": 0.89
    }
  },
  "cache_size": 150,
  "timestamp": 1704067200
}
```

---

## Data Models

### ChatMessage
```json
{
  "id": "string (UUID)",
  "content": "string",
  "sender": "user | ai",
  "timestamp": "string (ISO 8601)",
  "session_id": "string (UUID)",
  "metadata": {
    "context": ["string"],
    "processing_time": "number",
    "model_used": "string"
  }
}
```

### Session
```json
{
  "id": "string (UUID)",
  "user_id": "string (optional)",
  "title": "string",
  "created_at": "string (ISO 8601)",
  "updated_at": "string (ISO 8601)",
  "metadata": {
    "message_count": "number",
    "last_activity": "string (ISO 8601)"
  }
}
```

### Document
```json
{
  "id": "string (UUID)",
  "title": "string",
  "file_path": "string",
  "document_type": "book | article | manual",
  "status": "processing | completed | failed",
  "created_at": "string (ISO 8601)",
  "metadata": {
    "author": "string",
    "category": "string",
    "file_size": "number",
    "chunk_count": "number"
  }
}
```

---

## Security Considerations

### Input Validation
- All user inputs are sanitized to prevent XSS attacks
- SQL injection protection through parameterized queries
- File upload validation for type and size
- Rate limiting to prevent abuse

### Data Protection
- Sensitive data is hashed before storage
- Session tokens are securely generated
- CORS properly configured for frontend domains
- Security headers added to all responses

### Best Practices
- Use HTTPS in production
- Implement proper authentication
- Regular security audits
- Monitor for suspicious activity
- Keep dependencies updated

---

## Performance Optimization

### Caching
- Response caching for frequently asked questions
- Context caching for repeated queries
- Session data caching

### Database Optimization
- Proper indexing on frequently queried fields
- Connection pooling
- Query optimization

### Vector Database
- Efficient similarity search
- Proper collection configuration
- Batch processing for embeddings

---

## Error Codes

| Code | Description |
|------|-------------|
| `INVALID_INPUT` | Input validation failed |
| `SESSION_NOT_FOUND` | Session ID not found |
| `FILE_TOO_LARGE` | Uploaded file exceeds size limit |
| `UNSUPPORTED_FORMAT` | File format not supported |
| `RATE_LIMIT_EXCEEDED` | Too many requests |
| `PROCESSING_ERROR` | Error during document processing |
| `EMBEDDING_ERROR` | Error generating embeddings |
| `CONTEXT_RETRIEVAL_ERROR` | Error retrieving context |
| `AI_SERVICE_ERROR` | Error from AI service |

---

## SDK Examples

### Python SDK Example
```python
import requests

class RAGChatbotClient:
    def __init__(self, base_url="http://localhost:8000/api"):
        self.base_url = base_url
        self.session_id = None
    
    def chat(self, message):
        response = requests.post(
            f"{self.base_url}/chat/rag",
            json={
                "message": message,
                "session_id": self.session_id
            }
        )
        data = response.json()
        self.session_id = data["session_id"]
        return data["response"]
    
    def upload_file(self, file_path):
        with open(file_path, 'rb') as f:
            response = requests.post(
                f"{self.base_url}/chat/file",
                files={"file": f}
            )
        return response.json()

# Usage
client = RAGChatbotClient()
response = client.chat("What is machine learning?")
print(response)
```

### JavaScript SDK Example
```javascript
class RAGChatbotClient {
    constructor(baseUrl = 'http://localhost:8000/api') {
        this.baseUrl = baseUrl;
        this.sessionId = null;
    }
    
    async chat(message) {
        const response = await fetch(`${this.baseUrl}/chat/rag`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                message,
                session_id: this.sessionId
            })
        });
        
        const data = await response.json();
        this.sessionId = data.session_id;
        return data.response;
    }
    
    async uploadFile(file) {
        const formData = new FormData();
        formData.append('file', file);
        
        const response = await fetch(`${this.baseUrl}/chat/file`, {
            method: 'POST',
            body: formData
        });
        
        return response.json();
    }
}

// Usage
const client = new RAGChatbotClient();
const response = await client.chat('What is machine learning?');
console.log(response);
```

---

## Changelog

### Version 1.0.0
- Initial release with RAG chat capabilities
- Voice processing support
- File upload functionality
- Document ingestion pipeline
- Security enhancements
- Performance optimizations

---

## Support

For technical support or questions:
- GitHub Issues: [Repository Issues](https://github.com/your-repo/issues)
- Documentation: [Full Documentation](https://docs.your-domain.com)
- Email: support@your-domain.com