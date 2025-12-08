# Book Backend API

AI-powered Book Creation and Management API with RAG (Retrieval-Augmented Generation) Chatbot capabilities.

## Features

- 📚 **Book Content Management**: Create, read, update, and delete books and chapters
- 🔐 **Secure Authentication**: User registration and login with JWT tokens
- 🎯 **Content Personalization**: Personalized content based on user preferences and experience level
- 🌍 **Multi-language Support**: Content translation (currently supports Urdu)
- 🤖 **RAG Chatbot**: Intelligent chatbot that answers questions about book content using vector search
- 📊 **Vector Database**: Qdrant integration for semantic search capabilities
- 🚀 **FastAPI**: Modern, fast web framework with automatic API documentation

## Tech Stack

- **Framework**: FastAPI
- **Database**: Neon Serverless Postgres
- **Vector Database**: Qdrant
- **AI Services**: OpenAI Agents SDK, Gemini API
- **Authentication**: JWT tokens with Better-Auth integration
- **ORM**: SQLAlchemy
- **Testing**: pytest
- **Documentation**: Automatic OpenAPI/Swagger docs

## Quick Start

### Prerequisites

- Python 3.8+
- uv (Python package manager)
- PostgreSQL database (Neon recommended)
- Qdrant instance
- OpenAI API key
- Google Gemini API key

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd book-backend
   ```

2. **Install dependencies using uv**
   ```bash
   uv pip install -r requirements.txt
   ```

3. **Set up environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

4. **Run database migrations**
   ```bash
   python -c "from app.core.database import create_tables; create_tables()"
   ```

5. **Start the development server**
   ```bash
   uvicorn app.main:app --reload --port 8001
   ```

The API will be available at `http://localhost:8001`

## Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
# Database
NEON_DATABASE_URL=postgresql://username:password@host:port/database

# AI Services
OPENAI_API_KEY=your_openai_api_key
GEMINI_API_KEY=your_gemini_api_key

# Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key

# Authentication
JWT_SECRET_KEY=your_jwt_secret_key
BETTER_AUTH_API_KEY=your_better_auth_key

# Application
DEBUG=true
ENVIRONMENT=development
```

## API Documentation

Once the server is running, you can access:

- **Interactive API Docs (Swagger UI)**: http://localhost:8001/api/v1/docs
- **Alternative API Docs (ReDoc)**: http://localhost:8001/api/v1/redoc
- **OpenAPI JSON**: http://localhost:8001/api/v1/openapi.json

## API Endpoints

### Authentication
- `POST /api/v1/auth/register` - Register a new user
- `POST /api/v1/auth/login` - Login user
- `GET /api/v1/auth/me` - Get current user profile
- `PUT /api/v1/auth/preferences` - Update user preferences

### Content Management
- `GET /api/v1/content/books` - List all books
- `POST /api/v1/content/books` - Create a new book
- `GET /api/v1/content/books/{id}` - Get book by ID
- `PUT /api/v1/content/books/{id}` - Update book
- `DELETE /api/v1/content/books/{id}` - Delete book
- `GET /api/v1/content/books/{id}/chapters` - Get book chapters
- `POST /api/v1/content/chapters` - Create a new chapter
- `GET /api/v1/content/chapters/{id}` - Get chapter by ID
- `PUT /api/v1/content/chapters/{id}` - Update chapter
- `DELETE /api/v1/content/chapters/{id}` - Delete chapter
- `GET /api/v1/content/chapters/{id}/personalized` - Get personalized chapter content
- `GET /api/v1/content/chapters/{id}/translated` - Get translated chapter content

### Chatbot
- `POST /api/v1/chatbot/sessions` - Create chat session
- `GET /api/v1/chatbot/sessions` - Get user's chat sessions
- `GET /api/v1/chatbot/sessions/{id}` - Get specific chat session
- `GET /api/v1/chatbot/sessions/{id}/messages` - Get session messages
- `POST /api/v1/chatbot/query` - Send chat query with RAG
- `POST /api/v1/chatbot/context` - Send chat query with explicit context
- `GET /api/v1/chatbot/suggestions/{session_id}` - Get chat suggestions
- `DELETE /api/v1/chatbot/sessions/{id}` - Delete chat session

### Health & Status
- `GET /` - Root endpoint with API info
- `GET /health` - Health check endpoint

## Usage Examples

### 1. Register and Login

```bash
# Register a new user
curl -X POST "http://localhost:8001/api/v1/auth/register" \
  -H "Content-Type: application/json" \
  -d '{
    "username": "johndoe",
    "email": "john@example.com",
    "password": "securepassword123",
    "persona": "developer",
    "experience_level": "intermediate"
  }'

# Login
curl -X POST "http://localhost:8001/api/v1/auth/login" \
  -H "Content-Type: application/json" \
  -d '{
    "username": "johndoe",
    "password": "securepassword123"
  }'
```

### 2. Create Book and Chapter

```bash
# Create a book
curl -X POST "http://localhost:8001/api/v1/content/books" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{
    "title": "Introduction to AI",
    "description": "A comprehensive guide to artificial intelligence",
    "author": "Jane Smith",
    "isbn": "1234567890123"
  }'

# Create a chapter
curl -X POST "http://localhost:8001/api/v1/content/chapters" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{
    "book_id": 1,
    "title": "What is AI?",
    "chapter_number": 1,
    "content": "Artificial Intelligence (AI) is..."
  }'
```

### 3. Chat with RAG Bot

```bash
# Create chat session
curl -X POST "http://localhost:8001/api/v1/chatbot/sessions" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "AI Discussion",
    "session_type": "rag",
    "language": "en"
  }'

# Send chat query
curl -X POST "http://localhost:8001/api/v1/chatbot/query" \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": 1,
    "message": "What are the main concepts of AI?",
    "use_rag": true
  }'
```

## Content Ingestion

To populate the vector database with book content for RAG functionality:

```bash
# Ingest all published books
python scripts/ingest_content.py --all-published

# Ingest specific book
python scripts/ingest_content.py --book-id 1

# Check ingestion status
python scripts/ingest_content.py --status

# List all books
python scripts/ingest_content.py --list
```

## Testing

Run the test suite:

```bash
# Run all tests
pytest

# Run unit tests only
pytest tests/unit/

# Run integration tests only
pytest tests/integration/

# Run with coverage
pytest --cov=app

# Run specific test file
pytest tests/unit/test_content_service.py -v
```

## Development

### Project Structure

```
book-backend/
├── app/
│   ├── api/
│   │   └── v1/
│   │       └── endpoints/
│   │           ├── auth.py
│   │           ├── content.py
│   │           └── chatbot.py
│   ├── core/
│   │   ├── config.py
│   │   └── database.py
│   ├── models/
│   │   ├── database.py
│   │   ├── user.py
│   │   ├── content.py
│   │   └── chat.py
│   ├── services/
│   │   ├── auth_service.py
│   │   ├── content_service.py
│   │   ├── chatbot_service.py
│   │   ├── translation_service.py
│   │   ├── qdrant_service.py
│   │   └── content_ingestion_service.py
│   └── main.py
├── scripts/
│   └── ingest_content.py
├── tests/
│   ├── unit/
│   └── integration/
├── logs/
├── requirements.txt
├── pytest.ini
└── README.md
```

### Adding New Features

1. **Create Models**: Add Pydantic models in `app/models/`
2. **Create Services**: Add business logic in `app/services/`
3. **Create Endpoints**: Add API endpoints in `app/api/v1/endpoints/`
4. **Add Tests**: Write tests in `tests/unit/` and `tests/integration/`
5. **Update Documentation**: Update this README and API docs

### Code Style

- Follow PEP 8 guidelines
- Use type hints
- Add docstrings to functions and classes
- Write comprehensive tests
- Use meaningful variable and function names

## Deployment

### Docker (Recommended)

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

EXPOSE 8001

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8001"]
```

### Environment-specific Configuration

- **Development**: Use SQLite for local development
- **Staging**: Use Neon Postgres with debug logging
- **Production**: Use Neon Postgres with optimized settings

## Monitoring and Logging

The application includes comprehensive logging:

- **Console Logs**: Real-time application logs
- **File Logs**: Rotating log files in `logs/` directory
- **Error Logs**: Separate error log file
- **Structured Logging**: JSON format for production

Log levels can be configured via environment variables.

## Security

- JWT token-based authentication
- Password hashing with bcrypt
- Input validation with Pydantic
- CORS configuration for frontend integration
- Environment variable protection
- SQL injection prevention with SQLAlchemy ORM

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Ensure all tests pass
6. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support and questions:

- Create an issue in the repository
- Check the API documentation at `/api/v1/docs`
- Review the test files for usage examples

## Changelog

### v1.0.0
- Initial release
- Book content management
- User authentication
- Content personalization
- Multi-language support
- RAG chatbot integration
- Vector database integration
- Comprehensive API documentation