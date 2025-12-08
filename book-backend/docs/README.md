# RAG Chatbot Backend Documentation

Welcome to the RAG (Retrieval-Augmented Generation) Chatbot Backend documentation. This system provides intelligent conversational AI capabilities enhanced with document retrieval and context-aware responses.

## 📚 Documentation Index

### API Documentation
- **[API Reference](./api_documentation.md)** - Complete API endpoint documentation
- **[Authentication Guide](./authentication.md)** - Authentication and authorization
- **[Rate Limiting](./rate_limiting.md)** - API rate limiting policies

### Development Guides
- **[Setup Guide](./setup.md)** - Development environment setup
- **[Architecture Overview](./architecture.md)** - System architecture and design
- **[Database Schema](./database_schema.md)** - Database models and relationships

### Deployment
- **[Deployment Guide](./deployment.md)** - Production deployment instructions
- **[Environment Configuration](./environment.md)** - Environment variables and configuration
- **[Monitoring](./monitoring.md)** - System monitoring and logging

### Testing
- **[Testing Guide](./testing.md)** - Testing strategies and examples
- **[Load Testing](./load_testing.md)** - Performance and load testing
- **[Security Testing](./security_testing.md)** - Security validation

### Advanced Topics
- **[Performance Optimization](./performance.md)** - Performance tuning guide
- **[Security Best Practices](./security.md)** - Security implementation guide
- **[Troubleshooting](./troubleshooting.md)** - Common issues and solutions

## 🚀 Quick Start

### Prerequisites
- Python 3.13+
- PostgreSQL (Neon Serverless)
- Qdrant Vector Database
- Gemini API Key

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd book-backend

# Install dependencies
uv sync

# Setup environment variables
cp .env.example .env
# Edit .env with your configuration

# Run database migrations
uv run alembic upgrade head

# Start the development server
uv run uvicorn main:app --reload
```

### First API Call
```bash
curl -X POST "http://localhost:8000/api/chat/rag" \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello, how can you help me?"}'
```

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Frontend      │    │   Backend API   │    │   Vector DB     │
│   (React)       │◄──►│   (FastAPI)     │◄──►│   (Qdrant)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │   Database      │    │   AI Service    │
                       │   (PostgreSQL)  │    │   (Gemini)      │
                       └─────────────────┘    └─────────────────┘
```

## 🔧 Core Features

### RAG (Retrieval-Augmented Generation)
- **Context Retrieval**: Intelligent document search using vector embeddings
- **Response Generation**: AI-powered responses enhanced with retrieved context
- **Session Management**: Persistent conversation history

### Document Processing
- **Multi-format Support**: PDF, DOCX, EPUB, TXT, Markdown
- **Intelligent Chunking**: Context-aware document segmentation
- **Embedding Generation**: Vector representations for semantic search

### Voice Processing
- **Speech-to-Text**: Whisper-based audio transcription
- **Voice Chat**: Complete voice interaction pipeline
- **Audio Format Support**: WAV, MP3, M4A, FLAC

### File Upload
- **Temporary Processing**: Upload documents for immediate context
- **Security Validation**: File type and content validation
- **Real-time Processing**: Instant document analysis

## 📊 Performance Metrics

### Response Times (Target)
- Chat Response: < 2 seconds
- Context Retrieval: < 500ms
- Embedding Generation: < 200ms
- File Processing: < 5 seconds

### Throughput (Target)
- Concurrent Users: 100+
- Requests per Second: 50+
- File Uploads: 10/hour per user

### Resource Usage
- Memory: < 2GB
- CPU: < 80%
- Database Connections: < 50

## 🔒 Security Features

### Input Validation
- XSS Protection
- SQL Injection Prevention
- File Upload Validation
- Rate Limiting

### Data Protection
- Secure Session Management
- Data Sanitization
- Encrypted Storage
- CORS Configuration

### Monitoring
- Request Logging
- Error Tracking
- Performance Monitoring
- Security Alerts

## 🧪 Testing

### Test Coverage
- Unit Tests: Core business logic
- Integration Tests: API endpoints
- End-to-End Tests: Complete user flows
- Load Tests: Performance validation

### Running Tests
```bash
# Unit tests
uv run pytest tests/unit/ -v

# Integration tests
uv run pytest tests/integration/ -v

# Load tests
python tests/load/run_load_tests.py
```

## 📈 Monitoring and Observability

### Logging
- Structured logging with JSON format
- Log levels: DEBUG, INFO, WARNING, ERROR
- Request/response logging
- Performance metrics

### Health Checks
- `/health` endpoint for basic health
- Database connectivity checks
- External service validation
- Resource usage monitoring

### Metrics
- Response time tracking
- Error rate monitoring
- Resource utilization
- User activity analytics

## 🚀 Deployment

### Development
```bash
uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Production
```bash
# Using Docker
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 rag-chatbot-backend

# Using systemd
sudo systemctl start rag-chatbot
sudo systemctl enable rag-chatbot
```

### Environment Variables
```bash
# Required
NEON_DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
GEMINI_API_KEY=...

# Optional
ENVIRONMENT=production
LOG_LEVEL=INFO
SECURITY_SALT=your-secret-salt
```

## 🤝 Contributing

### Development Workflow
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

### Code Standards
- Follow PEP 8 style guide
- Add type hints
- Write comprehensive tests
- Update documentation

### Commit Messages
```
feat: add new RAG endpoint
fix: resolve embedding generation issue
docs: update API documentation
test: add integration tests for chat
```

## 📞 Support

### Getting Help
- **Documentation**: Check this documentation first
- **GitHub Issues**: Report bugs and request features
- **Discussions**: Community discussions and Q&A

### Common Issues
- **Database Connection**: Check connection string and credentials
- **Vector Database**: Verify Qdrant service is running
- **API Keys**: Ensure all required API keys are configured
- **Dependencies**: Run `uv sync` to update dependencies

### Performance Issues
- **Slow Responses**: Check database indexes and query optimization
- **Memory Usage**: Monitor embedding model memory consumption
- **Rate Limits**: Verify rate limiting configuration

## 📋 Roadmap

### Version 1.1
- [ ] Better-Auth integration
- [ ] User personalization
- [ ] Multi-language support
- [ ] Advanced caching

### Version 1.2
- [ ] Real-time chat with WebSockets
- [ ] Advanced analytics
- [ ] Plugin system
- [ ] Mobile SDK

### Version 2.0
- [ ] Multi-modal support (images, videos)
- [ ] Advanced AI models
- [ ] Distributed architecture
- [ ] Enterprise features

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](../LICENSE) file for details.

---

**Last Updated**: December 2024  
**Version**: 1.0.0  
**Maintainers**: Development Team