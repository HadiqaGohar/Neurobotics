# Development Setup Guide

This guide will help you set up the RAG Chatbot Backend for local development.

## Prerequisites

### System Requirements
- **Python**: 3.13 or higher
- **Node.js**: 18+ (for frontend development)
- **Git**: Latest version
- **Docker**: Optional, for containerized services

### External Services
- **Neon PostgreSQL**: Serverless PostgreSQL database
- **Qdrant**: Vector database for embeddings
- **Google Gemini API**: AI language model access

## Installation Steps

### 1. Clone the Repository
```bash
git clone <repository-url>
cd book-backend
```

### 2. Python Environment Setup

#### Using uv (Recommended)
```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create and activate virtual environment
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
uv sync
```

#### Using pip (Alternative)
```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Configuration

#### Create Environment File
```bash
cp .env.example .env
```

#### Configure Environment Variables
Edit `.env` file with your configuration:

```bash
# Database Configuration
NEON_DATABASE_URL=postgresql://username:password@host:port/database

# Vector Database Configuration
QDRANT_URL=https://your-qdrant-instance.com
QDRANT_API_KEY=your-qdrant-api-key

# AI Service Configuration
GEMINI_API_KEY=your-gemini-api-key

# Security Configuration
SECURITY_SALT=your-random-salt-string
JWT_SECRET_KEY=your-jwt-secret-key

# Application Configuration
ENVIRONMENT=development
LOG_LEVEL=DEBUG
DEBUG=true
```

### 4. Database Setup

#### Initialize Alembic (if not done)
```bash
uv run alembic init alembic
```

#### Create Initial Migration
```bash
uv run alembic revision --autogenerate -m "Initial RAG schema"
```

#### Apply Migrations
```bash
uv run alembic upgrade head
```

#### Verify Database Connection
```bash
uv run python -c "
from book_backend.src.database.database import engine
from sqlalchemy import text
with engine.connect() as conn:
    result = conn.execute(text('SELECT 1'))
    print('Database connection successful!')
"
```

### 5. Vector Database Setup

#### Qdrant Local Installation (Optional)
```bash
# Using Docker
docker run -p 6333:6333 qdrant/qdrant

# Using Docker Compose
cat > docker-compose.yml << EOF
version: '3.8'
services:
  qdrant:
    image: qdrant/qdrant
    ports:
      - "6333:6333"
    volumes:
      - qdrant_data:/qdrant/storage
volumes:
  qdrant_data:
EOF

docker-compose up -d
```

#### Create Qdrant Collection
```bash
uv run python -c "
from book_backend.src.services.qdrant_service import create_collection
import asyncio
asyncio.run(create_collection())
print('Qdrant collection created successfully!')
"
```

### 6. Verify Installation

#### Run Health Check
```bash
uv run python -c "
import asyncio
from book_backend.src.services.embedding_service import EmbeddingService

async def test():
    service = EmbeddingService()
    result = await service.generate_embeddings(['test'])
    print(f'Embedding service working: {len(result[0])} dimensions')

asyncio.run(test())
"
```

#### Start Development Server
```bash
uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

#### Test API Endpoint
```bash
curl -X GET "http://localhost:8000/health"
```

Expected response:
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

## Development Tools

### Code Quality Tools

#### Install Development Dependencies
```bash
uv add --dev black isort flake8 mypy pytest pytest-asyncio pytest-cov
```

#### Code Formatting
```bash
# Format code with Black
uv run black .

# Sort imports with isort
uv run isort .

# Lint with flake8
uv run flake8 .

# Type checking with mypy
uv run mypy .
```

#### Pre-commit Hooks
```bash
# Install pre-commit
uv add --dev pre-commit

# Setup pre-commit hooks
cat > .pre-commit-config.yaml << EOF
repos:
  - repo: https://github.com/psf/black
    rev: 23.12.1
    hooks:
      - id: black
  - repo: https://github.com/pycqa/isort
    rev: 5.13.2
    hooks:
      - id: isort
  - repo: https://github.com/pycqa/flake8
    rev: 7.0.0
    hooks:
      - id: flake8
EOF

uv run pre-commit install
```

### Testing Setup

#### Run Tests
```bash
# Unit tests
uv run pytest tests/unit/ -v

# Integration tests
uv run pytest tests/integration/ -v

# All tests with coverage
uv run pytest --cov=book_backend tests/ --cov-report=html
```

#### Load Testing
```bash
# Install load testing dependencies
uv add --dev locust pytest-benchmark

# Run load tests
python tests/load/run_load_tests.py
```

### Database Management

#### Database Migrations
```bash
# Create new migration
uv run alembic revision --autogenerate -m "Description of changes"

# Apply migrations
uv run alembic upgrade head

# Rollback migration
uv run alembic downgrade -1

# View migration history
uv run alembic history
```

#### Database Reset (Development Only)
```bash
# Drop all tables and recreate
uv run python -c "
from book_backend.src.database.database import engine
from book_backend.src.database.models import Base
Base.metadata.drop_all(engine)
Base.metadata.create_all(engine)
print('Database reset complete!')
"

# Reapply migrations
uv run alembic stamp head
```

## IDE Configuration

### VS Code Setup

#### Recommended Extensions
- Python
- Pylance
- Black Formatter
- isort
- GitLens
- REST Client

#### VS Code Settings
Create `.vscode/settings.json`:
```json
{
  "python.defaultInterpreterPath": "./.venv/bin/python",
  "python.formatting.provider": "black",
  "python.linting.enabled": true,
  "python.linting.flake8Enabled": true,
  "python.linting.mypyEnabled": true,
  "editor.formatOnSave": true,
  "editor.codeActionsOnSave": {
    "source.organizeImports": true
  }
}
```

#### Launch Configuration
Create `.vscode/launch.json`:
```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "FastAPI Server",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/.venv/bin/uvicorn",
      "args": ["main:app", "--reload", "--host", "0.0.0.0", "--port", "8000"],
      "console": "integratedTerminal",
      "cwd": "${workspaceFolder}",
      "env": {
        "PYTHONPATH": "${workspaceFolder}"
      }
    }
  ]
}
```

### PyCharm Setup

#### Project Configuration
1. Open project in PyCharm
2. Configure Python interpreter: `.venv/bin/python`
3. Mark `book_backend` as source root
4. Configure run configuration for FastAPI

#### Code Style
1. Go to Settings → Editor → Code Style → Python
2. Set line length to 88 (Black default)
3. Enable "Optimize imports on the fly"

## Troubleshooting

### Common Issues

#### Import Errors
```bash
# Ensure PYTHONPATH is set
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# Or add to .env file
echo "PYTHONPATH=$(pwd)" >> .env
```

#### Database Connection Issues
```bash
# Test database connection
uv run python -c "
import os
from sqlalchemy import create_engine, text
engine = create_engine(os.getenv('NEON_DATABASE_URL'))
with engine.connect() as conn:
    result = conn.execute(text('SELECT version()'))
    print(result.fetchone())
"
```

#### Qdrant Connection Issues
```bash
# Test Qdrant connection
uv run python -c "
import os
from qdrant_client import QdrantClient
client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)
print(client.get_collections())
"
```

#### Embedding Model Issues
```bash
# Clear model cache and re-download
rm -rf ~/.cache/torch/sentence_transformers/

# Test embedding generation
uv run python -c "
from book_backend.src.services.embedding_service import EmbeddingService
import asyncio
async def test():
    service = EmbeddingService()
    result = await service.generate_embeddings(['test'])
    print(f'Success: {len(result)} embeddings generated')
asyncio.run(test())
"
```

### Performance Issues

#### Memory Usage
```bash
# Monitor memory usage
pip install memory-profiler
uv run python -m memory_profiler your_script.py
```

#### Database Performance
```bash
# Enable query logging
export SQLALCHEMY_ECHO=true

# Monitor slow queries
tail -f app.log | grep "slow query"
```

### Development Workflow

#### Daily Development
```bash
# Start development session
source .venv/bin/activate
uv run uvicorn main:app --reload

# In another terminal - run tests
uv run pytest tests/ -v --tb=short

# Format code before commit
uv run black .
uv run isort .
```

#### Before Committing
```bash
# Run full test suite
uv run pytest tests/ --cov=book_backend

# Check code quality
uv run flake8 .
uv run mypy .

# Update documentation if needed
# Commit changes
git add .
git commit -m "feat: description of changes"
```

## Next Steps

After completing the setup:

1. **Read the [API Documentation](./api_documentation.md)** to understand available endpoints
2. **Review the [Architecture Overview](./architecture.md)** to understand system design
3. **Check the [Testing Guide](./testing.md)** for testing best practices
4. **Explore the [Performance Guide](./performance.md)** for optimization tips

## Getting Help

If you encounter issues during setup:

1. Check the [Troubleshooting Guide](./troubleshooting.md)
2. Search existing [GitHub Issues](https://github.com/your-repo/issues)
3. Create a new issue with detailed error information
4. Join our development discussions

---

**Happy Coding!** 🚀