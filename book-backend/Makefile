# Book Backend API Makefile

.PHONY: help install dev test clean lint format init-db ingest-content

help:  ## Show this help message
	@echo "Available commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

install:  ## Install dependencies
	uv pip install -r requirements.txt

dev:  ## Start development server
	python scripts/start_server.py

test:  ## Run tests
	pytest -v

test-cov:  ## Run tests with coverage
	pytest --cov=app --cov-report=html --cov-report=term

lint:  ## Run linting
	flake8 app tests
	black --check app tests

format:  ## Format code
	black app tests
	isort app tests

init-db:  ## Initialize database tables
	python scripts/init_db.py

ingest-content:  ## Ingest all published books into vector database
	python scripts/ingest_content.py --all-published

ingest-book:  ## Ingest specific book (usage: make ingest-book BOOK_ID=1)
	python scripts/ingest_content.py --book-id $(BOOK_ID)

list-books:  ## List all books in database
	python scripts/ingest_content.py --list

status:  ## Check ingestion status
	python scripts/ingest_content.py --status

clean:  ## Clean up temporary files
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	rm -rf .pytest_cache
	rm -rf htmlcov
	rm -rf dist
	rm -rf build

setup:  ## Full setup (install + init-db)
	make install
	make init-db

docker-build:  ## Build Docker image
	docker build -t book-backend-api .

docker-run:  ## Run Docker container
	docker run -p 8001:8001 --env-file .env book-backend-api

# Development helpers
logs:  ## Show application logs
	tail -f logs/app.log

error-logs:  ## Show error logs
	tail -f logs/error.log

check-env:  ## Check environment variables
	@echo "Checking required environment variables..."
	@python -c "from app.core.config import get_settings; settings = get_settings(); print('✓ Configuration loaded successfully')"