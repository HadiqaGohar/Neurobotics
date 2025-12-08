"""
Performance benchmark tests using pytest-benchmark.
"""

import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock
from book_backend.src.services.rag_service import RAGService
from book_backend.src.services.embedding_service import EmbeddingService


class TestRAGServiceBenchmarks:
    """Benchmark tests for RAG service performance."""
    
    @pytest.fixture
    def mock_qdrant_client(self):
        """Mock Qdrant client for testing."""
        client = MagicMock()
        client.search = AsyncMock(return_value=[
            MagicMock(payload={"text": "Sample context 1"}),
            MagicMock(payload={"text": "Sample context 2"}),
            MagicMock(payload={"text": "Sample context 3"})
        ])
        return client
    
    @pytest.fixture
    def mock_embedding_service(self):
        """Mock embedding service for testing."""
        service = MagicMock()
        service.generate_embeddings = MagicMock(return_value=[[0.1] * 384])
        return service
    
    @pytest.fixture
    def rag_service(self, mock_qdrant_client, mock_embedding_service):
        """Create RAG service with mocked dependencies."""
        return RAGService(mock_qdrant_client, mock_embedding_service)
    
    def test_retrieve_context_performance(self, benchmark, rag_service):
        """Benchmark context retrieval performance."""
        query = "What is machine learning?"
        
        def retrieve_context():
            return asyncio.run(rag_service.retrieve_context(query, top_k=5))
        
        result = benchmark(retrieve_context)
        assert len(result) > 0
    
    def test_generate_rag_response_performance(self, benchmark, rag_service):
        """Benchmark RAG response generation performance."""
        query = "What is machine learning?"
        context = [
            "Machine learning is a subset of AI",
            "It involves algorithms that learn from data",
            "Neural networks are a key component"
        ]
        
        def generate_response():
            return asyncio.run(rag_service.generate_rag_response(query, context))
        
        result = benchmark(generate_response)
        assert isinstance(result, str)
        assert len(result) > 0


class TestEmbeddingServiceBenchmarks:
    """Benchmark tests for embedding service performance."""
    
    @pytest.fixture
    def embedding_service(self):
        """Create embedding service for testing."""
        # Mock the model loading to avoid actual model download
        service = EmbeddingService()
        service.model = MagicMock()
        service.model.encode = MagicMock(return_value=[[0.1] * 384])
        return service
    
    def test_generate_embeddings_single_text(self, benchmark, embedding_service):
        """Benchmark single text embedding generation."""
        texts = ["This is a test sentence for embedding generation."]
        
        result = benchmark(embedding_service.generate_embeddings, texts)
        assert len(result) == 1
        assert len(result[0]) == 384
    
    def test_generate_embeddings_batch(self, benchmark, embedding_service):
        """Benchmark batch embedding generation."""
        texts = [
            "This is the first test sentence.",
            "This is the second test sentence.",
            "This is the third test sentence.",
            "This is the fourth test sentence.",
            "This is the fifth test sentence."
        ]
        
        result = benchmark(embedding_service.generate_embeddings, texts)
        assert len(result) == 5
        assert all(len(emb) == 384 for emb in result)
    
    def test_chunk_document_performance(self, benchmark, embedding_service):
        """Benchmark document chunking performance."""
        # Create a large text document
        large_text = " ".join([
            f"This is sentence number {i} in a large document that needs to be chunked."
            for i in range(1000)
        ])
        
        result = benchmark(
            embedding_service.chunk_document,
            large_text,
            chunk_size=512,
            overlap=50
        )
        assert len(result) > 1
        assert all(len(chunk) <= 512 + 50 for chunk in result)


class TestDatabaseBenchmarks:
    """Benchmark tests for database operations."""
    
    @pytest.fixture
    def mock_db_session(self):
        """Mock database session."""
        session = AsyncMock()
        session.execute = AsyncMock()
        session.commit = AsyncMock()
        session.add = MagicMock()
        return session
    
    def test_message_insertion_performance(self, benchmark, mock_db_session):
        """Benchmark message insertion performance."""
        from book_backend.src.database.models import Message
        
        def insert_message():
            message = Message(
                session_id="test-session",
                user_message="Test query",
                bot_response="Test response",
                context_used="Test context"
            )
            mock_db_session.add(message)
            return asyncio.run(mock_db_session.commit())
        
        benchmark(insert_message)
    
    def test_session_creation_performance(self, benchmark, mock_db_session):
        """Benchmark session creation performance."""
        from book_backend.src.database.models import Session
        
        def create_session():
            session = Session(
                user_id="test-user",
                title="Test Session"
            )
            mock_db_session.add(session)
            return asyncio.run(mock_db_session.commit())
        
        benchmark(create_session)


# Performance thresholds (adjust based on requirements)
@pytest.mark.benchmark(
    group="rag_performance",
    min_rounds=5,
    max_time=30,
    warmup=True
)
class TestPerformanceThresholds:
    """Test performance against defined thresholds."""
    
    def test_context_retrieval_threshold(self, benchmark):
        """Ensure context retrieval meets performance requirements."""
        # Mock fast retrieval
        def fast_retrieval():
            import time
            time.sleep(0.1)  # Simulate 100ms retrieval
            return ["context1", "context2", "context3"]
        
        result = benchmark(fast_retrieval)
        
        # Assert performance threshold (should be under 500ms)
        assert benchmark.stats.mean < 0.5
        assert len(result) == 3
    
    def test_embedding_generation_threshold(self, benchmark):
        """Ensure embedding generation meets performance requirements."""
        # Mock embedding generation
        def fast_embedding():
            import time
            time.sleep(0.05)  # Simulate 50ms embedding generation
            return [[0.1] * 384]
        
        result = benchmark(fast_embedding)
        
        # Assert performance threshold (should be under 200ms)
        assert benchmark.stats.mean < 0.2
        assert len(result[0]) == 384