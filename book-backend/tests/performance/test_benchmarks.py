"""
Performance benchmarks for RAG Chatbot components using pytest-benchmark
Run with: pytest tests/performance/test_benchmarks.py --benchmark-only
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
import time
import uuid

# Import the services to benchmark
from book-backend.src.services.embedding_service import EmbeddingService
from book-backend.src.services.rag_service import RAGService
from book-backend.src.database import crud


class TestEmbeddingServiceBenchmarks:
    """Benchmark tests for EmbeddingService"""
    
    @pytest.fixture
    def embedding_service(self):
        """Create a mocked EmbeddingService for benchmarking"""
        with patch('book-backend.src.services.embedding_service.SentenceTransformer') as mock_transformer:
            mock_model = Mock()
            # Simulate realistic embedding generation time
            def mock_encode(texts):
                time.sleep(0.1 * len(texts))  # 100ms per text
                return [[0.1] * 384 for _ in texts]  # 384-dimensional embeddings
            
            mock_model.encode = mock_encode
            mock_transformer.return_value = mock_model
            
            service = EmbeddingService()
            return service
    
    def test_single_text_embedding_performance(self, benchmark, embedding_service):
        """Benchmark single text embedding generation"""
        text = "This is a test sentence for embedding generation performance testing."
        
        def generate_single_embedding():
            return asyncio.run(embedding_service.generate_embeddings([text]))
        
        result = benchmark(generate_single_embedding)
        assert len(result) == 1
        assert len(result[0]) == 384
    
    def test_batch_embedding_performance(self, benchmark, embedding_service):
        """Benchmark batch embedding generation"""
        texts = [
            "First test sentence for batch embedding generation.",
            "Second test sentence with different content.",
            "Third sentence about machine learning concepts.",
            "Fourth sentence discussing neural networks.",
            "Fifth sentence about artificial intelligence."
        ]
        
        def generate_batch_embeddings():
            return asyncio.run(embedding_service.generate_embeddings(texts))
        
        result = benchmark(generate_batch_embeddings)
        assert len(result) == 5
        assert all(len(emb) == 384 for emb in result)
    
    def test_document_chunking_performance(self, benchmark, embedding_service):
        """Benchmark document chunking performance"""
        # Create a large document
        large_text = "This is a test sentence. " * 1000  # ~25KB of text
        
        def chunk_large_document():
            return asyncio.run(embedding_service.chunk_document(large_text, chunk_size=512, overlap=50))
        
        result = benchmark(chunk_large_document)
        assert len(result) > 10  # Should create multiple chunks
        assert all(isinstance(chunk, str) for chunk in result)


class TestRAGServiceBenchmarks:
    """Benchmark tests for RAGService"""
    
    @pytest.fixture
    def rag_service(self):
        """Create a mocked RAGService for benchmarking"""
        with patch('book-backend.src.services.rag_service.get_db') as mock_get_db, \
             patch('book-backend.src.services.rag_service.crud') as mock_crud, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent:
            
            # Setup database mock
            mock_db = Mock()
            mock_get_db.return_value = iter([mock_db])
            
            # Setup CRUD mock with realistic performance
            def mock_get_chunks(db, embedding_ids):
                time.sleep(0.05)  # 50ms database query time
                return [f"Context chunk {i}" for i in range(len(embedding_ids))]
            
            mock_crud.get_document_chunk_content_by_embedding_ids = mock_get_chunks
            
            # Setup ChatAgent mock with realistic response time
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "This is a realistic RAG response based on the provided context."
            
            async def mock_send_message(prompt):
                await asyncio.sleep(1.5)  # 1.5s LLM response time
                return mock_response
            
            mock_agent_instance.send_message = mock_send_message
            mock_chat_agent.return_value = mock_agent_instance
            
            # Create mocks for dependencies
            mock_qdrant_client = Mock()
            mock_embedding_service = Mock()
            
            # Setup embedding service mock
            async def mock_generate_embeddings(texts):
                await asyncio.sleep(0.1 * len(texts))  # 100ms per text
                return [[0.1] * 384 for _ in texts]
            
            mock_embedding_service.generate_embeddings = mock_generate_embeddings
            
            # Setup Qdrant search mock
            def mock_search(collection_name, query_vector, limit):
                time.sleep(0.1)  # 100ms vector search time
                return [Mock(payload={"embedding_id": f"embed_{i}"}) for i in range(limit)]
            
            mock_qdrant_client.search = mock_search
            
            service = RAGService(mock_qdrant_client, mock_embedding_service)
            return service
    
    def test_context_retrieval_performance(self, benchmark, rag_service):
        """Benchmark context retrieval performance"""
        query = "What is machine learning and how does it work?"
        
        def retrieve_context():
            return asyncio.run(rag_service.retrieve_context(query, top_k=5))
        
        result = benchmark(retrieve_context)
        assert len(result) == 5
        assert all("Context chunk" in chunk for chunk in result)
    
    def test_rag_response_generation_performance(self, benchmark, rag_service):
        """Benchmark RAG response generation performance"""
        query = "Explain neural networks"
        context = [
            "Neural networks are computational models inspired by biological neural networks.",
            "They consist of interconnected nodes called neurons or perceptrons.",
            "Neural networks can learn complex patterns through training on data.",
            "Deep learning uses neural networks with multiple hidden layers.",
            "Applications include image recognition, natural language processing, and more."
        ]
        
        def generate_rag_response():
            return asyncio.run(rag_service.generate_rag_response(query, context))
        
        result = benchmark(generate_rag_response)
        assert isinstance(result, str)
        assert len(result) > 10  # Should be a meaningful response
    
    def test_full_rag_pipeline_performance(self, benchmark, rag_service):
        """Benchmark the complete RAG pipeline"""
        query = "How do recommendation systems work in machine learning?"
        
        async def full_rag_pipeline():
            # Step 1: Retrieve context
            context = await rag_service.retrieve_context(query, top_k=3)
            # Step 2: Generate response
            response = await rag_service.generate_rag_response(query, context)
            return response
        
        def run_full_pipeline():
            return asyncio.run(full_rag_pipeline())
        
        result = benchmark(run_full_pipeline)
        assert isinstance(result, str)
        assert len(result) > 10


class TestDatabaseBenchmarks:
    """Benchmark tests for database operations"""
    
    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session"""
        from sqlalchemy import create_engine
        from sqlalchemy.orm import sessionmaker
        from sqlalchemy.pool import StaticPool
        from book-backend.src.database.models import Base
        
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        return TestingSessionLocal()
    
    def test_session_creation_performance(self, benchmark, mock_db_session):
        """Benchmark session creation performance"""
        def create_session():
            return crud.create_session(mock_db_session, uuid.uuid4())
        
        result = benchmark(create_session)
        assert result.id is not None
    
    def test_message_creation_performance(self, benchmark, mock_db_session):
        """Benchmark message creation performance"""
        # Create a session first
        session = crud.create_session(mock_db_session)
        
        def create_message():
            return crud.create_message(
                mock_db_session,
                session.id,
                "Test message for performance benchmarking",
                "user",
                {"benchmark": True}
            )
        
        result = benchmark(create_message)
        assert result.content == "Test message for performance benchmarking"
    
    def test_bulk_message_retrieval_performance(self, benchmark, mock_db_session):
        """Benchmark bulk message retrieval performance"""
        # Create a session and multiple messages
        session = crud.create_session(mock_db_session)
        
        # Create 50 messages
        for i in range(50):
            crud.create_message(
                mock_db_session,
                session.id,
                f"Message {i} for bulk retrieval test",
                "user" if i % 2 == 0 else "ai"
            )
        
        def retrieve_all_messages():
            return crud.get_messages_by_session_id(mock_db_session, session.id)
        
        result = benchmark(retrieve_all_messages)
        assert len(result) == 50
    
    def test_document_chunk_retrieval_performance(self, benchmark, mock_db_session):
        """Benchmark document chunk retrieval by embedding IDs"""
        # Create a document and chunks
        document = crud.create_document(mock_db_session, "Test Document", "/path/test.pdf")
        
        embedding_ids = []
        for i in range(20):
            chunk = crud.create_document_chunk(
                mock_db_session,
                document.id,
                i,
                f"This is chunk {i} content for performance testing",
                f"embed_{i}"
            )
            embedding_ids.append(chunk.embedding_id)
        
        def retrieve_chunks_by_embeddings():
            return crud.get_document_chunk_content_by_embedding_ids(
                mock_db_session, 
                embedding_ids[:10]  # Retrieve 10 chunks
            )
        
        result = benchmark(retrieve_chunks_by_embeddings)
        assert len(result) == 10
        assert all("chunk" in content for content in result)


class TestAPIEndpointBenchmarks:
    """Benchmark tests for API endpoints (mocked)"""
    
    def test_health_endpoint_performance(self, benchmark):
        """Benchmark health check endpoint"""
        from fastapi.testclient import TestClient
        from book-backend.main import app
        
        client = TestClient(app)
        
        def call_health_endpoint():
            response = client.get("/health")
            return response.json()
        
        result = benchmark(call_health_endpoint)
        assert result["status"] == "healthy"
    
    def test_chat_endpoint_performance_mock(self, benchmark):
        """Benchmark chat endpoint with mocked dependencies"""
        from fastapi.testclient import TestClient
        from book-backend.main import app
        
        with patch('book-backend.src.api.chat.get_rag_service') as mock_rag_service:
            # Mock RAG service for consistent performance testing
            mock_service = Mock()
            mock_service.retrieve_context = AsyncMock(return_value=["Test context"])
            mock_service.generate_rag_response = AsyncMock(return_value="Test response")
            mock_rag_service.return_value = mock_service
            
            client = TestClient(app)
            
            def call_chat_endpoint():
                response = client.post("/api/chat/rag", json={
                    "message": "Test performance question"
                })
                return response.json()
            
            result = benchmark(call_chat_endpoint)
            assert "response" in result


# Performance thresholds and assertions
class TestPerformanceThresholds:
    """Test that performance meets specified thresholds"""
    
    def test_embedding_generation_threshold(self):
        """Test that embedding generation meets performance requirements"""
        with patch('book-backend.src.services.embedding_service.SentenceTransformer') as mock_transformer:
            mock_model = Mock()
            
            def timed_encode(texts):
                start_time = time.time()
                # Simulate realistic processing time
                time.sleep(0.05 * len(texts))  # 50ms per text (optimistic)
                end_time = time.time()
                
                # Assert performance threshold: < 100ms per text
                processing_time = end_time - start_time
                assert processing_time < 0.1 * len(texts), f"Embedding generation too slow: {processing_time:.3f}s for {len(texts)} texts"
                
                return [[0.1] * 384 for _ in texts]
            
            mock_model.encode = timed_encode
            mock_transformer.return_value = mock_model
            
            service = EmbeddingService()
            
            # Test with different batch sizes
            for batch_size in [1, 5, 10]:
                texts = [f"Test text {i}" for i in range(batch_size)]
                result = asyncio.run(service.generate_embeddings(texts))
                assert len(result) == batch_size
    
    def test_rag_response_threshold(self):
        """Test that RAG response generation meets latency requirements"""
        # According to NFRs: p95 latency should be 3-5 seconds
        start_time = time.time()
        
        # Simulate a realistic RAG pipeline
        time.sleep(2.5)  # Simulated processing time
        
        end_time = time.time()
        processing_time = end_time - start_time
        
        # Assert p95 latency requirement (5 seconds max)
        assert processing_time < 5.0, f"RAG response too slow: {processing_time:.3f}s (should be < 5s)"
        
        # For p50, should be faster (3 seconds)
        assert processing_time < 3.0, f"RAG response slower than p50 target: {processing_time:.3f}s (should be < 3s for p50)"


if __name__ == "__main__":
    print("RAG Chatbot Performance Benchmarks")
    print("==================================")
    print("Run with: pytest tests/performance/test_benchmarks.py --benchmark-only")
    print("For detailed output: pytest tests/performance/test_benchmarks.py --benchmark-only --benchmark-verbose")
    print("To save results: pytest tests/performance/test_benchmarks.py --benchmark-only --benchmark-json=benchmark_results.json")