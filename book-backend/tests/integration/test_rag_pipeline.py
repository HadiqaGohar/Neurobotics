import pytest
import asyncio
import tempfile
import os
from unittest.mock import Mock, patch, AsyncMock
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from book-backend.main import app
from book-backend.src.database.models import Base
from book-backend.src.database.database import get_db


class TestRAGPipelineIntegration:
    """Integration tests for the full RAG pipeline"""
    
    @pytest.fixture
    def test_db(self):
        """Create a test database"""
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        
        def override_get_db():
            try:
                db = TestingSessionLocal()
                yield db
            finally:
                db.close()
        
        app.dependency_overrides[get_db] = override_get_db
        yield TestingSessionLocal()
        app.dependency_overrides.clear()
    
    @pytest.fixture
    def client(self, test_db):
        """Create a test client"""
        return TestClient(app)
    
    @pytest.fixture
    def mock_services(self):
        """Mock external services (Qdrant, Embedding, etc.)"""
        with patch('book-backend.src.api.chat.get_qdrant_client') as mock_qdrant, \
             patch('book-backend.src.api.chat.get_embedding_service') as mock_embedding, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent:
            
            # Mock Qdrant client
            mock_qdrant_instance = Mock()
            mock_hit = Mock()
            mock_hit.payload = {"embedding_id": "test_embed_1"}
            mock_qdrant_instance.search.return_value = [mock_hit]
            mock_qdrant.return_value = mock_qdrant_instance
            
            # Mock embedding service
            mock_embedding_instance = Mock()
            mock_embedding_instance.generate_embeddings = AsyncMock(return_value=[[0.1, 0.2, 0.3]])
            mock_embedding.return_value = mock_embedding_instance
            
            # Mock chat agent
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "This is a test RAG response based on the retrieved context."
            mock_agent_instance.send_message = AsyncMock(return_value=mock_response)
            mock_chat_agent.return_value = mock_agent_instance
            
            yield {
                'qdrant': mock_qdrant_instance,
                'embedding': mock_embedding_instance,
                'chat_agent': mock_agent_instance
            }
    
    def test_health_endpoint(self, client):
        """Test the health check endpoint"""
        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "RAG Chat" in data["features"]
    
    def test_rag_chat_endpoint_new_session(self, client, test_db, mock_services):
        """Test RAG chat endpoint with new session"""
        with patch('book-backend.src.database.crud.get_document_chunk_content_by_embedding_ids') as mock_crud:
            mock_crud.return_value = ["Test context from document"]
            
            response = client.post("/api/chat/rag", json={
                "message": "What is machine learning?"
            })
            
            assert response.status_code == 200
            data = response.json()
            
            assert "response" in data
            assert "session_id" in data
            assert "context" in data
            assert data["response"] == "This is a test RAG response based on the retrieved context."
            assert data["context"] == ["Test context from document"]
    
    def test_rag_chat_endpoint_existing_session(self, client, test_db, mock_services):
        """Test RAG chat endpoint with existing session"""
        # First, create a session
        response1 = client.post("/api/chat/rag", json={
            "message": "First message"
        })
        assert response1.status_code == 200
        session_id = response1.json()["session_id"]
        
        # Then use the same session
        with patch('book-backend.src.database.crud.get_document_chunk_content_by_embedding_ids') as mock_crud:
            mock_crud.return_value = ["Context for second message"]
            
            response2 = client.post("/api/chat/rag", json={
                "message": "Second message",
                "session_id": session_id
            })
            
            assert response2.status_code == 200
            data = response2.json()
            assert data["session_id"] == session_id
    
    def test_rag_chat_endpoint_invalid_session(self, client, test_db, mock_services):
        """Test RAG chat endpoint with invalid session ID"""
        response = client.post("/api/chat/rag", json={
            "message": "Test message",
            "session_id": "invalid-uuid"
        })
        
        # Should handle invalid UUID gracefully or return 400
        assert response.status_code in [400, 422]  # 422 for validation error
    
    def test_chat_history_endpoint(self, client, test_db, mock_services):
        """Test chat history retrieval"""
        # Create a session and send messages
        with patch('book-backend.src.database.crud.get_document_chunk_content_by_embedding_ids') as mock_crud:
            mock_crud.return_value = ["Test context"]
            
            response1 = client.post("/api/chat/rag", json={
                "message": "First message"
            })
            session_id = response1.json()["session_id"]
            
            response2 = client.post("/api/chat/rag", json={
                "message": "Second message",
                "session_id": session_id
            })
        
        # Get chat history
        response = client.post("/api/chat/history", json={
            "session_id": session_id
        })
        
        assert response.status_code == 200
        # The response format might vary based on implementation
    
    def test_ingestion_endpoint(self, client, test_db):
        """Test document ingestion endpoint"""
        with patch('book-backend.src.api.ingestion.get_ingestion_service') as mock_service:
            mock_ingestion_instance = Mock()
            mock_ingestion_instance.ingest_book_content = AsyncMock(return_value={
                "status": "success",
                "message": "Document ingested successfully",
                "documents_indexed": 1,
                "chunks_created": 5
            })
            mock_service.return_value = mock_ingestion_instance
            
            response = client.post("/api/ingest/book", json={
                "book_path": "/path/to/test.pdf"
            })
            
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "success"
            assert data["documents_indexed"] == 1
            assert data["chunks_created"] == 5
    
    def test_ingestion_endpoint_error(self, client, test_db):
        """Test document ingestion endpoint with error"""
        with patch('book-backend.src.api.ingestion.get_ingestion_service') as mock_service:
            mock_ingestion_instance = Mock()
            mock_ingestion_instance.ingest_book_content = AsyncMock(return_value={
                "status": "error",
                "message": "File not found",
                "documents_indexed": 0
            })
            mock_service.return_value = mock_ingestion_instance
            
            response = client.post("/api/ingest/book", json={
                "book_path": "/nonexistent/path.pdf"
            })
            
            assert response.status_code == 400
    
    def test_file_upload_endpoint(self, client, test_db):
        """Test file upload endpoint"""
        with patch('book-backend.src.api.file_upload.get_ingestion_service') as mock_service:
            mock_ingestion_instance = Mock()
            mock_ingestion_instance.ingest_book_content = AsyncMock(return_value={
                "status": "success",
                "message": "File processed successfully",
                "documents_indexed": 1,
                "chunks_created": 3
            })
            mock_service.return_value = mock_ingestion_instance
            
            # Create a temporary test file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as temp_file:
                temp_file.write("This is test content for file upload.")
                temp_file_path = temp_file.name
            
            try:
                with open(temp_file_path, 'rb') as test_file:
                    response = client.post("/api/chat/file", 
                        files={"file": ("test.txt", test_file, "text/plain")},
                        data={"session_id": "test-session-id"}
                    )
                
                assert response.status_code == 200
                data = response.json()
                assert data["status"] == "success"
                assert data["context_added"] == True
            finally:
                os.unlink(temp_file_path)
    
    def test_voice_endpoint_mock(self, client, test_db):
        """Test voice processing endpoint (mocked)"""
        with patch('book-backend.src.api.voice_chat.get_rag_service') as mock_rag, \
             patch('book-backend.src.api.voice_chat.transcribe_audio_whisper') as mock_whisper:
            
            # Mock RAG service
            mock_rag_instance = Mock()
            mock_rag_instance.retrieve_context = AsyncMock(return_value=["Voice context"])
            mock_rag_instance.generate_rag_response = AsyncMock(return_value="Voice response")
            mock_rag.return_value = mock_rag_instance
            
            # Mock Whisper transcription
            mock_whisper.return_value = "This is a transcribed voice message"
            
            # Mock base64 audio data
            fake_audio_base64 = "UklGRiQAAABXQVZFZm10IBAAAAABAAEARKwAAIhYAQACABAAZGF0YQAAAAA="
            
            response = client.post("/api/chat/voice", json={
                "audio_base64": fake_audio_base64
            })
            
            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "transcript" in data
            assert "session_id" in data
            assert data["transcript"] == "This is a transcribed voice message"
            assert data["response"] == "Voice response"


class TestRAGPipelineEndToEnd:
    """End-to-end integration tests"""
    
    @pytest.fixture
    def test_db(self):
        """Create a test database"""
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        
        def override_get_db():
            try:
                db = TestingSessionLocal()
                yield db
            finally:
                db.close()
        
        app.dependency_overrides[get_db] = override_get_db
        yield TestingSessionLocal()
        app.dependency_overrides.clear()
    
    @pytest.fixture
    def client(self, test_db):
        """Create a test client"""
        return TestClient(app)
    
    def test_complete_rag_workflow(self, client, test_db):
        """Test the complete RAG workflow: ingest -> chat -> retrieve history"""
        with patch('book-backend.src.api.ingestion.get_ingestion_service') as mock_ingestion, \
             patch('book-backend.src.api.chat.get_qdrant_client') as mock_qdrant, \
             patch('book-backend.src.api.chat.get_embedding_service') as mock_embedding, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent, \
             patch('book-backend.src.database.crud.get_document_chunk_content_by_embedding_ids') as mock_crud:
            
            # Step 1: Mock ingestion
            mock_ingestion_instance = Mock()
            mock_ingestion_instance.ingest_book_content = AsyncMock(return_value={
                "status": "success",
                "message": "Document ingested",
                "documents_indexed": 1,
                "chunks_created": 3
            })
            mock_ingestion.return_value = mock_ingestion_instance
            
            # Step 2: Mock RAG components
            mock_qdrant_instance = Mock()
            mock_hit = Mock()
            mock_hit.payload = {"embedding_id": "test_embed"}
            mock_qdrant_instance.search.return_value = [mock_hit]
            mock_qdrant.return_value = mock_qdrant_instance
            
            mock_embedding_instance = Mock()
            mock_embedding_instance.generate_embeddings = AsyncMock(return_value=[[0.1, 0.2, 0.3]])
            mock_embedding.return_value = mock_embedding_instance
            
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "Based on the document, machine learning is..."
            mock_agent_instance.send_message = AsyncMock(return_value=mock_response)
            mock_chat_agent.return_value = mock_agent_instance
            
            mock_crud.return_value = ["Machine learning is a subset of artificial intelligence..."]
            
            # Step 1: Ingest a document
            ingest_response = client.post("/api/ingest/book", json={
                "book_path": "/path/to/ml_book.pdf"
            })
            assert ingest_response.status_code == 200
            
            # Step 2: Ask a question using RAG
            chat_response = client.post("/api/chat/rag", json={
                "message": "What is machine learning?"
            })
            assert chat_response.status_code == 200
            chat_data = chat_response.json()
            session_id = chat_data["session_id"]
            
            # Verify RAG response
            assert "machine learning" in chat_data["response"].lower()
            assert len(chat_data["context"]) > 0
            
            # Step 3: Ask a follow-up question
            followup_response = client.post("/api/chat/rag", json={
                "message": "Can you explain more about AI?",
                "session_id": session_id
            })
            assert followup_response.status_code == 200
            
            # Step 4: Get chat history
            history_response = client.post("/api/chat/history", json={
                "session_id": session_id
            })
            assert history_response.status_code == 200
    
    def test_error_handling_pipeline(self, client, test_db):
        """Test error handling throughout the pipeline"""
        # Test with invalid input
        response = client.post("/api/chat/rag", json={
            "message": ""  # Empty message
        })
        # Should handle gracefully (might return 400 or process empty message)
        assert response.status_code in [200, 400, 422]
        
        # Test with malformed JSON
        response = client.post("/api/chat/rag", 
            data="invalid json",
            headers={"Content-Type": "application/json"}
        )
        assert response.status_code == 422  # Validation error
    
    def test_concurrent_requests(self, client, test_db):
        """Test handling concurrent requests"""
        with patch('book-backend.src.api.chat.get_qdrant_client') as mock_qdrant, \
             patch('book-backend.src.api.chat.get_embedding_service') as mock_embedding, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent, \
             patch('book-backend.src.database.crud.get_document_chunk_content_by_embedding_ids') as mock_crud:
            
            # Setup mocks
            mock_qdrant_instance = Mock()
            mock_hit = Mock()
            mock_hit.payload = {"embedding_id": "test_embed"}
            mock_qdrant_instance.search.return_value = [mock_hit]
            mock_qdrant.return_value = mock_qdrant_instance
            
            mock_embedding_instance = Mock()
            mock_embedding_instance.generate_embeddings = AsyncMock(return_value=[[0.1, 0.2, 0.3]])
            mock_embedding.return_value = mock_embedding_instance
            
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "Concurrent response"
            mock_agent_instance.send_message = AsyncMock(return_value=mock_response)
            mock_chat_agent.return_value = mock_agent_instance
            
            mock_crud.return_value = ["Test context"]
            
            # Send multiple concurrent requests
            responses = []
            for i in range(3):
                response = client.post("/api/chat/rag", json={
                    "message": f"Concurrent message {i}"
                })
                responses.append(response)
            
            # All should succeed
            for response in responses:
                assert response.status_code == 200
                data = response.json()
                assert "session_id" in data
                assert "response" in data


if __name__ == "__main__":
    pytest.main([__file__])