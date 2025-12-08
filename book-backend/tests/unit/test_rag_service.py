import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock, MagicMock
from book-backend.src.services.rag_service import RAGService


class TestRAGService:
    """Unit tests for RAGService"""
    
    @pytest.fixture
    def mock_qdrant_client(self):
        """Create a mock Qdrant client"""
        mock_client = Mock()
        return mock_client
    
    @pytest.fixture
    def mock_embedding_service(self):
        """Create a mock EmbeddingService"""
        mock_service = Mock()
        return mock_service
    
    @pytest.fixture
    def rag_service(self, mock_qdrant_client, mock_embedding_service):
        """Create a RAGService instance for testing"""
        with patch('book-backend.src.services.rag_service.get_db') as mock_get_db, \
             patch('book-backend.src.services.rag_service.crud') as mock_crud, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent:
            
            # Setup database mock
            mock_db = Mock()
            mock_get_db.return_value = iter([mock_db])
            
            # Setup CRUD mock
            mock_crud.get_document_chunk_content_by_embedding_ids.return_value = [
                "Test content 1", "Test content 2"
            ]
            
            # Setup ChatAgent mock
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "Test AI response"
            mock_agent_instance.send_message = AsyncMock(return_value=mock_response)
            mock_chat_agent.return_value = mock_agent_instance
            
            service = RAGService(mock_qdrant_client, mock_embedding_service)
            service.chat_agent = mock_agent_instance
            
            return service
    
    @pytest.mark.asyncio
    async def test_retrieve_context_success(self, rag_service, mock_embedding_service, mock_qdrant_client):
        """Test successful context retrieval"""
        # Setup mocks
        query = "test query"
        mock_embedding = [0.1, 0.2, 0.3]
        mock_embedding_service.generate_embeddings.return_value = [mock_embedding]
        
        # Mock Qdrant search results
        mock_hit1 = Mock()
        mock_hit1.payload = {"embedding_id": "embed1"}
        mock_hit2 = Mock()
        mock_hit2.payload = {"embedding_id": "embed2"}
        mock_qdrant_client.search.return_value = [mock_hit1, mock_hit2]
        
        # Call the method
        result = await rag_service.retrieve_context(query, top_k=2)
        
        # Assertions
        mock_embedding_service.generate_embeddings.assert_called_once_with([query])
        mock_qdrant_client.search.assert_called_once_with(
            collection_name="book_content",
            query_vector=mock_embedding,
            limit=2
        )
        assert result == ["Test content 1", "Test content 2"]
    
    @pytest.mark.asyncio
    async def test_retrieve_context_no_embeddings(self, rag_service, mock_embedding_service):
        """Test context retrieval when no embeddings are generated"""
        query = "test query"
        mock_embedding_service.generate_embeddings.return_value = []
        
        result = await rag_service.retrieve_context(query)
        
        assert result == []
        mock_embedding_service.generate_embeddings.assert_called_once_with([query])
    
    @pytest.mark.asyncio
    async def test_retrieve_context_no_search_results(self, rag_service, mock_embedding_service, mock_qdrant_client):
        """Test context retrieval when Qdrant returns no results"""
        query = "test query"
        mock_embedding = [0.1, 0.2, 0.3]
        mock_embedding_service.generate_embeddings.return_value = [mock_embedding]
        mock_qdrant_client.search.return_value = []
        
        result = await rag_service.retrieve_context(query)
        
        assert result == []
    
    @pytest.mark.asyncio
    async def test_retrieve_context_invalid_payload(self, rag_service, mock_embedding_service, mock_qdrant_client):
        """Test context retrieval when search results have invalid payload"""
        query = "test query"
        mock_embedding = [0.1, 0.2, 0.3]
        mock_embedding_service.generate_embeddings.return_value = [mock_embedding]
        
        # Mock search results with invalid payloads
        mock_hit1 = Mock()
        mock_hit1.payload = None
        mock_hit2 = Mock()
        mock_hit2.payload = {"invalid_key": "value"}
        mock_qdrant_client.search.return_value = [mock_hit1, mock_hit2]
        
        result = await rag_service.retrieve_context(query)
        
        assert result == []
    
    @pytest.mark.asyncio
    async def test_generate_rag_response_success(self, rag_service):
        """Test successful RAG response generation"""
        query = "What is machine learning?"
        context = ["Machine learning is a subset of AI", "It involves training algorithms"]
        
        result = await rag_service.generate_rag_response(query, context)
        
        assert result == "Test AI response"
        
        # Verify the prompt was constructed correctly
        expected_prompt = """You are a helpful AI assistant. Use the following pieces of context to answer the question at the end.
        If you don't know the answer, just say that you don't know, don't try to make up an answer.

        Context:
        Machine learning is a subset of AI
It involves training algorithms

        Question: What is machine learning?

        Helpful Answer:"""
        
        rag_service.chat_agent.send_message.assert_called_once_with(expected_prompt)
    
    @pytest.mark.asyncio
    async def test_generate_rag_response_empty_context(self, rag_service):
        """Test RAG response generation with empty context"""
        query = "What is machine learning?"
        context = []
        
        result = await rag_service.generate_rag_response(query, context)
        
        assert result == "Test AI response"
        
        # Verify the prompt was constructed with empty context
        expected_prompt = """You are a helpful AI assistant. Use the following pieces of context to answer the question at the end.
        If you don't know the answer, just say that you don't know, don't try to make up an answer.

        Context:
        

        Question: What is machine learning?

        Helpful Answer:"""
        
        rag_service.chat_agent.send_message.assert_called_once_with(expected_prompt)
    
    @pytest.mark.asyncio
    async def test_generate_rag_response_chat_agent_error(self, rag_service):
        """Test RAG response generation when ChatAgent fails"""
        query = "test query"
        context = ["test context"]
        
        # Make ChatAgent raise an exception
        rag_service.chat_agent.send_message.side_effect = Exception("ChatAgent error")
        
        with pytest.raises(Exception, match="ChatAgent error"):
            await rag_service.generate_rag_response(query, context)
    
    def test_init(self, mock_qdrant_client, mock_embedding_service):
        """Test RAGService initialization"""
        with patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent, \
             patch.dict('os.environ', {'GEMINI_API_KEY': 'test_key'}):
            
            mock_agent_instance = Mock()
            mock_chat_agent.return_value = mock_agent_instance
            
            service = RAGService(mock_qdrant_client, mock_embedding_service)
            
            assert service.qdrant_client == mock_qdrant_client
            assert service.embedding_service == mock_embedding_service
            assert service.collection_name == "book_content"
            assert service.chat_agent == mock_agent_instance
            
            mock_chat_agent.assert_called_once_with(
                model_name="gemini-pro", 
                api_key="test_key"
            )


# Integration-style tests
class TestRAGServiceIntegration:
    """Integration-style unit tests for RAGService"""
    
    @pytest.mark.asyncio
    async def test_full_rag_workflow(self):
        """Test the complete RAG workflow"""
        with patch('book-backend.src.services.rag_service.get_db') as mock_get_db, \
             patch('book-backend.src.services.rag_service.crud') as mock_crud, \
             patch('book-backend.src.services.rag_service.ChatAgent') as mock_chat_agent:
            
            # Setup mocks
            mock_db = Mock()
            mock_get_db.return_value = iter([mock_db])
            
            mock_crud.get_document_chunk_content_by_embedding_ids.return_value = [
                "Context about machine learning", "More ML context"
            ]
            
            mock_agent_instance = Mock()
            mock_response = Mock()
            mock_response.content = "Machine learning is a powerful technology..."
            mock_agent_instance.send_message = AsyncMock(return_value=mock_response)
            mock_chat_agent.return_value = mock_agent_instance
            
            # Create mocks for dependencies
            mock_qdrant_client = Mock()
            mock_embedding_service = Mock()
            
            # Setup embedding service mock
            mock_embedding_service.generate_embeddings.return_value = [[0.1, 0.2, 0.3]]
            
            # Setup Qdrant search mock
            mock_hit = Mock()
            mock_hit.payload = {"embedding_id": "test_embed_1"}
            mock_qdrant_client.search.return_value = [mock_hit]
            
            # Create service
            service = RAGService(mock_qdrant_client, mock_embedding_service)
            
            # Test the full workflow
            query = "What is machine learning?"
            
            # Step 1: Retrieve context
            context = await service.retrieve_context(query)
            
            # Step 2: Generate response
            response = await service.generate_rag_response(query, context)
            
            # Assertions
            assert context == ["Context about machine learning", "More ML context"]
            assert response == "Machine learning is a powerful technology..."
            
            # Verify all components were called correctly
            mock_embedding_service.generate_embeddings.assert_called_with([query])
            mock_qdrant_client.search.assert_called_once()
            mock_crud.get_document_chunk_content_by_embedding_ids.assert_called_once()
            mock_agent_instance.send_message.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__])