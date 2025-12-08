import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
from book-backend.src.services.embedding_service import EmbeddingService


class TestEmbeddingService:
    """Unit tests for EmbeddingService"""
    
    @pytest.fixture
    def embedding_service(self):
        """Create an EmbeddingService instance for testing"""
        with patch('book-backend.src.services.embedding_service.SentenceTransformer') as mock_transformer:
            mock_model = Mock()
            mock_transformer.return_value = mock_model
            service = EmbeddingService()
            service.model = mock_model
            return service
    
    def test_init(self):
        """Test EmbeddingService initialization"""
        with patch('book-backend.src.services.embedding_service.SentenceTransformer') as mock_transformer:
            mock_model = Mock()
            mock_transformer.return_value = mock_model
            
            service = EmbeddingService()
            
            mock_transformer.assert_called_once_with('all-MiniLM-L6-v2')
            assert service.model == mock_model
    
    @pytest.mark.asyncio
    async def test_generate_embeddings_success(self, embedding_service):
        """Test successful embedding generation"""
        # Mock data
        texts = ["Hello world", "This is a test"]
        mock_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        
        # Mock the model encode method
        embedding_service.model.encode.return_value = mock_embeddings
        
        # Call the method
        result = await embedding_service.generate_embeddings(texts)
        
        # Assertions
        embedding_service.model.encode.assert_called_once_with(texts)
        assert result == mock_embeddings
    
    @pytest.mark.asyncio
    async def test_generate_embeddings_empty_input(self, embedding_service):
        """Test embedding generation with empty input"""
        result = await embedding_service.generate_embeddings([])
        assert result == []
        embedding_service.model.encode.assert_not_called()
    
    @pytest.mark.asyncio
    async def test_generate_embeddings_exception(self, embedding_service):
        """Test embedding generation with exception"""
        texts = ["Hello world"]
        embedding_service.model.encode.side_effect = Exception("Model error")
        
        with pytest.raises(Exception, match="Model error"):
            await embedding_service.generate_embeddings(texts)
    
    @pytest.mark.asyncio
    async def test_chunk_document_default_params(self, embedding_service):
        """Test document chunking with default parameters"""
        text = "This is a test document. " * 100  # Create a long text
        
        result = await embedding_service.chunk_document(text)
        
        assert isinstance(result, list)
        assert len(result) > 0
        assert all(isinstance(chunk, str) for chunk in result)
        
        # Check that chunks are not too long (default chunk_size=512)
        for chunk in result:
            assert len(chunk) <= 600  # Allow some buffer for word boundaries
    
    @pytest.mark.asyncio
    async def test_chunk_document_custom_params(self, embedding_service):
        """Test document chunking with custom parameters"""
        text = "This is a test document. " * 50
        chunk_size = 100
        overlap = 20
        
        result = await embedding_service.chunk_document(text, chunk_size=chunk_size, overlap=overlap)
        
        assert isinstance(result, list)
        assert len(result) > 0
        
        # Check that chunks respect the size limit
        for chunk in result:
            assert len(chunk) <= chunk_size + 50  # Allow buffer for word boundaries
    
    @pytest.mark.asyncio
    async def test_chunk_document_short_text(self, embedding_service):
        """Test document chunking with short text"""
        text = "Short text"
        
        result = await embedding_service.chunk_document(text)
        
        assert result == [text]
    
    @pytest.mark.asyncio
    async def test_chunk_document_empty_text(self, embedding_service):
        """Test document chunking with empty text"""
        result = await embedding_service.chunk_document("")
        assert result == []
    
    @pytest.mark.asyncio
    async def test_chunk_document_whitespace_only(self, embedding_service):
        """Test document chunking with whitespace-only text"""
        result = await embedding_service.chunk_document("   \n\t   ")
        assert result == []
    
    def test_chunk_text_by_sentences(self, embedding_service):
        """Test the sentence-based chunking logic"""
        text = "First sentence. Second sentence! Third sentence? Fourth sentence."
        chunk_size = 30
        overlap = 5
        
        chunks = embedding_service._chunk_text_by_sentences(text, chunk_size, overlap)
        
        assert isinstance(chunks, list)
        assert len(chunks) > 0
        assert all(isinstance(chunk, str) for chunk in chunks)
    
    def test_chunk_text_by_words(self, embedding_service):
        """Test the word-based chunking logic"""
        text = "This is a test document with many words that should be chunked properly"
        chunk_size = 20
        overlap = 5
        
        chunks = embedding_service._chunk_text_by_words(text, chunk_size, overlap)
        
        assert isinstance(chunks, list)
        assert len(chunks) > 0
        assert all(isinstance(chunk, str) for chunk in chunks)
        
        # Check overlap functionality
        if len(chunks) > 1:
            # There should be some overlap between consecutive chunks
            first_chunk_words = chunks[0].split()
            second_chunk_words = chunks[1].split()
            
            # Check if there's any word overlap
            overlap_found = any(word in second_chunk_words for word in first_chunk_words[-overlap:])
            assert overlap_found or len(first_chunk_words) < overlap


# Integration-style tests (still unit tests but testing the full flow)
class TestEmbeddingServiceIntegration:
    """Integration-style unit tests for EmbeddingService"""
    
    @pytest.mark.asyncio
    async def test_full_workflow(self):
        """Test the full workflow: chunk document and generate embeddings"""
        with patch('book-backend.src.services.embedding_service.SentenceTransformer') as mock_transformer:
            # Setup mock
            mock_model = Mock()
            mock_model.encode.return_value = [[0.1, 0.2], [0.3, 0.4]]
            mock_transformer.return_value = mock_model
            
            # Create service
            service = EmbeddingService()
            
            # Test data
            text = "This is a test document. It has multiple sentences. Each sentence should be processed correctly."
            
            # Chunk the document
            chunks = await service.chunk_document(text, chunk_size=50)
            
            # Generate embeddings
            embeddings = await service.generate_embeddings(chunks)
            
            # Assertions
            assert len(chunks) > 0
            assert len(embeddings) == len(chunks)
            assert all(isinstance(embedding, list) for embedding in embeddings)
            mock_model.encode.assert_called_once_with(chunks)


if __name__ == "__main__":
    pytest.main([__file__])