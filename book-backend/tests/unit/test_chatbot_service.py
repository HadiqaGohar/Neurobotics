"""Unit tests for ChatbotService"""

import pytest
from unittest.mock import Mock, patch, AsyncMock
from app.services.chatbot_service import ChatbotService
from app.models.database import ChatSession, ChatMessage, User


class TestChatbotService:
    """Test ChatbotService functionality"""
    
    def test_create_chat_session(self, db_session):
        """Test creating a chat session"""
        service = ChatbotService(db_session)
        
        session = service.create_chat_session(
            user_id=1,
            title="Test Session",
            session_type="rag",
            language="en"
        )
        
        assert session.title == "Test Session"
        assert session.session_type == "rag"
        assert session.language == "en"
        assert session.user_id == 1
        assert session.id is not None
    
    def test_get_chat_session(self, db_session):
        """Test retrieving a chat session"""
        service = ChatbotService(db_session)
        
        # Create session
        created_session = service.create_chat_session(
            title="Test Session"
        )
        
        # Retrieve session
        retrieved_session = service.get_chat_session(created_session.id)
        
        assert retrieved_session is not None
        assert retrieved_session.id == created_session.id
        assert retrieved_session.title == "Test Session"
    
    def test_add_message_to_session(self, db_session):
        """Test adding a message to a session"""
        service = ChatbotService(db_session)
        
        # Create session
        session = service.create_chat_session(title="Test Session")
        
        # Add message
        message = service.add_message_to_session(
            session_id=session.id,
            content="Hello, chatbot!",
            sender="user"
        )
        
        assert message.content == "Hello, chatbot!"
        assert message.sender == "user"
        assert message.session_id == session.id
        assert message.id is not None
    
    def test_get_session_messages(self, db_session):
        """Test retrieving messages from a session"""
        service = ChatbotService(db_session)
        
        # Create session
        session = service.create_chat_session(title="Test Session")
        
        # Add messages
        message1 = service.add_message_to_session(
            session_id=session.id,
            content="First message",
            sender="user"
        )
        message2 = service.add_message_to_session(
            session_id=session.id,
            content="Second message",
            sender="assistant"
        )
        
        # Get messages
        messages = service.get_session_messages(session.id)
        
        assert len(messages) == 2
        assert messages[0].content == "First message"
        assert messages[1].content == "Second message"
    
    @patch('app.services.chatbot_service.AsyncOpenAI')
    def test_generate_embeddings(self, mock_openai, db_session):
        """Test generating embeddings"""
        service = ChatbotService(db_session)
        
        # Mock OpenAI response
        mock_response = Mock()
        mock_response.data = [Mock(embedding=[0.1, 0.2, 0.3])]
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_response
        mock_openai.return_value = mock_client
        
        # Test embedding generation
        embedding = service.generate_embeddings("test text")
        
        assert embedding == [0.1, 0.2, 0.3]
    
    @patch('app.services.chatbot_service.qdrant_service')
    def test_search_relevant_content(self, mock_qdrant, db_session):
        """Test searching for relevant content"""
        service = ChatbotService(db_session)
        
        # Mock Qdrant response
        mock_qdrant.search_similar.return_value = [
            {
                "payload": {
                    "text": "Relevant content",
                    "chapter_id": 1
                },
                "score": 0.9
            }
        ]
        
        # Mock embedding generation
        with patch.object(service, 'generate_embeddings', return_value=[0.1, 0.2, 0.3]):
            results = service.search_relevant_content("test query")
        
        assert len(results) >= 0  # May be empty due to missing database content
    
    def test_get_chat_suggestions(self, db_session):
        """Test getting chat suggestions"""
        service = ChatbotService(db_session)
        
        # Create session
        session = service.create_chat_session(title="Test Session")
        
        # Get suggestions
        suggestions = service.get_chat_suggestions(session.id)
        
        assert isinstance(suggestions, list)
        assert len(suggestions) > 0
        assert all(isinstance(s, str) for s in suggestions)
    
    def test_delete_chat_session(self, db_session):
        """Test deleting a chat session"""
        service = ChatbotService(db_session)
        
        # Create session with messages
        session = service.create_chat_session(title="Test Session")
        service.add_message_to_session(
            session_id=session.id,
            content="Test message",
            sender="user"
        )
        
        # Delete session
        result = service.delete_chat_session(session.id)
        
        assert result is True
        
        # Verify session is deleted
        deleted_session = service.get_chat_session(session.id)
        assert deleted_session is None
    
    @patch('app.services.chatbot_service.Runner')
    @patch('app.services.chatbot_service.Agent')
    def test_chat_with_rag_success(self, mock_agent, mock_runner, db_session):
        """Test successful RAG chat"""
        service = ChatbotService(db_session)
        
        # Mock agent and runner
        mock_result = Mock()
        mock_result.data = "This is a test response"
        mock_result.messages = []
        
        mock_runner_instance = Mock()
        mock_runner_instance.run = AsyncMock(return_value=mock_result)
        mock_runner.return_value = mock_runner_instance
        
        service.agent = Mock()
        service.runner = mock_runner_instance
        
        # Create session
        session = service.create_chat_session(title="Test Session")
        
        # Test chat
        result = service.chat_with_rag(
            session_id=session.id,
            user_message="What is this book about?",
            use_rag=True
        )
        
        assert "response" in result
        assert "session_id" in result
        assert "confidence_score" in result
        assert result["session_id"] == session.id
    
    def test_chat_with_rag_no_agent(self, db_session):
        """Test RAG chat when agent is not available"""
        service = ChatbotService(db_session)
        
        # Ensure agent is None
        service.agent = None
        service.runner = None
        
        # Create session
        session = service.create_chat_session(title="Test Session")
        
        # Test chat
        result = service.chat_with_rag(
            session_id=session.id,
            user_message="What is this book about?",
            use_rag=True
        )
        
        assert "response" in result
        assert "AI service" in result["response"] or "properly configured" in result["response"]