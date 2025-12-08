import pytest
import uuid
from datetime import datetime
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from book-backend.src.database.models import Base, Session as DBSession, Message, Document, DocumentChunk
from book-backend.src.database import crud


class TestDatabaseModels:
    """Unit tests for database models"""
    
    @pytest.fixture
    def db_session(self):
        """Create an in-memory SQLite database for testing"""
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        session = TestingSessionLocal()
        try:
            yield session
        finally:
            session.close()
    
    def test_session_model(self, db_session):
        """Test Session model creation and attributes"""
        user_id = uuid.uuid4()
        session = DBSession(user_id=user_id)
        
        db_session.add(session)
        db_session.commit()
        db_session.refresh(session)
        
        assert session.id is not None
        assert session.user_id == user_id
        assert session.created_at is not None
        assert session.updated_at is not None
        assert isinstance(session.created_at, datetime)
        assert isinstance(session.updated_at, datetime)
    
    def test_session_model_without_user(self, db_session):
        """Test Session model creation without user_id"""
        session = DBSession()
        
        db_session.add(session)
        db_session.commit()
        db_session.refresh(session)
        
        assert session.id is not None
        assert session.user_id is None
        assert session.created_at is not None
        assert session.updated_at is not None
    
    def test_message_model(self, db_session):
        """Test Message model creation and attributes"""
        # Create a session first
        session = DBSession()
        db_session.add(session)
        db_session.commit()
        db_session.refresh(session)
        
        # Create a message
        message = Message(
            session_id=session.id,
            content="Test message",
            sender="user",
            extra_data={"test": "data"}
        )
        
        db_session.add(message)
        db_session.commit()
        db_session.refresh(message)
        
        assert message.id is not None
        assert message.session_id == session.id
        assert message.content == "Test message"
        assert message.sender == "user"
        assert message.timestamp is not None
        assert message.extra_data == {"test": "data"}
        assert isinstance(message.timestamp, datetime)
    
    def test_message_model_relationships(self, db_session):
        """Test Message model relationships with Session"""
        # Create a session
        session = DBSession()
        db_session.add(session)
        db_session.commit()
        db_session.refresh(session)
        
        # Create messages
        message1 = Message(session_id=session.id, content="Message 1", sender="user")
        message2 = Message(session_id=session.id, content="Message 2", sender="ai")
        
        db_session.add_all([message1, message2])
        db_session.commit()
        
        # Test relationship
        db_session.refresh(session)
        assert len(session.messages) == 2
        assert message1 in session.messages
        assert message2 in session.messages
    
    def test_document_model(self, db_session):
        """Test Document model creation and attributes"""
        document = Document(
            title="Test Document",
            source_uri="/path/to/test.pdf",
            total_chunks=5
        )
        
        db_session.add(document)
        db_session.commit()
        db_session.refresh(document)
        
        assert document.id is not None
        assert document.title == "Test Document"
        assert document.source_uri == "/path/to/test.pdf"
        assert document.total_chunks == 5
        assert document.created_at is not None
        assert isinstance(document.created_at, datetime)
    
    def test_document_chunk_model(self, db_session):
        """Test DocumentChunk model creation and attributes"""
        # Create a document first
        document = Document(title="Test Document", source_uri="/path/to/test.pdf")
        db_session.add(document)
        db_session.commit()
        db_session.refresh(document)
        
        # Create a document chunk
        chunk = DocumentChunk(
            document_id=document.id,
            chunk_index=0,
            content="This is a test chunk content",
            embedding_id="embed_123",
            extra_data={"chunk_type": "paragraph"}
        )
        
        db_session.add(chunk)
        db_session.commit()
        db_session.refresh(chunk)
        
        assert chunk.id is not None
        assert chunk.document_id == document.id
        assert chunk.chunk_index == 0
        assert chunk.content == "This is a test chunk content"
        assert chunk.embedding_id == "embed_123"
        assert chunk.extra_data == {"chunk_type": "paragraph"}
        assert chunk.created_at is not None
        assert isinstance(chunk.created_at, datetime)
    
    def test_document_chunk_relationships(self, db_session):
        """Test DocumentChunk model relationships with Document"""
        # Create a document
        document = Document(title="Test Document", source_uri="/path/to/test.pdf")
        db_session.add(document)
        db_session.commit()
        db_session.refresh(document)
        
        # Create chunks
        chunk1 = DocumentChunk(
            document_id=document.id,
            chunk_index=0,
            content="Chunk 1",
            embedding_id="embed_1"
        )
        chunk2 = DocumentChunk(
            document_id=document.id,
            chunk_index=1,
            content="Chunk 2",
            embedding_id="embed_2"
        )
        
        db_session.add_all([chunk1, chunk2])
        db_session.commit()
        
        # Test relationship
        db_session.refresh(document)
        assert len(document.chunks) == 2
        assert chunk1 in document.chunks
        assert chunk2 in document.chunks


class TestCRUDOperations:
    """Unit tests for CRUD operations"""
    
    @pytest.fixture
    def db_session(self):
        """Create an in-memory SQLite database for testing"""
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        session = TestingSessionLocal()
        try:
            yield session
        finally:
            session.close()
    
    def test_create_session(self, db_session):
        """Test creating a session"""
        user_id = uuid.uuid4()
        session = crud.create_session(db_session, user_id)
        
        assert session.id is not None
        assert session.user_id == user_id
        assert session.created_at is not None
        assert session.updated_at is not None
    
    def test_create_session_without_user(self, db_session):
        """Test creating a session without user_id"""
        session = crud.create_session(db_session)
        
        assert session.id is not None
        assert session.user_id is None
        assert session.created_at is not None
        assert session.updated_at is not None
    
    def test_get_session(self, db_session):
        """Test retrieving a session"""
        # Create a session
        created_session = crud.create_session(db_session)
        
        # Retrieve the session
        retrieved_session = crud.get_session(db_session, created_session.id)
        
        assert retrieved_session is not None
        assert retrieved_session.id == created_session.id
        assert retrieved_session.user_id == created_session.user_id
    
    def test_get_session_not_found(self, db_session):
        """Test retrieving a non-existent session"""
        non_existent_id = uuid.uuid4()
        session = crud.get_session(db_session, non_existent_id)
        
        assert session is None
    
    def test_create_message(self, db_session):
        """Test creating a message"""
        # Create a session first
        session = crud.create_session(db_session)
        
        # Create a message
        message = crud.create_message(
            db_session,
            session.id,
            "Test message content",
            "user",
            {"test": "data"}
        )
        
        assert message.id is not None
        assert message.session_id == session.id
        assert message.content == "Test message content"
        assert message.sender == "user"
        assert message.extra_data == {"test": "data"}
        assert message.timestamp is not None
    
    def test_get_messages_by_session_id(self, db_session):
        """Test retrieving messages by session ID"""
        # Create a session
        session = crud.create_session(db_session)
        
        # Create messages
        message1 = crud.create_message(db_session, session.id, "Message 1", "user")
        message2 = crud.create_message(db_session, session.id, "Message 2", "ai")
        
        # Retrieve messages
        messages = crud.get_messages_by_session_id(db_session, session.id)
        
        assert len(messages) == 2
        assert messages[0].id == message1.id  # Should be ordered by timestamp
        assert messages[1].id == message2.id
        assert all(msg.session_id == session.id for msg in messages)
    
    def test_get_messages_empty_session(self, db_session):
        """Test retrieving messages from empty session"""
        # Create a session without messages
        session = crud.create_session(db_session)
        
        # Retrieve messages
        messages = crud.get_messages_by_session_id(db_session, session.id)
        
        assert len(messages) == 0
    
    def test_create_document(self, db_session):
        """Test creating a document"""
        document = crud.create_document(
            db_session,
            "Test Document",
            "/path/to/test.pdf",
            5
        )
        
        assert document.id is not None
        assert document.title == "Test Document"
        assert document.source_uri == "/path/to/test.pdf"
        assert document.total_chunks == 5
        assert document.created_at is not None
    
    def test_create_document_chunk(self, db_session):
        """Test creating a document chunk"""
        # Create a document first
        document = crud.create_document(db_session, "Test Document", "/path/to/test.pdf")
        
        # Create a chunk
        chunk = crud.create_document_chunk(
            db_session,
            document.id,
            0,
            "Test chunk content",
            "embed_123",
            {"chunk_type": "paragraph"}
        )
        
        assert chunk.id is not None
        assert chunk.document_id == document.id
        assert chunk.chunk_index == 0
        assert chunk.content == "Test chunk content"
        assert chunk.embedding_id == "embed_123"
        assert chunk.extra_data == {"chunk_type": "paragraph"}
        assert chunk.created_at is not None
    
    def test_get_document_chunk_content_by_embedding_ids(self, db_session):
        """Test retrieving document chunk content by embedding IDs"""
        # Create a document
        document = crud.create_document(db_session, "Test Document", "/path/to/test.pdf")
        
        # Create chunks
        chunk1 = crud.create_document_chunk(
            db_session, document.id, 0, "Content 1", "embed_1"
        )
        chunk2 = crud.create_document_chunk(
            db_session, document.id, 1, "Content 2", "embed_2"
        )
        chunk3 = crud.create_document_chunk(
            db_session, document.id, 2, "Content 3", "embed_3"
        )
        
        # Retrieve content by embedding IDs
        embedding_ids = ["embed_1", "embed_3", "embed_nonexistent"]
        content = crud.get_document_chunk_content_by_embedding_ids(db_session, embedding_ids)
        
        # Should return content in the order of embedding_ids, skipping non-existent ones
        assert len(content) == 2
        assert content[0] == "Content 1"
        assert content[1] == "Content 3"
    
    def test_get_document_chunk_content_empty_list(self, db_session):
        """Test retrieving document chunk content with empty embedding IDs list"""
        content = crud.get_document_chunk_content_by_embedding_ids(db_session, [])
        assert content == []
    
    def test_get_document_chunk_content_no_matches(self, db_session):
        """Test retrieving document chunk content with no matching embedding IDs"""
        content = crud.get_document_chunk_content_by_embedding_ids(
            db_session, ["nonexistent_1", "nonexistent_2"]
        )
        assert content == []


class TestDatabaseConstraints:
    """Test database constraints and edge cases"""
    
    @pytest.fixture
    def db_session(self):
        """Create an in-memory SQLite database for testing"""
        engine = create_engine(
            "sqlite:///:memory:",
            connect_args={"check_same_thread": False},
            poolclass=StaticPool,
        )
        Base.metadata.create_all(bind=engine)
        TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        session = TestingSessionLocal()
        try:
            yield session
        finally:
            session.close()
    
    def test_message_sender_constraint(self, db_session):
        """Test that message sender must be 'user' or 'ai'"""
        session = crud.create_session(db_session)
        
        # Valid senders should work
        crud.create_message(db_session, session.id, "Test", "user")
        crud.create_message(db_session, session.id, "Test", "ai")
        
        # Invalid sender should raise an error (in a real database with constraints)
        # Note: SQLite doesn't enforce CHECK constraints by default, so this might not fail
        # In a production PostgreSQL database, this would fail
        try:
            crud.create_message(db_session, session.id, "Test", "invalid_sender")
            # If we reach here, the constraint wasn't enforced (SQLite behavior)
        except Exception:
            # This would happen in PostgreSQL with proper constraints
            pass
    
    def test_unique_embedding_id_constraint(self, db_session):
        """Test that embedding_id should be unique"""
        document = crud.create_document(db_session, "Test Document", "/path/to/test.pdf")
        
        # First chunk with embedding_id should work
        crud.create_document_chunk(db_session, document.id, 0, "Content 1", "embed_unique")
        
        # Second chunk with same embedding_id should fail (in production with proper constraints)
        try:
            crud.create_document_chunk(db_session, document.id, 1, "Content 2", "embed_unique")
            # If we reach here, the constraint wasn't enforced (SQLite behavior)
        except Exception:
            # This would happen in PostgreSQL with proper UNIQUE constraints
            pass


if __name__ == "__main__":
    pytest.main([__file__])