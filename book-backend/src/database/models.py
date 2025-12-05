from sqlalchemy import create_engine, Column, String, Text, DateTime, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from datetime import datetime
import uuid

Base = declarative_base()

class Session(Base):
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), nullable=True) # Will be linked to 'users' table later
    created_at = Column(DateTime, default=datetime.now)
    updated_at = Column(DateTime, default=datetime.now, onupdate=datetime.now)

    messages = relationship("Message", back_populates="session", cascade="all, delete-orphan")

class Message(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("sessions.id", ondelete='CASCADE'))
    content = Column(Text, nullable=False)
    sender = Column(String(10), nullable=False) # 'user' or 'ai'
    timestamp = Column(DateTime, default=datetime.now)
    extra_data = Column(JSONB) # Store additional context, e.g., retrieved document IDs

    session = relationship("Session", back_populates="messages")

class Document(Base):
    __tablename__ = "documents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String(255), nullable=False)
    source_uri = Column(Text, nullable=False) # e.g., URL, file path, book chapter
    total_chunks = Column(Integer)
    created_at = Column(DateTime, default=datetime.now)

    chunks = relationship("DocumentChunk", back_populates="document", cascade="all, delete-orphan")

class DocumentChunk(Base):
    __tablename__ = "document_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id", ondelete='CASCADE'))
    chunk_index = Column(Integer, nullable=False)
    content = Column(Text, nullable=False) # Storing original chunk text here for retrieval/reconstruction
    embedding_id = Column(Text, unique=True) # ID used in Qdrant
    extra_data = Column(JSONB) # Any additional chunk-specific info
    created_at = Column(DateTime, default=datetime.now)

    document = relationship("Document", back_populates="chunks")
