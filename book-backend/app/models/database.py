"""Database models for the application"""

from typing import Optional, List, Dict, Any
from datetime import datetime
from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, JSON, Float, ForeignKey
from sqlalchemy.orm import relationship
from app.core.database import Base


class User(Base):
    """User database model"""
    __tablename__ = "users"
    
    id = Column(Integer, primary_key=True, index=True)
    email = Column(String(100), unique=True, index=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)
    full_name = Column(String(100), nullable=True)
    persona = Column(String(50), default="general")
    experience_level = Column(String(50), default="intermediate")
    preferred_language = Column(String(10), ForeignKey("languages.code"), default="en")
    secondary_language = Column(String(10), ForeignKey("languages.code"), nullable=True)
    language_preferences = Column(JSON, nullable=True)
    preferences = Column(JSON, default=dict)
    is_active = Column(Boolean, default=True)
    last_login = Column(DateTime, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    chat_sessions = relationship("ChatSession", back_populates="user")


class Book(Base):
    """Book database model"""
    __tablename__ = "books"
    
    id = Column(Integer, primary_key=True, index=True)
    title = Column(String(200), nullable=False, index=True)
    description = Column(Text)
    author = Column(String(100))
    isbn = Column(String(20), unique=True)
    is_published = Column(Boolean, default=False)
    processing_status = Column(String(50), default="pending")
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    chapters = relationship("Chapter", back_populates="book", cascade="all, delete-orphan")
    embeddings = relationship("BookEmbedding", back_populates="book", cascade="all, delete-orphan")


class Chapter(Base):
    """Chapter database model"""
    __tablename__ = "chapters"
    
    id = Column(Integer, primary_key=True, index=True)
    book_id = Column(Integer, ForeignKey("books.id"), nullable=False, index=True)
    title = Column(String(200), nullable=False)
    chapter_number = Column(Integer, nullable=False)
    content = Column(Text, nullable=False)
    translated_content = Column(JSON, default=dict)  # Store translations
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    book = relationship("Book", back_populates="chapters")
    embeddings = relationship("ChapterEmbedding", back_populates="chapter", cascade="all, delete-orphan")


class ChatSession(Base):
    """Chat session database model"""
    __tablename__ = "chat_sessions"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True, index=True)
    title = Column(String(200), nullable=False)
    session_type = Column(String(50), default="general", nullable=False)
    language = Column(String(10), default="en", nullable=False)
    context_book_id = Column(Integer, ForeignKey("books.id"), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    user = relationship("User", back_populates="chat_sessions")
    messages = relationship("ChatMessage", back_populates="session", cascade="all, delete-orphan")
    context_book = relationship("Book")


class ChatMessage(Base):
    """Chat message database model"""
    __tablename__ = "chat_messages"
    
    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("chat_sessions.id"), nullable=False, index=True)
    content = Column(Text, nullable=False)
    sender = Column(String(20), nullable=False)  # 'user' or 'assistant'
    message_type = Column(String(20), default="text", nullable=False)
    language = Column(String(10), default="en", nullable=False)
    context_used = Column(JSON, nullable=True)  # List of sources used
    confidence_score = Column(Float, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    
    # Relationships
    session = relationship("ChatSession", back_populates="messages")


class BookEmbedding(Base):
    """Book embedding database model for vector storage tracking"""
    __tablename__ = "book_embeddings"
    
    id = Column(Integer, primary_key=True, index=True)
    book_id = Column(Integer, ForeignKey("books.id"), nullable=False, index=True)
    text_chunk = Column(Text, nullable=False)
    chunk_index = Column(Integer, nullable=False)
    embedding_vector = Column(JSON, nullable=False)  # Store as JSON array
    qdrant_id = Column(String(100), unique=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    book = relationship("Book", back_populates="embeddings")


class ChapterEmbedding(Base):
    """Chapter embedding database model for vector storage tracking"""
    __tablename__ = "chapter_embeddings"
    
    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False, index=True)
    text_chunk = Column(Text, nullable=False)
    chunk_index = Column(Integer, nullable=False)
    embedding_vector = Column(JSON, nullable=False)  # Store as JSON array
    qdrant_id = Column(String(100), unique=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    chapter = relationship("Chapter", back_populates="embeddings")


# Personalization Models
class UserPreferences(Base):
    """User preferences for content personalization"""
    __tablename__ = "user_preferences"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, unique=True, index=True)
    software_background = Column(JSON, nullable=False, default=dict)
    hardware_background = Column(JSON, nullable=False, default=dict)
    content_complexity = Column(String(50), nullable=False, default="moderate")
    explanation_depth = Column(String(50), nullable=False, default="standard")
    example_style = Column(String(50), nullable=False, default="practical")
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    
    # Relationships
    user = relationship("User", backref="preferences")
    content_variants = relationship("ContentVariant", back_populates="user_preferences")


class ContentVariant(Base):
    """Content variants for different personalization levels"""
    __tablename__ = "content_variants"
    
    id = Column(Integer, primary_key=True, index=True)
    content_id = Column(String(255), unique=True, index=True, nullable=False)
    chapter_id = Column(String(255), index=True, nullable=False)
    section_id = Column(String(255), index=True, nullable=True)
    variant_type = Column(String(50), nullable=False)
    target_audience = Column(JSON, nullable=False, default=dict)
    content = Column(JSON, nullable=False, default=dict)
    content_metadata = Column(JSON, nullable=False, default=dict)
    is_ai_generated = Column(Boolean, nullable=False, default=False)
    quality_score = Column(Integer, nullable=True)
    usage_count = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    
    # Foreign key to user preferences (optional, for user-specific variants)
    user_preferences_id = Column(Integer, ForeignKey("user_preferences.id"), nullable=True)
    user_preferences = relationship("UserPreferences", back_populates="content_variants")


class PersonalizationLog(Base):
    """Log of personalization requests for analytics"""
    __tablename__ = "personalization_logs"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String(255), index=True, nullable=False)
    section_id = Column(String(255), index=True, nullable=True)
    requested_complexity = Column(String(50), nullable=True)
    applied_personalization = Column(JSON, nullable=False, default=dict)
    cache_hit = Column(Boolean, nullable=False, default=False)
    generation_time_ms = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    
    # Relationships
    user = relationship("User")