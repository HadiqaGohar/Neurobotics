from sqlalchemy import Column, String, Text, DateTime, Integer, ForeignKey, Boolean
from sqlalchemy.dialects.postgresql import UUID, JSONB, ARRAY
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

from src.database.models import Base # Import Base from existing models


class Book(Base):
    __tablename__ = "books"

    id = Column(Integer, primary_key=True, index=True) # Assuming integer primary key for books as implied by service
    title = Column(String(255), nullable=False, index=True)
    description = Column(Text, nullable=True)
    author_id = Column(Integer, nullable=False) # Assuming author_id is integer, links to User later
    isbn = Column(String(17), unique=True, nullable=True)
    language = Column(String(10), default="en")
    category = Column(String(100), nullable=True)
    tags = Column(ARRAY(String), nullable=True)
    is_published = Column(Boolean, default=False)
    is_featured = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    chapters = relationship("Chapter", back_populates="book", cascade="all, delete-orphan")


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True, index=True) # Assuming integer primary key for chapters
    book_id = Column(Integer, ForeignKey("books.id", ondelete="CASCADE"), nullable=False)
    title = Column(String(255), nullable=False, index=True)
    content = Column(Text, nullable=False)
    chapter_number = Column(Integer, nullable=False)
    word_count = Column(Integer, default=0)
    reading_time = Column(Integer, default=0) # in minutes
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    translated_content = Column(JSONB, nullable=True) # Store translations as JSONB

    book = relationship("Book", back_populates="chapters")
