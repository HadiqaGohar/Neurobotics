"""Unit tests for ContentService"""

import pytest
from unittest.mock import Mock, patch
from app.services.content_service import ContentService
from app.models.database import Book, Chapter


class TestContentService:
    """Test ContentService functionality"""
    
    def test_create_book(self, db_session, sample_book_data):
        """Test creating a book"""
        service = ContentService(db_session)
        
        book = service.create_book(**sample_book_data)
        
        assert book.title == sample_book_data["title"]
        assert book.author == sample_book_data["author"]
        assert book.isbn == sample_book_data["isbn"]
        assert book.id is not None
    
    def test_get_book(self, db_session, sample_book_data):
        """Test retrieving a book"""
        service = ContentService(db_session)
        
        # Create book
        created_book = service.create_book(**sample_book_data)
        
        # Retrieve book
        retrieved_book = service.get_book(created_book.id)
        
        assert retrieved_book is not None
        assert retrieved_book.id == created_book.id
        assert retrieved_book.title == sample_book_data["title"]
    
    def test_get_nonexistent_book(self, db_session):
        """Test retrieving a non-existent book"""
        service = ContentService(db_session)
        
        book = service.get_book(999)
        
        assert book is None
    
    def test_create_chapter(self, db_session, sample_book_data, sample_chapter_data):
        """Test creating a chapter"""
        service = ContentService(db_session)
        
        # Create book first
        book = service.create_book(**sample_book_data)
        
        # Create chapter
        chapter = service.create_chapter(
            book_id=book.id,
            **sample_chapter_data
        )
        
        assert chapter.title == sample_chapter_data["title"]
        assert chapter.chapter_number == sample_chapter_data["chapter_number"]
        assert chapter.content == sample_chapter_data["content"]
        assert chapter.book_id == book.id
    
    def test_get_book_chapters(self, db_session, sample_book_data, sample_chapter_data):
        """Test retrieving chapters for a book"""
        service = ContentService(db_session)
        
        # Create book
        book = service.create_book(**sample_book_data)
        
        # Create multiple chapters
        chapter1 = service.create_chapter(
            book_id=book.id,
            title="Chapter 1",
            chapter_number=1,
            content="Content 1"
        )
        chapter2 = service.create_chapter(
            book_id=book.id,
            title="Chapter 2", 
            chapter_number=2,
            content="Content 2"
        )
        
        # Get chapters
        chapters = service.get_book_chapters(book.id)
        
        assert len(chapters) == 2
        assert chapters[0].chapter_number == 1
        assert chapters[1].chapter_number == 2
    
    def test_update_chapter(self, db_session, sample_book_data, sample_chapter_data):
        """Test updating a chapter"""
        service = ContentService(db_session)
        
        # Create book and chapter
        book = service.create_book(**sample_book_data)
        chapter = service.create_chapter(book_id=book.id, **sample_chapter_data)
        
        # Update chapter
        updated_data = {
            "title": "Updated Chapter Title",
            "content": "Updated chapter content"
        }
        
        updated_chapter = service.update_chapter(chapter.id, **updated_data)
        
        assert updated_chapter.title == updated_data["title"]
        assert updated_chapter.content == updated_data["content"]
        assert updated_chapter.chapter_number == sample_chapter_data["chapter_number"]
    
    def test_delete_chapter(self, db_session, sample_book_data, sample_chapter_data):
        """Test deleting a chapter"""
        service = ContentService(db_session)
        
        # Create book and chapter
        book = service.create_book(**sample_book_data)
        chapter = service.create_chapter(book_id=book.id, **sample_chapter_data)
        
        # Delete chapter
        result = service.delete_chapter(chapter.id)
        
        assert result is True
        
        # Verify chapter is deleted
        deleted_chapter = service.get_chapter(chapter.id)
        assert deleted_chapter is None
    
    @patch('app.services.content_service.genai')
    def test_personalize_content(self, mock_genai, db_session, sample_book_data, sample_chapter_data):
        """Test content personalization"""
        service = ContentService(db_session)
        
        # Mock Gemini response
        mock_response = Mock()
        mock_response.text = "Personalized content for developer"
        mock_genai.GenerativeModel.return_value.generate_content.return_value = mock_response
        
        # Create book and chapter
        book = service.create_book(**sample_book_data)
        chapter = service.create_chapter(book_id=book.id, **sample_chapter_data)
        
        # Test personalization
        user_preferences = {
            "persona": "developer",
            "experience_level": "intermediate"
        }
        
        personalized_content = service.personalize_content(
            chapter.content, 
            user_preferences
        )
        
        assert personalized_content == "Personalized content for developer"
        mock_genai.GenerativeModel.return_value.generate_content.assert_called_once()