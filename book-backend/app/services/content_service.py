"""Content service for Book and Chapter CRUD operations"""

from typing import Optional, List, Dict, Any
from datetime import datetime
from sqlalchemy.orm import Session
from sqlalchemy import func, and_, or_
import logging


from src.database.book_models import Book, Chapter
from src.database.user_models import User

from app.models.book import BookCreate, BookUpdate, ChapterCreate, ChapterUpdate
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class ContentService:
    """Service for content management operations"""
    
    def __init__(self, db: Session):
        self.db = db
    
    # Book CRUD operations
    def create_book(self, book_data: BookCreate, author_id: int) -> Book:
        """Create a new book"""
        try:
            db_book = Book(
                title=book_data.title,
                description=book_data.description,
                author_id=author_id,
                isbn=book_data.isbn,
                language=book_data.language,
                category=book_data.category,
                tags=book_data.tags,
                is_published=book_data.is_published,
                is_featured=book_data.is_featured,
                created_at=datetime.utcnow()
            )
            
            self.db.add(db_book)
            self.db.commit()
            self.db.refresh(db_book)
            
            logger.info(f"Created book: {db_book.title} (ID: {db_book.id})")
            return db_book
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error creating book: {e}")
            raise
    
    def get_book(self, book_id: int) -> Optional[Book]:
        """Get book by ID"""
        return self.db.query(Book).filter(Book.id == book_id).first()
    
    def get_book_by_isbn(self, isbn: str) -> Optional[Book]:
        """Get book by ISBN"""
        return self.db.query(Book).filter(Book.isbn == isbn).first()
    
    def get_books(
        self, 
        skip: int = 0, 
        limit: int = 20,
        author_id: Optional[int] = None,
        is_published: Optional[bool] = None,
        category: Optional[str] = None,
        language: Optional[str] = None,
        search_query: Optional[str] = None
    ) -> List[Book]:
        """Get books with optional filters"""
        query = self.db.query(Book)
        
        # Apply filters
        if author_id:
            query = query.filter(Book.author_id == author_id)
        
        if is_published is not None:
            query = query.filter(Book.is_published == is_published)
        
        if category:
            query = query.filter(Book.category == category)
        
        if language:
            query = query.filter(Book.language == language)
        
        if search_query:
            search_filter = or_(
                Book.title.ilike(f"%{search_query}%"),
                Book.description.ilike(f"%{search_query}%")
            )
            query = query.filter(search_filter)
        
        return query.offset(skip).limit(limit).all()
    
    def update_book(self, book_id: int, book_data: BookUpdate) -> Optional[Book]:
        """Update book"""
        try:
            db_book = self.get_book(book_id)
            if not db_book:
                return None
            
            update_data = book_data.dict(exclude_unset=True)
            
            for field, value in update_data.items():
                setattr(db_book, field, value)
            
            db_book.updated_at = datetime.utcnow()
            
            self.db.commit()
            self.db.refresh(db_book)
            
            logger.info(f"Updated book: {db_book.title} (ID: {db_book.id})")
            return db_book
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating book {book_id}: {e}")
            raise
    
    def delete_book(self, book_id: int) -> bool:
        """Delete book and all its chapters"""
        try:
            db_book = self.get_book(book_id)
            if not db_book:
                return False
            
            # Delete all chapters first (cascade should handle this, but being explicit)
            self.db.query(Chapter).filter(Chapter.book_id == book_id).delete()
            
            # Delete the book
            self.db.delete(db_book)
            self.db.commit()
            
            logger.info(f"Deleted book: {db_book.title} (ID: {book_id})")
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error deleting book {book_id}: {e}")
            return False
    
    def get_book_stats(self, book_id: int) -> Dict[str, Any]:
        """Get book statistics"""
        book = self.get_book(book_id)
        if not book:
            return {}
        
        chapter_count = self.db.query(func.count(Chapter.id)).filter(
            Chapter.book_id == book_id
        ).scalar()
        
        total_words = self.db.query(func.sum(Chapter.word_count)).filter(
            Chapter.book_id == book_id
        ).scalar() or 0
        
        return {
            "book_id": book_id,
            "title": book.title,
            "chapter_count": chapter_count,
            "total_words": total_words,
            "is_published": book.is_published,
            "created_at": book.created_at,
            "updated_at": book.updated_at
        }
    
    # Chapter CRUD operations
    def create_chapter(self, chapter_data: ChapterCreate, book_id: int) -> Chapter:
        """Create a new chapter"""
        try:
            # Check if book exists
            book = self.get_book(book_id)
            if not book:
                raise ValueError(f"Book with ID {book_id} not found")
            
            # Calculate word count and reading time
            word_count = len(chapter_data.content.split())
            reading_time = max(1, word_count // 200)  # Assume 200 words per minute
            
            db_chapter = Chapter(
                book_id=book_id,
                title=chapter_data.title,
                content=chapter_data.content,
                chapter_number=chapter_data.chapter_number,
                word_count=word_count,
                reading_time=reading_time,
                created_at=datetime.utcnow()
            )
            
            self.db.add(db_chapter)
            self.db.commit()
            self.db.refresh(db_chapter)
            
            logger.info(f"Created chapter: {db_chapter.title} (ID: {db_chapter.id})")
            return db_chapter
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error creating chapter: {e}")
            raise
    
    def get_chapter(self, chapter_id: int) -> Optional[Chapter]:
        """Get chapter by ID"""
        return self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
    
    def get_chapters_by_book(
        self, 
        book_id: int, 
        skip: int = 0, 
        limit: int = 50
    ) -> List[Chapter]:
        """Get chapters for a specific book"""
        return self.db.query(Chapter).filter(
            Chapter.book_id == book_id
        ).order_by(Chapter.chapter_number).offset(skip).limit(limit).all()
    
    def get_chapter_by_number(self, book_id: int, chapter_number: int) -> Optional[Chapter]:
        """Get chapter by book ID and chapter number"""
        return self.db.query(Chapter).filter(
            and_(
                Chapter.book_id == book_id,
                Chapter.chapter_number == chapter_number
            )
        ).first()
    
    def update_chapter(self, chapter_id: int, chapter_data: ChapterUpdate) -> Optional[Chapter]:
        """Update chapter"""
        try:
            db_chapter = self.get_chapter(chapter_id)
            if not db_chapter:
                return None
            
            update_data = chapter_data.dict(exclude_unset=True)
            
            # Recalculate word count and reading time if content is updated
            if 'content' in update_data:
                word_count = len(update_data['content'].split())
                update_data['word_count'] = word_count
                update_data['reading_time'] = max(1, word_count // 200)
            
            for field, value in update_data.items():
                setattr(db_chapter, field, value)
            
            db_chapter.updated_at = datetime.utcnow()
            
            self.db.commit()
            self.db.refresh(db_chapter)
            
            logger.info(f"Updated chapter: {db_chapter.title} (ID: {db_chapter.id})")
            return db_chapter
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating chapter {chapter_id}: {e}")
            raise
    
    def delete_chapter(self, chapter_id: int) -> bool:
        """Delete chapter"""
        try:
            db_chapter = self.get_chapter(chapter_id)
            if not db_chapter:
                return False
            
            self.db.delete(db_chapter)
            self.db.commit()
            
            logger.info(f"Deleted chapter: {db_chapter.title} (ID: {chapter_id})")
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error deleting chapter {chapter_id}: {e}")
            return False
    
    def reorder_chapters(self, book_id: int, chapter_orders: List[Dict[str, int]]) -> bool:
        """
        Reorder chapters in a book
        
        Args:
            book_id: Book ID
            chapter_orders: List of dicts with 'chapter_id' and 'new_number'
        """
        try:
            for order_data in chapter_orders:
                chapter_id = order_data['chapter_id']
                new_number = order_data['new_number']
                
                chapter = self.get_chapter(chapter_id)
                if chapter and chapter.book_id == book_id:
                    chapter.chapter_number = new_number
                    chapter.updated_at = datetime.utcnow()
            
            self.db.commit()
            logger.info(f"Reordered chapters for book {book_id}")
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error reordering chapters for book {book_id}: {e}")
            return False
    
    def search_chapters(
        self, 
        search_query: str, 
        book_id: Optional[int] = None,
        limit: int = 20
    ) -> List[Chapter]:
        """Search chapters by content or title"""
        query = self.db.query(Chapter)
        
        if book_id:
            query = query.filter(Chapter.book_id == book_id)
        
        search_filter = or_(
            Chapter.title.ilike(f"%{search_query}%"),
            Chapter.content.ilike(f"%{search_query}%")
        )
        
        return query.filter(search_filter).limit(limit).all()
    
    # Personalization methods
    def get_personalized_chapter_content(
        self, 
        chapter_id: int, 
        user_preferences: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """
        Get personalized chapter content based on user preferences
        """
        chapter = self.get_chapter(chapter_id)
        if not chapter:
            return None
        
        # Get user persona and experience level from preferences
        persona = user_preferences.get('persona', 'beginner')
        experience_level = user_preferences.get('experience_level', 'beginner')
        domains_of_interest = user_preferences.get('domains_of_interest', [])
        complexity_preference = user_preferences.get('complexity_preference', 1)  # 1-3 scale
        
        # Apply personalization transformations
        personalized_content = self._apply_personalization_rules(
            chapter.content,
            persona,
            experience_level,
            domains_of_interest,
            complexity_preference
        )
        
        # Generate personalized suggestions
        suggestions = self._generate_content_suggestions(
            chapter,
            persona,
            experience_level,
            domains_of_interest
        )
        
        return {
            "original_content": chapter.content,
            "personalized_content": personalized_content,
            "persona_applied": persona,
            "experience_level": experience_level,
            "suggestions": suggestions,
            "reading_time": chapter.reading_time,
            "word_count": chapter.word_count
        }
    
    def _apply_personalization_rules(
        self,
        content: str,
        persona: str,
        experience_level: str,
        domains_of_interest: List[str],
        complexity_preference: int
    ) -> str:
        """
        Apply personalization rules to content based on user profile
        """
        personalized_content = content
        
        # Add persona-specific context
        if persona == "developer":
            personalized_content = self._add_developer_context(personalized_content)
        elif persona == "researcher":
            personalized_content = self._add_researcher_context(personalized_content)
        elif persona == "student":
            personalized_content = self._add_student_context(personalized_content)
        
        # Adjust complexity based on experience level
        if experience_level == "beginner":
            personalized_content = self._simplify_for_beginners(personalized_content)
        elif experience_level == "advanced":
            personalized_content = self._add_advanced_details(personalized_content)
        
        # Add domain-specific examples if relevant
        if domains_of_interest:
            personalized_content = self._add_domain_examples(
                personalized_content, 
                domains_of_interest
            )
        
        return personalized_content
    
    def _add_developer_context(self, content: str) -> str:
        """Add developer-specific context and examples"""
        # Add code examples, implementation details, best practices
        developer_note = "\n\n💻 **Developer Note:** "
        
        if "algorithm" in content.lower():
            developer_note += "Consider the time complexity and space complexity when implementing this algorithm. "
        
        if "data" in content.lower():
            developer_note += "Think about data structures that would be most efficient for this use case. "
        
        if "system" in content.lower():
            developer_note += "Consider scalability, maintainability, and performance implications. "
        
        return content + developer_note
    
    def _add_researcher_context(self, content: str) -> str:
        """Add researcher-specific context and references"""
        researcher_note = "\n\n🔬 **Research Context:** "
        
        if "method" in content.lower():
            researcher_note += "Consider the methodology and potential biases in this approach. "
        
        if "result" in content.lower():
            researcher_note += "Evaluate the statistical significance and reproducibility of these results. "
        
        researcher_note += "Look for peer-reviewed sources and recent publications on this topic."
        
        return content + researcher_note
    
    def _add_student_context(self, content: str) -> str:
        """Add student-specific learning aids"""
        student_note = "\n\n📚 **Study Tips:** "
        
        student_note += "Take notes on key concepts. "
        student_note += "Try to explain this concept in your own words. "
        student_note += "Look for real-world applications of this knowledge."
        
        return content + student_note
    
    def _simplify_for_beginners(self, content: str) -> str:
        """Simplify content for beginners"""
        # Add explanatory notes for complex terms
        beginner_note = "\n\n🌟 **Beginner's Guide:** "
        beginner_note += "Don't worry if this seems complex at first. "
        beginner_note += "Focus on understanding the main concepts before diving into details. "
        beginner_note += "Feel free to re-read sections that are unclear."
        
        return content + beginner_note
    
    def _add_advanced_details(self, content: str) -> str:
        """Add advanced details for experienced users"""
        advanced_note = "\n\n⚡ **Advanced Insights:** "
        
        if "concept" in content.lower():
            advanced_note += "Consider the theoretical foundations and edge cases. "
        
        if "implementation" in content.lower():
            advanced_note += "Think about optimization opportunities and alternative approaches. "
        
        advanced_note += "Explore connections to related advanced topics."
        
        return content + advanced_note
    
    def _add_domain_examples(self, content: str, domains: List[str]) -> str:
        """Add domain-specific examples"""
        domain_examples = "\n\n🎯 **Relevant Examples:** "
        
        for domain in domains:
            if domain.lower() in ["ai", "machine learning", "ml"]:
                domain_examples += f"In AI/ML: This concept applies to model training and optimization. "
            elif domain.lower() in ["web development", "web", "frontend", "backend"]:
                domain_examples += f"In Web Development: Consider how this applies to user experience and performance. "
            elif domain.lower() in ["data science", "analytics"]:
                domain_examples += f"In Data Science: Think about data preprocessing and analysis pipelines. "
            elif domain.lower() in ["mobile", "app development"]:
                domain_examples += f"In Mobile Development: Consider memory constraints and user interface design. "
        
        return content + domain_examples if len(domain_examples) > len("\n\n🎯 **Relevant Examples:** ") else content
    
    def _generate_content_suggestions(
        self,
        chapter: Chapter,
        persona: str,
        experience_level: str,
        domains_of_interest: List[str]
    ) -> List[str]:
        """Generate personalized content suggestions"""
        suggestions = []
        
        # Persona-based suggestions
        if persona == "developer":
            suggestions.append("Try implementing the concepts discussed in your preferred programming language")
            suggestions.append("Look for open-source projects that demonstrate these principles")
        elif persona == "researcher":
            suggestions.append("Search for recent academic papers on this topic")
            suggestions.append("Consider how this relates to your current research area")
        elif persona == "student":
            suggestions.append("Create flashcards for key terms and concepts")
            suggestions.append("Form a study group to discuss these ideas")
        
        # Experience level suggestions
        if experience_level == "beginner":
            suggestions.append("Start with the basics and don't rush through complex sections")
            suggestions.append("Look up unfamiliar terms and concepts")
        elif experience_level == "advanced":
            suggestions.append("Challenge yourself to find limitations or improvements")
            suggestions.append("Connect this knowledge to advanced topics you already know")
        
        # Domain-specific suggestions
        for domain in domains_of_interest:
            if domain.lower() in ["ai", "machine learning"]:
                suggestions.append("Consider how this applies to your ML projects")
            elif domain.lower() in ["web development"]:
                suggestions.append("Think about implementing this in a web application")
        
        return suggestions[:5]  # Limit to 5 suggestions
    
    def get_translated_chapter_content(
        self, 
        chapter_id: int, 
        target_language: str,
        force_retranslate: bool = False
    ) -> Optional[Dict[str, Any]]:
        """
        Get translated chapter content using TranslationService
        """
        try:
            from app.services.translation_service import TranslationService
            
            chapter = self.get_chapter(chapter_id)
            if not chapter:
                return None
            
            # Check if translation already exists and is not forced
            if not force_retranslate and chapter.translated_content:
                existing_translation = chapter.translated_content.get(target_language)
                if existing_translation:
                    return {
                        "chapter_id": chapter_id,
                        "original_title": chapter.title,
                        "original_content": chapter.content,
                        "translated_title": existing_translation.get("title", chapter.title),
                        "translated_content": existing_translation.get("content", chapter.content),
                        "target_language": target_language,
                        "translated_at": existing_translation.get("translated_at"),
                        "confidence_score": existing_translation.get("confidence_score"),
                        "method": existing_translation.get("method", "database_cache"),
                        "cached": True
                    }
            
            # Use TranslationService to translate
            translation_service = TranslationService(self.db)
            result = translation_service.translate_chapter_content(
                chapter_id, 
                target_language, 
                force_retranslate
            )
            
            # Refresh chapter to get updated data
            self.db.refresh(chapter)
            
            translated_data = chapter.translated_content.get(target_language, {})
            
            return {
                "chapter_id": chapter_id,
                "original_title": chapter.title,
                "original_content": chapter.content,
                "translated_title": translated_data.get("title", chapter.title),
                "translated_content": translated_data.get("content", chapter.content),
                "target_language": target_language,
                "translated_at": translated_data.get("translated_at"),
                "confidence_score": translated_data.get("confidence_score"),
                "method": translated_data.get("method"),
                "cached": result.get("cached", False)
            }
            
        except Exception as e:
            logger.error(f"Error getting translated content for chapter {chapter_id}: {e}")
            return None
    
    def get_available_translations(self, chapter_id: int) -> List[str]:
        """
        Get list of available translations for a chapter
        
        Returns:
            List of language codes that have translations
        """
        chapter = self.get_chapter(chapter_id)
        if not chapter or not chapter.translated_content:
            return []
        
        return list(chapter.translated_content.keys())
    
    def translate_multiple_chapters(
        self,
        book_id: int,
        target_language: str,
        chapter_ids: Optional[List[int]] = None
    ) -> Dict[str, Any]:
        """
        Translate multiple chapters of a book
        
        Args:
            book_id: Book ID
            target_language: Target language code
            chapter_ids: Specific chapter IDs to translate (if None, translates all)
        
        Returns:
            Dict with translation results
        """
        try:
            from app.services.translation_service import TranslationService
            
            # Get book
            book = self.get_book(book_id)
            if not book:
                raise ValueError(f"Book {book_id} not found")
            
            # Get chapters to translate
            if chapter_ids:
                chapters = [self.get_chapter(cid) for cid in chapter_ids]
                chapters = [c for c in chapters if c and c.book_id == book_id]
            else:
                chapters = self.get_chapters_by_book(book_id, limit=1000)  # Get all chapters
            
            if not chapters:
                return {
                    "book_id": book_id,
                    "target_language": target_language,
                    "total_chapters": 0,
                    "translated_chapters": 0,
                    "failed_chapters": 0,
                    "results": []
                }
            
            translation_service = TranslationService(self.db)
            results = []
            translated_count = 0
            failed_count = 0
            
            for chapter in chapters:
                try:
                    result = translation_service.translate_chapter_content(
                        chapter.id,
                        target_language
                    )
                    results.append({
                        "chapter_id": chapter.id,
                        "chapter_title": chapter.title,
                        "status": "success",
                        "cached": result.get("cached", False)
                    })
                    translated_count += 1
                    
                except Exception as e:
                    logger.error(f"Failed to translate chapter {chapter.id}: {e}")
                    results.append({
                        "chapter_id": chapter.id,
                        "chapter_title": chapter.title,
                        "status": "failed",
                        "error": str(e)
                    })
                    failed_count += 1
            
            logger.info(f"Batch translation completed: {translated_count} success, {failed_count} failed")
            
            return {
                "book_id": book_id,
                "book_title": book.title,
                "target_language": target_language,
                "total_chapters": len(chapters),
                "translated_chapters": translated_count,
                "failed_chapters": failed_count,
                "results": results
            }
            
        except Exception as e:
            logger.error(f"Error in batch translation for book {book_id}: {e}")
            raise
    
    def get_book_translation_status(self, book_id: int) -> Dict[str, Any]:
        """
        Get translation status for all chapters in a book
        
        Returns:
            Dict with translation status information
        """
        try:
            book = self.get_book(book_id)
            if not book:
                return {}
            
            chapters = self.get_chapters_by_book(book_id, limit=1000)
            
            translation_status = {}
            total_chapters = len(chapters)
            
            # Supported languages (from TranslationService)
            supported_languages = ["ur", "hi", "ar", "es", "fr", "de", "zh", "ja", "ko"]
            
            for lang in supported_languages:
                translated_chapters = 0
                chapter_details = []
                
                for chapter in chapters:
                    has_translation = (
                        chapter.translated_content and 
                        lang in chapter.translated_content
                    )
                    
                    if has_translation:
                        translated_chapters += 1
                        translation_info = chapter.translated_content[lang]
                        chapter_details.append({
                            "chapter_id": chapter.id,
                            "chapter_number": chapter.chapter_number,
                            "title": chapter.title,
                            "translated_at": translation_info.get("translated_at"),
                            "confidence_score": translation_info.get("confidence_score")
                        })
                    else:
                        chapter_details.append({
                            "chapter_id": chapter.id,
                            "chapter_number": chapter.chapter_number,
                            "title": chapter.title,
                            "translated_at": None,
                            "confidence_score": None
                        })
                
                translation_status[lang] = {
                    "language_code": lang,
                    "total_chapters": total_chapters,
                    "translated_chapters": translated_chapters,
                    "completion_percentage": round((translated_chapters / total_chapters) * 100, 1) if total_chapters > 0 else 0,
                    "chapters": chapter_details
                }
            
            return {
                "book_id": book_id,
                "book_title": book.title,
                "total_chapters": total_chapters,
                "translation_status": translation_status
            }
            
        except Exception as e:
            logger.error(f"Error getting translation status for book {book_id}: {e}")
            return {}