"""Content management endpoints for books and chapters"""

from typing import List, Optional, Any
from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.services.content_service import ContentService
from app.models.book import (
    Book, BookCreate, BookUpdate, 
    Chapter, ChapterCreate, ChapterUpdate
)
from app.models.personalization import PersonalizedContent, PersonalizationPreferences
from app.models.translation import (
    TranslationRequest, TranslationResponse, ChapterTranslationRequest,
    TranslatedChapter, BatchTranslationRequest, BatchTranslationResponse,
    BookTranslationStatus, TranslationStats, SupportedLanguages
)
from app.models.user import User
from app.api.dependencies import get_current_user, get_current_active_user
import logging

logger = logging.getLogger(__name__)

router = APIRouter()


# Book endpoints
@router.post("/books", response_model=Book, status_code=status.HTTP_201_CREATED)
def create_book(
    book_data: BookCreate,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Create a new book"""
    try:
        content_service = ContentService(db)
        
        # Check if ISBN already exists
        if book_data.isbn:
            existing_book = content_service.get_book_by_isbn(book_data.isbn)
            if existing_book:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Book with this ISBN already exists"
                )
        
        book = content_service.create_book(book_data, current_user.id)
        return book
        
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error creating book: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/books", response_model=List[Book])
def get_books(
    skip: int = Query(0, ge=0),
    limit: int = Query(20, ge=1, le=100),
    author_id: Optional[int] = Query(None),
    is_published: Optional[bool] = Query(None),
    category: Optional[str] = Query(None),
    language: Optional[str] = Query(None),
    search: Optional[str] = Query(None),
    db: Session = Depends(get_db)
):
    """Get books with optional filters"""
    try:
        content_service = ContentService(db)
        books = content_service.get_books(
            skip=skip,
            limit=limit,
            author_id=author_id,
            is_published=is_published,
            category=category,
            language=language,
            search_query=search
        )
        return books
        
    except Exception as e:
        logger.error(f"Error getting books: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/books/{book_id}", response_model=Book)
def get_book(
    book_id: int,
    db: Session = Depends(get_db)
):
    """Get book by ID"""
    try:
        content_service = ContentService(db)
        book = content_service.get_book(book_id)
        
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        return book
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.put("/books/{book_id}", response_model=Book)
def update_book(
    book_id: int,
    book_data: BookUpdate,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Update book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists and user has permission
        existing_book = content_service.get_book(book_id)
        if not existing_book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        # Check if user is the author or superuser
        if existing_book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        # Check ISBN uniqueness if being updated
        if book_data.isbn and book_data.isbn != existing_book.isbn:
            isbn_book = content_service.get_book_by_isbn(book_data.isbn)
            if isbn_book:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Book with this ISBN already exists"
                )
        
        book = content_service.update_book(book_id, book_data)
        return book
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.delete("/books/{book_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_book(
    book_id: int,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Delete book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists and user has permission
        existing_book = content_service.get_book(book_id)
        if not existing_book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        # Check if user is the author or superuser
        if existing_book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        success = content_service.delete_book(book_id)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to delete book"
            )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/books/{book_id}/stats")
def get_book_stats(
    book_id: int,
    db: Session = Depends(get_db)
):
    """Get book statistics"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        stats = content_service.get_book_stats(book_id)
        return stats
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting book stats {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


# Chapter endpoints
@router.post("/books/{book_id}/chapters", response_model=Chapter, status_code=status.HTTP_201_CREATED)
def create_chapter(
    book_id: int,
    chapter_data: ChapterCreate,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Create a new chapter"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists and user has permission
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        if book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        # Check if chapter number already exists
        existing_chapter = content_service.get_chapter_by_number(book_id, chapter_data.chapter_number)
        if existing_chapter:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Chapter with this number already exists"
            )
        
        chapter = content_service.create_chapter(chapter_data, book_id)
        return chapter
        
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error creating chapter: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/books/{book_id}/chapters", response_model=List[Chapter])
def get_chapters(
    book_id: int,
    skip: int = Query(0, ge=0),
    limit: int = Query(50, ge=1, le=100),
    db: Session = Depends(get_db)
):
    """Get chapters for a book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        chapters = content_service.get_chapters_by_book(book_id, skip, limit)
        return chapters
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting chapters for book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/chapters/{chapter_id}", response_model=Chapter)
def get_chapter(
    chapter_id: int,
    db: Session = Depends(get_db)
):
    """Get chapter by ID"""
    try:
        content_service = ContentService(db)
        chapter = content_service.get_chapter(chapter_id)
        
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        return chapter
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.put("/chapters/{chapter_id}", response_model=Chapter)
def update_chapter(
    chapter_id: int,
    chapter_data: ChapterUpdate,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Update chapter"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        existing_chapter = content_service.get_chapter(chapter_id)
        if not existing_chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        # Check if user has permission (book author or superuser)
        book = content_service.get_book(existing_chapter.book_id)
        if book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        # Check chapter number uniqueness if being updated
        if (chapter_data.chapter_number and 
            chapter_data.chapter_number != existing_chapter.chapter_number):
            existing_number = content_service.get_chapter_by_number(
                existing_chapter.book_id, 
                chapter_data.chapter_number
            )
            if existing_number:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Chapter with this number already exists"
                )
        
        chapter = content_service.update_chapter(chapter_id, chapter_data)
        return chapter
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.delete("/chapters/{chapter_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_chapter(
    chapter_id: int,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Delete chapter"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        existing_chapter = content_service.get_chapter(chapter_id)
        if not existing_chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        # Check if user has permission (book author or superuser)
        book = content_service.get_book(existing_chapter.book_id)
        if book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        success = content_service.delete_chapter(chapter_id)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to delete chapter"
            )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/chapters/search")
def search_chapters(
    q: str = Query(..., min_length=1),
    book_id: Optional[int] = Query(None),
    limit: int = Query(20, ge=1, le=50),
    db: Session = Depends(get_db)
):
    """Search chapters by content or title"""
    try:
        content_service = ContentService(db)
        chapters = content_service.search_chapters(q, book_id, limit)
        return chapters
        
    except Exception as e:
        logger.error(f"Error searching chapters: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


# Personalization endpoints
@router.get("/chapters/{chapter_id}/personalized", response_model=PersonalizedContent)
def get_personalized_chapter(
    chapter_id: int,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get personalized chapter content based on user preferences"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        chapter = content_service.get_chapter(chapter_id)
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        # Get user preferences (use defaults if not set)
        user_preferences = current_user.preferences or {}
        default_preferences = {
            "persona": current_user.persona,
            "experience_level": current_user.experience_level,
            "language_preference": current_user.preferred_language,
            "domains_of_interest": [],
            "complexity_preference": 2
        }
        
        # Merge user preferences with defaults
        merged_preferences = {**default_preferences, **user_preferences}
        
        # Get personalized content
        personalized_data = content_service.get_personalized_chapter_content(
            chapter_id, 
            merged_preferences
        )
        
        if not personalized_data:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        return PersonalizedContent(**personalized_data)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting personalized chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/books/{book_id}/chapters/personalized")
def get_personalized_chapters_for_book(
    book_id: int,
    skip: int = Query(0, ge=0),
    limit: int = Query(10, ge=1, le=50),
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get personalized chapters for a book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        # Get chapters
        chapters = content_service.get_chapters_by_book(book_id, skip, limit)
        
        # Get user preferences
        user_preferences = current_user.preferences or {}
        default_preferences = {
            "persona": current_user.persona,
            "experience_level": current_user.experience_level,
            "language_preference": current_user.preferred_language,
            "domains_of_interest": [],
            "complexity_preference": 2
        }
        merged_preferences = {**default_preferences, **user_preferences}
        
        # Get personalized content for each chapter
        personalized_chapters = []
        for chapter in chapters:
            personalized_data = content_service.get_personalized_chapter_content(
                chapter.id, 
                merged_preferences
            )
            if personalized_data:
                personalized_chapters.append({
                    "chapter_id": chapter.id,
                    "chapter_number": chapter.chapter_number,
                    "title": chapter.title,
                    "personalized_content": personalized_data
                })
        
        return {
            "book_id": book_id,
            "book_title": book.title,
            "personalized_chapters": personalized_chapters,
            "total_chapters": len(personalized_chapters),
            "user_preferences": merged_preferences
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting personalized chapters for book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/personalization/preferences")
def update_personalization_preferences(
    preferences: PersonalizationPreferences,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Update user's personalization preferences"""
    try:
        from app.services.user_service import UserService
        
        user_service = UserService(db)
        
        # Convert preferences to dict and merge with existing
        new_preferences = preferences.dict(exclude_unset=True)
        existing_preferences = current_user.preferences or {}
        merged_preferences = {**existing_preferences, **new_preferences}
        
        # Update user preferences
        updated_user = user_service.update_preferences(current_user.id, merged_preferences)
        
        if not updated_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
        
        return {
            "message": "Personalization preferences updated successfully",
            "preferences": updated_user.preferences
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating personalization preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update preferences"
        )


@router.get("/personalization/recommendations")
def get_content_recommendations(
    limit: int = Query(10, ge=1, le=20),
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get personalized content recommendations for user"""
    try:
        content_service = ContentService(db)
        
        # Get user preferences
        user_preferences = current_user.preferences or {}
        persona = user_preferences.get("persona", current_user.persona)
        experience_level = user_preferences.get("experience_level", current_user.experience_level)
        domains_of_interest = user_preferences.get("domains_of_interest", [])
        
        # Get books that match user's interests
        books = content_service.get_books(
            limit=50,  # Get more books to filter from
            is_published=True
        )
        
        recommendations = []
        
        for book in books:
            # Simple recommendation logic based on tags and category
            relevance_score = 0.5  # Base score
            
            # Boost score if book category matches domains of interest
            if book.category and any(domain.lower() in book.category.lower() for domain in domains_of_interest):
                relevance_score += 0.3
            
            # Boost score if book tags match domains of interest
            if book.tags:
                matching_tags = sum(1 for tag in book.tags if any(domain.lower() in tag.lower() for domain in domains_of_interest))
                relevance_score += min(0.2, matching_tags * 0.1)
            
            # Adjust for experience level (simple heuristic)
            if experience_level == "beginner" and "basic" in book.title.lower():
                relevance_score += 0.1
            elif experience_level == "advanced" and "advanced" in book.title.lower():
                relevance_score += 0.1
            
            if relevance_score > 0.6:  # Only recommend if score is decent
                recommendations.append({
                    "book_id": book.id,
                    "title": book.title,
                    "description": book.description,
                    "category": book.category,
                    "relevance_score": min(1.0, relevance_score),
                    "reason": f"Matches your {persona} persona and {experience_level} experience level"
                })
        
        # Sort by relevance score and limit results
        recommendations.sort(key=lambda x: x["relevance_score"], reverse=True)
        recommendations = recommendations[:limit]
        
        return {
            "recommendations": recommendations,
            "total": len(recommendations),
            "user_persona": persona,
            "experience_level": experience_level,
            "domains_of_interest": domains_of_interest
        }
        
    except Exception as e:
        logger.error(f"Error getting content recommendations: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get recommendations"
        )
# Translation endpoints
@router.post("/translate", response_model=TranslationResponse)
def translate_text(
    translation_request: TranslationRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Translate text to target language"""
    try:
        from app.services.translation_service import TranslationService
        
        translation_service = TranslationService(db)
        result = translation_service.translate_text(
            text=translation_request.text,
            target_language=translation_request.target_language,
            source_language=translation_request.source_language,
            use_cache=translation_request.use_cache
        )
        
        return TranslationResponse(**result)
        
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Translation failed"
        )


@router.get("/chapters/{chapter_id}/translate/{target_language}", response_model=TranslatedChapter)
def get_translated_chapter(
    chapter_id: int,
    target_language: str,
    force_retranslate: bool = Query(False),
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get translated chapter content"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        chapter = content_service.get_chapter(chapter_id)
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        # Get translated content
        translated_data = content_service.get_translated_chapter_content(
            chapter_id, 
            target_language,
            force_retranslate
        )
        
        if not translated_data:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Translation failed"
            )
        
        return TranslatedChapter(**translated_data)
        
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error getting translated chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Translation failed"
        )


@router.post("/chapters/{chapter_id}/translate", response_model=TranslatedChapter)
def translate_chapter(
    chapter_id: int,
    translation_request: ChapterTranslationRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Translate chapter content"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        chapter = content_service.get_chapter(chapter_id)
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        # Check if user has permission (book author or superuser)
        book = content_service.get_book(chapter.book_id)
        if book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        # Get translated content
        translated_data = content_service.get_translated_chapter_content(
            chapter_id,
            translation_request.target_language,
            translation_request.force_retranslate
        )
        
        if not translated_data:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Translation failed"
            )
        
        return TranslatedChapter(**translated_data)
        
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error translating chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Translation failed"
        )


@router.post("/books/{book_id}/translate", response_model=BatchTranslationResponse)
def translate_book_chapters(
    book_id: int,
    translation_request: BatchTranslationRequest,
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Translate multiple chapters of a book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        # Check if user has permission (book author or superuser)
        if book.author_id != current_user.id and not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        # Perform batch translation
        result = content_service.translate_multiple_chapters(
            book_id,
            translation_request.target_language,
            translation_request.chapter_ids
        )
        
        return BatchTranslationResponse(**result)
        
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error translating book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Batch translation failed"
        )


@router.get("/books/{book_id}/translation-status", response_model=BookTranslationStatus)
def get_book_translation_status(
    book_id: int,
    db: Session = Depends(get_db)
):
    """Get translation status for all chapters in a book"""
    try:
        content_service = ContentService(db)
        
        # Check if book exists
        book = content_service.get_book(book_id)
        if not book:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Book not found"
            )
        
        status_data = content_service.get_book_translation_status(book_id)
        
        if not status_data:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to get translation status"
            )
        
        return BookTranslationStatus(**status_data)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting translation status for book {book_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get translation status"
        )


@router.get("/chapters/{chapter_id}/translations")
def get_chapter_available_translations(
    chapter_id: int,
    db: Session = Depends(get_db)
):
    """Get list of available translations for a chapter"""
    try:
        content_service = ContentService(db)
        
        # Check if chapter exists
        chapter = content_service.get_chapter(chapter_id)
        if not chapter:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chapter not found"
            )
        
        available_languages = content_service.get_available_translations(chapter_id)
        
        return {
            "chapter_id": chapter_id,
            "chapter_title": chapter.title,
            "available_languages": available_languages,
            "total_translations": len(available_languages)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting available translations for chapter {chapter_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get available translations"
        )


@router.get("/translation/stats", response_model=TranslationStats)
def get_translation_statistics(
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get translation statistics (admin only)"""
    try:
        # Check if user is superuser
        if not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin access required"
            )
        
        from app.services.translation_service import TranslationService
        
        translation_service = TranslationService(db)
        stats = translation_service.get_translation_stats()
        
        return TranslationStats(**stats)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting translation stats: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get translation statistics"
        )


@router.get("/translation/supported-languages", response_model=SupportedLanguages)
def get_supported_languages():
    """Get list of supported languages for translation"""
    try:
        supported_languages = {
            "en": "English",
            "ur": "Urdu",
            "hi": "Hindi",
            "ar": "Arabic",
            "es": "Spanish",
            "fr": "French",
            "de": "German",
            "zh": "Chinese",
            "ja": "Japanese",
            "ko": "Korean"
        }
        
        return SupportedLanguages(languages=supported_languages)
        
    except Exception as e:
        logger.error(f"Error getting supported languages: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get supported languages"
        )


@router.delete("/translation/cache")
def clear_translation_cache(
    target_language: Optional[str] = Query(None),
    older_than_days: Optional[int] = Query(None, ge=1),
    current_user: User = Depends(get_current_active_user),
    db: Session = Depends(get_db)
):
    """Clear translation cache (admin only)"""
    try:
        # Check if user is superuser
        if not current_user.is_superuser:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin access required"
            )
        
        from app.services.translation_service import TranslationService
        
        translation_service = TranslationService(db)
        cleared_count = translation_service.clear_translation_cache(
            target_language=target_language,
            older_than_days=older_than_days
        )
        
        return {
            "message": f"Successfully cleared {cleared_count} translations from cache",
            "cleared_count": cleared_count,
            "target_language": target_language,
            "older_than_days": older_than_days
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error clearing translation cache: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to clear translation cache"
        )