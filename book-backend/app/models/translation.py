"""Pydantic models for translation features"""

from typing import Optional, List, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field


class TranslationRequest(BaseModel):
    """Model for translation request"""
    text: str = Field(..., min_length=1, max_length=50000)
    target_language: str = Field(..., regex=r'^[a-z]{2}$')
    source_language: str = Field("en", regex=r'^[a-z]{2}$')
    use_cache: bool = Field(True)
    
    class Config:
        schema_extra = {
            "example": {
                "text": "This is a sample text to translate",
                "target_language": "ur",
                "source_language": "en",
                "use_cache": True
            }
        }


class TranslationResponse(BaseModel):
    """Model for translation response"""
    original_text: str
    translated_text: str
    source_language: str
    target_language: str
    method: str
    confidence_score: float = Field(..., ge=0, le=1)
    cached: bool
    
    class Config:
        schema_extra = {
            "example": {
                "original_text": "This is a sample text to translate",
                "translated_text": "یہ ترجمہ کرنے کے لیے ایک نمونہ متن ہے",
                "source_language": "en",
                "target_language": "ur",
                "method": "gemini",
                "confidence_score": 0.85,
                "cached": False
            }
        }


class ChapterTranslationRequest(BaseModel):
    """Model for chapter translation request"""
    target_language: str = Field(..., regex=r'^[a-z]{2}$')
    force_retranslate: bool = Field(False)
    
    class Config:
        schema_extra = {
            "example": {
                "target_language": "ur",
                "force_retranslate": False
            }
        }


class TranslatedChapter(BaseModel):
    """Model for translated chapter response"""
    chapter_id: int
    original_title: str
    original_content: str
    translated_title: str
    translated_content: str
    target_language: str
    translated_at: Optional[str]
    confidence_score: Optional[float]
    method: Optional[str]
    cached: bool
    
    class Config:
        schema_extra = {
            "example": {
                "chapter_id": 123,
                "original_title": "Introduction to Algorithms",
                "original_content": "This chapter covers basic algorithms...",
                "translated_title": "الگورتھم کا تعارف",
                "translated_content": "یہ باب بنیادی الگورتھم کا احاطہ کرتا ہے...",
                "target_language": "ur",
                "translated_at": "2025-12-07T12:00:00",
                "confidence_score": 0.85,
                "method": "gemini",
                "cached": False
            }
        }


class BatchTranslationRequest(BaseModel):
    """Model for batch translation request"""
    target_language: str = Field(..., regex=r'^[a-z]{2}$')
    chapter_ids: Optional[List[int]] = Field(None, max_items=100)
    
    class Config:
        schema_extra = {
            "example": {
                "target_language": "ur",
                "chapter_ids": [1, 2, 3, 4, 5]
            }
        }


class BatchTranslationResponse(BaseModel):
    """Model for batch translation response"""
    book_id: int
    book_title: str
    target_language: str
    total_chapters: int
    translated_chapters: int
    failed_chapters: int
    results: List[Dict[str, Any]]
    
    class Config:
        schema_extra = {
            "example": {
                "book_id": 1,
                "book_title": "Complete Guide to Programming",
                "target_language": "ur",
                "total_chapters": 10,
                "translated_chapters": 8,
                "failed_chapters": 2,
                "results": [
                    {
                        "chapter_id": 1,
                        "chapter_title": "Introduction",
                        "status": "success",
                        "cached": False
                    }
                ]
            }
        }


class TranslationStatus(BaseModel):
    """Model for translation status"""
    language_code: str
    total_chapters: int
    translated_chapters: int
    completion_percentage: float
    chapters: List[Dict[str, Any]]
    
    class Config:
        schema_extra = {
            "example": {
                "language_code": "ur",
                "total_chapters": 10,
                "translated_chapters": 7,
                "completion_percentage": 70.0,
                "chapters": [
                    {
                        "chapter_id": 1,
                        "chapter_number": 1,
                        "title": "Introduction",
                        "translated_at": "2025-12-07T12:00:00",
                        "confidence_score": 0.85
                    }
                ]
            }
        }


class BookTranslationStatus(BaseModel):
    """Model for book translation status"""
    book_id: int
    book_title: str
    total_chapters: int
    translation_status: Dict[str, TranslationStatus]
    
    class Config:
        schema_extra = {
            "example": {
                "book_id": 1,
                "book_title": "Complete Guide to Programming",
                "total_chapters": 10,
                "translation_status": {
                    "ur": {
                        "language_code": "ur",
                        "total_chapters": 10,
                        "translated_chapters": 7,
                        "completion_percentage": 70.0,
                        "chapters": []
                    }
                }
            }
        }


class TranslationStats(BaseModel):
    """Model for translation statistics"""
    total_translations: int
    language_pairs: List[Dict[str, Any]]
    methods: Dict[str, int]
    average_confidence: float
    supported_languages: Dict[str, str]
    
    class Config:
        schema_extra = {
            "example": {
                "total_translations": 1250,
                "language_pairs": [
                    {
                        "source": "en",
                        "target": "ur",
                        "count": 450
                    }
                ],
                "methods": {
                    "gemini": 1100,
                    "manual": 150
                },
                "average_confidence": 0.82,
                "supported_languages": {
                    "en": "English",
                    "ur": "Urdu",
                    "hi": "Hindi"
                }
            }
        }


class SupportedLanguages(BaseModel):
    """Model for supported languages"""
    languages: Dict[str, str]
    
    class Config:
        schema_extra = {
            "example": {
                "languages": {
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
            }
        }