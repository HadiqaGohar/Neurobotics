"""Translation service for multi-language content support"""

from typing import Optional, Dict, Any, List
from datetime import datetime
from sqlalchemy.orm import Session
import logging
import json
import hashlib
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold

from app.models.database import Translation, Chapter
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class TranslationService:
    """Service for handling content translation"""
    
    def __init__(self, db: Session):
        self.db = db
        self.supported_languages = {
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
        self._initialize_gemini()
    
    def _initialize_gemini(self):
        """Initialize Gemini AI for translation"""
        try:
            if settings.gemini_api_key:
                genai.configure(api_key=settings.gemini_api_key)
                self.gemini_model = genai.GenerativeModel('gemini-pro')
                logger.info("Gemini AI initialized for translation")
            else:
                logger.warning("Gemini API key not configured")
                self.gemini_model = None
        except Exception as e:
            logger.error(f"Failed to initialize Gemini AI: {e}")
            self.gemini_model = None
    
    def translate_text(
        self, 
        text: str, 
        target_language: str, 
        source_language: str = "en",
        use_cache: bool = True
    ) -> Dict[str, Any]:
        """
        Translate text to target language
        
        Args:
            text: Text to translate
            target_language: Target language code (e.g., 'ur', 'hi')
            source_language: Source language code (default: 'en')
            use_cache: Whether to use cached translations
        
        Returns:
            Dict with translation result
        """
        try:
            # Validate languages
            if target_language not in self.supported_languages:
                raise ValueError(f"Unsupported target language: {target_language}")
            
            if source_language not in self.supported_languages:
                raise ValueError(f"Unsupported source language: {source_language}")
            
            # If source and target are the same, return original text
            if source_language == target_language:
                return {
                    "original_text": text,
                    "translated_text": text,
                    "source_language": source_language,
                    "target_language": target_language,
                    "method": "no_translation_needed",
                    "confidence_score": 1.0,
                    "cached": False
                }
            
            # Check cache first
            if use_cache:
                cached_translation = self._get_cached_translation(
                    text, source_language, target_language
                )
                if cached_translation:
                    # Update usage count
                    cached_translation.usage_count += 1
                    cached_translation.last_used = datetime.utcnow()
                    self.db.commit()
                    
                    return {
                        "original_text": text,
                        "translated_text": cached_translation.translated_text,
                        "source_language": source_language,
                        "target_language": target_language,
                        "method": cached_translation.translation_method,
                        "confidence_score": cached_translation.confidence_score,
                        "cached": True
                    }
            
            # Perform translation using Gemini AI
            translated_text, confidence_score = self._translate_with_gemini(
                text, source_language, target_language
            )
            
            # Cache the translation
            if use_cache:
                self._cache_translation(
                    text, translated_text, source_language, target_language,
                    "gemini", confidence_score
                )
            
            return {
                "original_text": text,
                "translated_text": translated_text,
                "source_language": source_language,
                "target_language": target_language,
                "method": "gemini",
                "confidence_score": confidence_score,
                "cached": False
            }
            
        except Exception as e:
            logger.error(f"Translation error: {e}")
            raise Exception(f"Translation failed: {str(e)}")
    
    def _translate_with_gemini(
        self, 
        text: str, 
        source_language: str, 
        target_language: str
    ) -> tuple[str, float]:
        """
        Translate text using Gemini AI
        
        Returns:
            Tuple of (translated_text, confidence_score)
        """
        if not self.gemini_model:
            raise Exception("Gemini AI not available")
        
        try:
            source_lang_name = self.supported_languages[source_language]
            target_lang_name = self.supported_languages[target_language]
            
            # Create translation prompt
            prompt = f"""
            Translate the following text from {source_lang_name} to {target_lang_name}.
            
            Instructions:
            1. Maintain the original meaning and context
            2. Preserve any technical terms appropriately
            3. Keep the same tone and style
            4. If translating to Urdu, use proper Urdu script and grammar
            5. For technical content, provide accurate translations of concepts
            6. Maintain any formatting like line breaks
            
            Text to translate:
            {text}
            
            Provide only the translation without any additional commentary.
            """
            
            # Configure safety settings for translation
            safety_settings = {
                HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
                HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_MEDIUM_AND_ABOVE,
            }
            
            # Generate translation
            response = self.gemini_model.generate_content(
                prompt,
                safety_settings=safety_settings,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.1,  # Low temperature for consistent translations
                    max_output_tokens=4000,
                )
            )
            
            if response.text:
                translated_text = response.text.strip()
                
                # Calculate confidence score based on response quality
                confidence_score = self._calculate_confidence_score(
                    text, translated_text, target_language
                )
                
                logger.info(f"Translation completed: {source_language} -> {target_language}")
                return translated_text, confidence_score
            else:
                raise Exception("Empty response from Gemini AI")
                
        except Exception as e:
            logger.error(f"Gemini translation error: {e}")
            raise Exception(f"Gemini translation failed: {str(e)}")
    
    def _calculate_confidence_score(
        self, 
        original_text: str, 
        translated_text: str, 
        target_language: str
    ) -> float:
        """
        Calculate confidence score for translation quality
        
        Returns:
            Confidence score between 0.0 and 1.0
        """
        try:
            # Basic heuristics for confidence scoring
            confidence = 0.7  # Base confidence for Gemini
            
            # Check if translation is not empty
            if not translated_text or len(translated_text.strip()) == 0:
                return 0.0
            
            # Check length ratio (translations shouldn't be too different in length)
            length_ratio = len(translated_text) / len(original_text)
            if 0.5 <= length_ratio <= 2.0:
                confidence += 0.1
            
            # Check for proper script usage (for Urdu)
            if target_language == "ur":
                # Check if Urdu text contains Arabic/Urdu script
                urdu_chars = sum(1 for char in translated_text if '\u0600' <= char <= '\u06FF')
                if urdu_chars > len(translated_text) * 0.3:  # At least 30% Urdu characters
                    confidence += 0.1
            
            # Check if translation is not just the original text
            if translated_text.lower().strip() != original_text.lower().strip():
                confidence += 0.1
            
            return min(1.0, confidence)
            
        except Exception as e:
            logger.error(f"Error calculating confidence score: {e}")
            return 0.5  # Default confidence
    
    def _get_cached_translation(
        self, 
        text: str, 
        source_language: str, 
        target_language: str
    ) -> Optional[Translation]:
        """Get cached translation if available"""
        try:
            # Create hash of the text for efficient lookup
            text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
            
            return self.db.query(Translation).filter(
                Translation.original_text == text,
                Translation.source_language == source_language,
                Translation.target_language == target_language
            ).first()
            
        except Exception as e:
            logger.error(f"Error getting cached translation: {e}")
            return None
    
    def _cache_translation(
        self,
        original_text: str,
        translated_text: str,
        source_language: str,
        target_language: str,
        method: str,
        confidence_score: float
    ):
        """Cache translation in database"""
        try:
            translation = Translation(
                original_text=original_text,
                translated_text=translated_text,
                source_language=source_language,
                target_language=target_language,
                translation_method=method,
                confidence_score=confidence_score,
                usage_count=1,
                created_at=datetime.utcnow(),
                last_used=datetime.utcnow()
            )
            
            self.db.add(translation)
            self.db.commit()
            
            logger.info(f"Translation cached: {source_language} -> {target_language}")
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error caching translation: {e}")
    
    def translate_chapter_content(
        self, 
        chapter_id: int, 
        target_language: str,
        force_retranslate: bool = False
    ) -> Dict[str, Any]:
        """
        Translate chapter content and store in database
        
        Args:
            chapter_id: Chapter ID to translate
            target_language: Target language code
            force_retranslate: Force retranslation even if cached
        
        Returns:
            Dict with translation result
        """
        try:
            # Get chapter
            chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
            if not chapter:
                raise ValueError(f"Chapter {chapter_id} not found")
            
            # Check if translation already exists and is not forced
            if not force_retranslate and chapter.translated_content:
                existing_translation = chapter.translated_content.get(target_language)
                if existing_translation:
                    return {
                        "chapter_id": chapter_id,
                        "target_language": target_language,
                        "translated_content": existing_translation,
                        "cached": True,
                        "translation_method": "database_cache"
                    }
            
            # Translate content
            translation_result = self.translate_text(
                chapter.content,
                target_language,
                source_language="en",
                use_cache=not force_retranslate
            )
            
            # Translate title
            title_translation = self.translate_text(
                chapter.title,
                target_language,
                source_language="en",
                use_cache=not force_retranslate
            )
            
            # Store translation in chapter
            if not chapter.translated_content:
                chapter.translated_content = {}
            
            chapter.translated_content[target_language] = {
                "title": title_translation["translated_text"],
                "content": translation_result["translated_text"],
                "translated_at": datetime.utcnow().isoformat(),
                "confidence_score": translation_result["confidence_score"],
                "method": translation_result["method"]
            }
            
            # Mark the field as modified for SQLAlchemy
            from sqlalchemy.orm.attributes import flag_modified
            flag_modified(chapter, "translated_content")
            
            chapter.updated_at = datetime.utcnow()
            self.db.commit()
            
            logger.info(f"Chapter {chapter_id} translated to {target_language}")
            
            return {
                "chapter_id": chapter_id,
                "target_language": target_language,
                "translated_content": chapter.translated_content[target_language],
                "cached": translation_result["cached"],
                "translation_method": translation_result["method"]
            }
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error translating chapter {chapter_id}: {e}")
            raise Exception(f"Chapter translation failed: {str(e)}")
    
    def get_translation_stats(self) -> Dict[str, Any]:
        """Get translation statistics"""
        try:
            total_translations = self.db.query(Translation).count()
            
            # Language pair statistics
            language_pairs = self.db.query(
                Translation.source_language,
                Translation.target_language,
                self.db.func.count(Translation.id)
            ).group_by(
                Translation.source_language,
                Translation.target_language
            ).all()
            
            # Method statistics
            method_stats = self.db.query(
                Translation.translation_method,
                self.db.func.count(Translation.id)
            ).group_by(Translation.translation_method).all()
            
            # Average confidence score
            avg_confidence = self.db.query(
                self.db.func.avg(Translation.confidence_score)
            ).scalar() or 0.0
            
            return {
                "total_translations": total_translations,
                "language_pairs": [
                    {
                        "source": pair[0],
                        "target": pair[1],
                        "count": pair[2]
                    }
                    for pair in language_pairs
                ],
                "methods": dict(method_stats),
                "average_confidence": round(float(avg_confidence), 3),
                "supported_languages": self.supported_languages
            }
            
        except Exception as e:
            logger.error(f"Error getting translation stats: {e}")
            return {}
    
    def clear_translation_cache(
        self, 
        target_language: Optional[str] = None,
        older_than_days: Optional[int] = None
    ) -> int:
        """
        Clear translation cache
        
        Args:
            target_language: Clear only specific language translations
            older_than_days: Clear translations older than specified days
        
        Returns:
            Number of translations cleared
        """
        try:
            query = self.db.query(Translation)
            
            if target_language:
                query = query.filter(Translation.target_language == target_language)
            
            if older_than_days:
                cutoff_date = datetime.utcnow() - timedelta(days=older_than_days)
                query = query.filter(Translation.created_at < cutoff_date)
            
            count = query.count()
            query.delete()
            self.db.commit()
            
            logger.info(f"Cleared {count} translations from cache")
            return count
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error clearing translation cache: {e}")
            return 0