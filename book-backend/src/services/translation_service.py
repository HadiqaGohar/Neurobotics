"""
Multi-lingual translation service for content localization.
"""

import os
import json
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime, timedelta
import asyncio
from functools import lru_cache
import hashlib

# For translation APIs
import requests
from googletrans import Translator
import openai

from book_backend.src.security.security_utils import SecurityValidator
from book_backend.src.performance.optimization import cache

logger = logging.getLogger(__name__)


class TranslationService:
    """Service for translating content to multiple languages."""
    
    def __init__(self):
        self.google_translator = Translator()
        self.openai_client = None
        
        # Initialize OpenAI client if API key is available
        openai_key = os.getenv("OPENAI_API_KEY")
        if openai_key:
            openai.api_key = openai_key
            self.openai_client = openai
        
        # Supported languages with their codes and names
        self.supported_languages = {
            "en": {
                "name": "English",
                "native_name": "English",
                "rtl": False,
                "google_code": "en"
            },
            "ur": {
                "name": "Urdu",
                "native_name": "اردو",
                "rtl": True,
                "google_code": "ur"
            },
            "ar": {
                "name": "Arabic",
                "native_name": "العربية",
                "rtl": True,
                "google_code": "ar"
            },
            "es": {
                "name": "Spanish",
                "native_name": "Español",
                "rtl": False,
                "google_code": "es"
            },
            "fr": {
                "name": "French",
                "native_name": "Français",
                "rtl": False,
                "google_code": "fr"
            },
            "de": {
                "name": "German",
                "native_name": "Deutsch",
                "rtl": False,
                "google_code": "de"
            },
            "zh": {
                "name": "Chinese",
                "native_name": "中文",
                "rtl": False,
                "google_code": "zh"
            },
            "hi": {
                "name": "Hindi",
                "native_name": "हिन्दी",
                "rtl": False,
                "google_code": "hi"
            },
            "ja": {
                "name": "Japanese",
                "native_name": "日本語",
                "rtl": False,
                "google_code": "ja"
            },
            "ko": {
                "name": "Korean",
                "native_name": "한국어",
                "rtl": False,
                "google_code": "ko"
            }
        }
        
        # Translation quality preferences
        self.translation_methods = {
            "google": {"priority": 1, "cost": "free", "quality": "good"},
            "openai": {"priority": 2, "cost": "paid", "quality": "excellent"},
            "cached": {"priority": 0, "cost": "free", "quality": "exact"}
        }
        
        # Language-specific formatting rules
        self.formatting_rules = {
            "ur": {
                "number_format": "eastern_arabic",
                "date_format": "islamic",
                "punctuation": "urdu_style"
            },
            "ar": {
                "number_format": "eastern_arabic", 
                "date_format": "hijri",
                "punctuation": "arabic_style"
            }
        }

    def get_supported_languages(self) -> Dict[str, Dict[str, Any]]:
        """Get list of supported languages."""
        return self.supported_languages

    def is_language_supported(self, language_code: str) -> bool:
        """Check if a language is supported."""
        return language_code in self.supported_languages

    def _generate_cache_key(self, text: str, source_lang: str, target_lang: str) -> str:
        """Generate cache key for translation."""
        content = f"{text}:{source_lang}:{target_lang}"
        return hashlib.md5(content.encode()).hexdigest()

    async def translate_text(
        self, 
        text: str, 
        target_language: str, 
        source_language: str = "en",
        method: str = "auto",
        preserve_formatting: bool = True
    ) -> Dict[str, Any]:
        """
        Translate text to target language.
        
        Args:
            text: Text to translate
            target_language: Target language code
            source_language: Source language code (default: en)
            method: Translation method (auto, google, openai)
            preserve_formatting: Whether to preserve markdown/HTML formatting
        
        Returns:
            Dict with translated text and metadata
        """
        try:
            # Validate inputs
            if not text or not text.strip():
                return {"translated_text": "", "method": "none", "confidence": 0}
            
            # Sanitize input
            text = SecurityValidator.sanitize_input(text)
            if len(text) > 10000:  # Limit text length
                text = text[:10000]
            
            # Check if translation is needed
            if source_language == target_language:
                return {
                    "translated_text": text,
                    "method": "none",
                    "confidence": 1.0,
                    "source_language": source_language,
                    "target_language": target_language
                }
            
            # Validate language support
            if not self.is_language_supported(target_language):
                raise ValueError(f"Unsupported target language: {target_language}")
            
            # Check cache first
            cache_key = self._generate_cache_key(text, source_language, target_language)
            cached_result = cache.get(cache_key)
            if cached_result:
                return {
                    **cached_result,
                    "method": "cached"
                }
            
            # Choose translation method
            if method == "auto":
                method = self._choose_best_method(text, target_language)
            
            # Perform translation
            if method == "openai" and self.openai_client:
                result = await self._translate_with_openai(text, source_language, target_language)
            else:
                result = await self._translate_with_google(text, source_language, target_language)
            
            # Apply language-specific formatting
            if preserve_formatting and target_language in self.formatting_rules:
                result["translated_text"] = self._apply_formatting_rules(
                    result["translated_text"], 
                    target_language
                )
            
            # Cache the result
            cache.set(cache_key, result, ttl=86400)  # Cache for 24 hours
            
            return result
            
        except Exception as e:
            logger.error(f"Translation error: {e}")
            return {
                "translated_text": text,  # Fallback to original
                "method": "error",
                "confidence": 0,
                "error": str(e)
            }

    def _choose_best_method(self, text: str, target_language: str) -> str:
        """Choose the best translation method based on text and language."""
        # For technical content or Urdu, prefer OpenAI if available
        if self.openai_client and (
            target_language in ["ur", "ar"] or 
            any(term in text.lower() for term in ["code", "api", "function", "algorithm"])
        ):
            return "openai"
        
        # Default to Google Translate
        return "google"

    async def _translate_with_google(
        self, 
        text: str, 
        source_lang: str, 
        target_lang: str
    ) -> Dict[str, Any]:
        """Translate using Google Translate."""
        try:
            # Get Google language codes
            source_code = self.supported_languages[source_lang]["google_code"]
            target_code = self.supported_languages[target_lang]["google_code"]
            
            # Perform translation
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                lambda: self.google_translator.translate(
                    text, 
                    src=source_code, 
                    dest=target_code
                )
            )
            
            return {
                "translated_text": result.text,
                "method": "google",
                "confidence": getattr(result, 'confidence', 0.8),
                "source_language": source_lang,
                "target_language": target_lang,
                "detected_language": getattr(result, 'src', source_lang)
            }
            
        except Exception as e:
            logger.error(f"Google Translate error: {e}")
            raise

    async def _translate_with_openai(
        self, 
        text: str, 
        source_lang: str, 
        target_lang: str
    ) -> Dict[str, Any]:
        """Translate using OpenAI GPT for better quality."""
        try:
            source_name = self.supported_languages[source_lang]["name"]
            target_name = self.supported_languages[target_lang]["name"]
            
            # Create translation prompt
            prompt = f"""Translate the following {source_name} text to {target_name}. 
Maintain the original meaning, tone, and any technical terms. 
If the text contains code or technical terms, preserve them accurately.
For Urdu translation, use proper Urdu script and grammar.

Text to translate:
{text}

Translation:"""

            # Make API call
            response = await self.openai_client.ChatCompletion.acreate(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": f"You are a professional translator specializing in {target_name} translation."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=2000,
                temperature=0.3
            )
            
            translated_text = response.choices[0].message.content.strip()
            
            return {
                "translated_text": translated_text,
                "method": "openai",
                "confidence": 0.9,
                "source_language": source_lang,
                "target_language": target_lang,
                "model": "gpt-3.5-turbo"
            }
            
        except Exception as e:
            logger.error(f"OpenAI translation error: {e}")
            # Fallback to Google Translate
            return await self._translate_with_google(text, source_lang, target_lang)

    def _apply_formatting_rules(self, text: str, language: str) -> str:
        """Apply language-specific formatting rules."""
        if language not in self.formatting_rules:
            return text
        
        rules = self.formatting_rules[language]
        
        # Apply number formatting
        if rules.get("number_format") == "eastern_arabic":
            text = self._convert_to_eastern_arabic_numerals(text)
        
        # Apply punctuation rules
        if rules.get("punctuation") == "urdu_style":
            text = self._apply_urdu_punctuation(text)
        elif rules.get("punctuation") == "arabic_style":
            text = self._apply_arabic_punctuation(text)
        
        return text

    def _convert_to_eastern_arabic_numerals(self, text: str) -> str:
        """Convert Western numerals to Eastern Arabic numerals."""
        western_to_eastern = {
            '0': '۰', '1': '۱', '2': '۲', '3': '۳', '4': '۴',
            '5': '۵', '6': '۶', '7': '۷', '8': '۸', '9': '۹'
        }
        
        for western, eastern in western_to_eastern.items():
            text = text.replace(western, eastern)
        
        return text

    def _apply_urdu_punctuation(self, text: str) -> str:
        """Apply Urdu-specific punctuation rules."""
        # Replace question mark with Urdu question mark
        text = text.replace('?', '؟')
        
        # Add proper spacing for Urdu text
        # This is a simplified implementation
        return text

    def _apply_arabic_punctuation(self, text: str) -> str:
        """Apply Arabic-specific punctuation rules."""
        # Replace question mark and semicolon
        text = text.replace('?', '؟')
        text = text.replace(';', '؛')
        
        return text

    async def translate_batch(
        self, 
        texts: List[str], 
        target_language: str, 
        source_language: str = "en"
    ) -> List[Dict[str, Any]]:
        """Translate multiple texts in batch."""
        try:
            # Limit batch size
            if len(texts) > 50:
                texts = texts[:50]
            
            # Create translation tasks
            tasks = []
            for text in texts:
                task = self.translate_text(text, target_language, source_language)
                tasks.append(task)
            
            # Execute translations concurrently
            results = await asyncio.gather(*tasks, return_exceptions=True)
            
            # Handle exceptions
            processed_results = []
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    logger.error(f"Batch translation error for text {i}: {result}")
                    processed_results.append({
                        "translated_text": texts[i],
                        "method": "error",
                        "confidence": 0,
                        "error": str(result)
                    })
                else:
                    processed_results.append(result)
            
            return processed_results
            
        except Exception as e:
            logger.error(f"Batch translation error: {e}")
            # Return original texts as fallback
            return [
                {
                    "translated_text": text,
                    "method": "error",
                    "confidence": 0,
                    "error": str(e)
                }
                for text in texts
            ]

    async def detect_language(self, text: str) -> Dict[str, Any]:
        """Detect the language of given text."""
        try:
            if not text or not text.strip():
                return {"language": "unknown", "confidence": 0}
            
            # Use Google Translate for detection
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                lambda: self.google_translator.detect(text)
            )
            
            detected_lang = result.lang
            confidence = result.confidence
            
            # Map to our supported languages
            for code, info in self.supported_languages.items():
                if info["google_code"] == detected_lang:
                    return {
                        "language": code,
                        "language_name": info["name"],
                        "confidence": confidence
                    }
            
            return {
                "language": detected_lang,
                "language_name": detected_lang,
                "confidence": confidence
            }
            
        except Exception as e:
            logger.error(f"Language detection error: {e}")
            return {"language": "unknown", "confidence": 0, "error": str(e)}

    def get_language_info(self, language_code: str) -> Optional[Dict[str, Any]]:
        """Get information about a specific language."""
        return self.supported_languages.get(language_code)

    async def translate_structured_content(
        self, 
        content: Dict[str, Any], 
        target_language: str, 
        source_language: str = "en"
    ) -> Dict[str, Any]:
        """Translate structured content (JSON) while preserving structure."""
        try:
            translated_content = {}
            
            for key, value in content.items():
                if isinstance(value, str):
                    # Translate string values
                    result = await self.translate_text(value, target_language, source_language)
                    translated_content[key] = result["translated_text"]
                elif isinstance(value, dict):
                    # Recursively translate nested dictionaries
                    translated_content[key] = await self.translate_structured_content(
                        value, target_language, source_language
                    )
                elif isinstance(value, list):
                    # Handle lists
                    translated_list = []
                    for item in value:
                        if isinstance(item, str):
                            result = await self.translate_text(item, target_language, source_language)
                            translated_list.append(result["translated_text"])
                        elif isinstance(item, dict):
                            translated_item = await self.translate_structured_content(
                                item, target_language, source_language
                            )
                            translated_list.append(translated_item)
                        else:
                            translated_list.append(item)
                    translated_content[key] = translated_list
                else:
                    # Keep non-string values as-is
                    translated_content[key] = value
            
            return translated_content
            
        except Exception as e:
            logger.error(f"Structured content translation error: {e}")
            return content  # Return original on error

    def get_translation_stats(self) -> Dict[str, Any]:
        """Get translation service statistics."""
        return {
            "supported_languages": len(self.supported_languages),
            "available_methods": list(self.translation_methods.keys()),
            "cache_size": cache.size(),
            "rtl_languages": [
                code for code, info in self.supported_languages.items() 
                if info.get("rtl", False)
            ]
        }

    async def validate_translation_quality(
        self, 
        original: str, 
        translated: str, 
        target_language: str
    ) -> Dict[str, Any]:
        """Validate translation quality using back-translation."""
        try:
            # Perform back-translation
            back_translation = await self.translate_text(
                translated, 
                "en", 
                target_language
            )
            
            # Simple similarity check (can be enhanced with more sophisticated methods)
            original_words = set(original.lower().split())
            back_words = set(back_translation["translated_text"].lower().split())
            
            # Calculate Jaccard similarity
            intersection = len(original_words.intersection(back_words))
            union = len(original_words.union(back_words))
            similarity = intersection / union if union > 0 else 0
            
            return {
                "back_translation": back_translation["translated_text"],
                "similarity_score": similarity,
                "quality_estimate": "good" if similarity > 0.6 else "fair" if similarity > 0.3 else "poor"
            }
            
        except Exception as e:
            logger.error(f"Translation validation error: {e}")
            return {
                "back_translation": "",
                "similarity_score": 0,
                "quality_estimate": "unknown",
                "error": str(e)
            }