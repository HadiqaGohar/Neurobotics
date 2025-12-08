"""AI Translation Service Integration."""

import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
import json
import hashlib
from enum import Enum

import aiohttp
from sqlalchemy.orm import Session

from app.core.config import get_settings
from app.models.multilingual import TranslationMemory, TerminologyGlossary

logger = logging.getLogger(__name__)
settings = get_settings()


class TranslationProvider(str, Enum):
    """Supported translation providers."""
    GOOGLE = "google"
    AZURE = "azure"
    AWS = "aws"
    OPENAI = "openai"


class TranslationQuality(str, Enum):
    """Translation quality levels."""
    DRAFT = "draft"
    GOOD = "good"
    EXCELLENT = "excellent"


class AITranslationService:
    """AI Translation Service with multiple provider support."""
    
    def __init__(self):
        self.providers = {
            TranslationProvider.GOOGLE: GoogleTranslateProvider(),
            TranslationProvider.AZURE: AzureTranslateProvider(),
            TranslationProvider.AWS: AWSTranslateProvider(),
            TranslationProvider.OPENAI: OpenAITranslateProvider(),
        }
        self.default_provider = TranslationProvider.GOOGLE
        self.cache = {}
        self.rate_limits = {}
    
    async def translate_text(
        self,
        text: str,
        source_language: str,
        target_language: str,
        provider: Optional[TranslationProvider] = None,
        context: Optional[str] = None,
        domain: Optional[str] = None,
        db: Session = None
    ) -> Dict[str, Any]:
        """Translate text using AI service."""
        try:
            # Use default provider if none specified
            if provider is None:
                provider = self.default_provider
            
            # Check cache first
            cache_key = self._generate_cache_key(text, source_language, target_language, provider)
            if cache_key in self.cache:
                logger.info(f"Cache hit for translation: {cache_key[:20]}...")
                return self.cache[cache_key]
            
            # Check translation memory
            if db:
                memory_result = await self._check_translation_memory(
                    text, source_language, target_language, db
                )
                if memory_result:
                    return memory_result
            
            # Check rate limits
            if not await self._check_rate_limit(provider):
                logger.warning(f"Rate limit exceeded for provider {provider}")
                # Try fallback provider
                fallback_provider = self._get_fallback_provider(provider)
                if fallback_provider and await self._check_rate_limit(fallback_provider):
                    provider = fallback_provider
                else:
                    raise Exception("All translation providers rate limited")
            
            # Get terminology suggestions
            terminology = await self._get_terminology_suggestions(
                text, source_language, target_language, domain, db
            ) if db else {}
            
            # Perform translation
            translation_provider = self.providers[provider]
            result = await translation_provider.translate(
                text=text,
                source_language=source_language,
                target_language=target_language,
                context=context,
                terminology=terminology
            )
            
            # Enhance result with quality metrics
            enhanced_result = await self._enhance_translation_result(
                result, text, source_language, target_language, provider
            )
            
            # Cache result
            self.cache[cache_key] = enhanced_result
            
            # Add to translation memory if quality is good
            if db and enhanced_result.get("quality_score", 0) >= 0.7:
                await self._add_to_memory(
                    text, enhanced_result["translated_text"],
                    source_language, target_language,
                    enhanced_result.get("quality_score"),
                    context, domain, db
                )
            
            # Update rate limit tracking
            await self._update_rate_limit(provider)
            
            return enhanced_result
            
        except Exception as e:
            logger.error(f"Translation error: {e}")
            return {
                "translated_text": text,  # Fallback to original
                "provider": provider.value if provider else "none",
                "quality_score": 0.0,
                "confidence": 0.0,
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat()
            }
    
    async def translate_batch(
        self,
        texts: List[str],
        source_language: str,
        target_language: str,
        provider: Optional[TranslationProvider] = None,
        context: Optional[str] = None,
        domain: Optional[str] = None,
        db: Session = None
    ) -> List[Dict[str, Any]]:
        """Translate multiple texts in batch."""
        try:
            # Use default provider if none specified
            if provider is None:
                provider = self.default_provider
            
            # Check if provider supports batch translation
            translation_provider = self.providers[provider]
            if hasattr(translation_provider, 'translate_batch'):
                return await translation_provider.translate_batch(
                    texts, source_language, target_language, context
                )
            
            # Fallback to individual translations with concurrency control
            semaphore = asyncio.Semaphore(5)  # Limit concurrent requests
            
            async def translate_single(text):
                async with semaphore:
                    return await self.translate_text(
                        text, source_language, target_language,
                        provider, context, domain, db
                    )
            
            tasks = [translate_single(text) for text in texts]
            results = await asyncio.gather(*tasks, return_exceptions=True)
            
            # Handle exceptions in results
            processed_results = []
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    processed_results.append({
                        "translated_text": texts[i],
                        "error": str(result),
                        "quality_score": 0.0
                    })
                else:
                    processed_results.append(result)
            
            return processed_results
            
        except Exception as e:
            logger.error(f"Batch translation error: {e}")
            return [{"translated_text": text, "error": str(e)} for text in texts]
    
    async def detect_language(
        self,
        text: str,
        provider: Optional[TranslationProvider] = None
    ) -> Dict[str, Any]:
        """Detect language of given text."""
        try:
            if provider is None:
                provider = self.default_provider
            
            translation_provider = self.providers[provider]
            if hasattr(translation_provider, 'detect_language'):
                return await translation_provider.detect_language(text)
            
            # Fallback to simple detection
            return {
                "language": "en",  # Default fallback
                "confidence": 0.5,
                "provider": provider.value
            }
            
        except Exception as e:
            logger.error(f"Language detection error: {e}")
            return {
                "language": "en",
                "confidence": 0.0,
                "error": str(e)
            }
    
    async def get_supported_languages(
        self,
        provider: Optional[TranslationProvider] = None
    ) -> List[Dict[str, str]]:
        """Get supported languages for a provider."""
        try:
            if provider is None:
                provider = self.default_provider
            
            translation_provider = self.providers[provider]
            if hasattr(translation_provider, 'get_supported_languages'):
                return await translation_provider.get_supported_languages()
            
            # Fallback to common languages
            return [
                {"code": "en", "name": "English"},
                {"code": "ur", "name": "Urdu"},
                {"code": "ar", "name": "Arabic"},
                {"code": "es", "name": "Spanish"},
                {"code": "fr", "name": "French"},
            ]
            
        except Exception as e:
            logger.error(f"Error getting supported languages: {e}")
            return []
    
    def _generate_cache_key(
        self,
        text: str,
        source_language: str,
        target_language: str,
        provider: TranslationProvider
    ) -> str:
        """Generate cache key for translation."""
        content = f"{text}:{source_language}:{target_language}:{provider.value}"
        return hashlib.md5(content.encode()).hexdigest()
    
    async def _check_translation_memory(
        self,
        text: str,
        source_language: str,
        target_language: str,
        db: Session
    ) -> Optional[Dict[str, Any]]:
        """Check translation memory for existing translation."""
        try:
            memory = db.query(TranslationMemory).filter(
                TranslationMemory.source_text == text,
                TranslationMemory.source_language == source_language,
                TranslationMemory.target_language == target_language
            ).first()
            
            if memory:
                # Update usage count
                memory.usage_count += 1
                db.commit()
                
                return {
                    "translated_text": memory.target_text,
                    "provider": "memory",
                    "quality_score": memory.quality_score or 0.8,
                    "confidence": 0.9,
                    "from_memory": True,
                    "usage_count": memory.usage_count,
                    "timestamp": datetime.utcnow().isoformat()
                }
            
            return None
            
        except Exception as e:
            logger.error(f"Error checking translation memory: {e}")
            return None
    
    async def _get_terminology_suggestions(
        self,
        text: str,
        source_language: str,
        target_language: str,
        domain: Optional[str],
        db: Session
    ) -> Dict[str, str]:
        """Get terminology suggestions for translation."""
        try:
            query = db.query(TerminologyGlossary).filter(
                TerminologyGlossary.source_language == source_language,
                TerminologyGlossary.target_language == target_language,
                TerminologyGlossary.approved == True
            )
            
            if domain:
                query = query.filter(TerminologyGlossary.domain == domain)
            
            terms = query.all()
            
            terminology = {}
            for term in terms:
                if term.term.lower() in text.lower():
                    terminology[term.term] = term.translation
            
            return terminology
            
        except Exception as e:
            logger.error(f"Error getting terminology: {e}")
            return {}
    
    async def _enhance_translation_result(
        self,
        result: Dict[str, Any],
        original_text: str,
        source_language: str,
        target_language: str,
        provider: TranslationProvider
    ) -> Dict[str, Any]:
        """Enhance translation result with quality metrics."""
        try:
            enhanced = result.copy()
            
            # Calculate quality score if not provided
            if "quality_score" not in enhanced:
                enhanced["quality_score"] = await self._calculate_quality_score(
                    original_text, enhanced.get("translated_text", ""),
                    source_language, target_language
                )
            
            # Add metadata
            enhanced.update({
                "provider": provider.value,
                "timestamp": datetime.utcnow().isoformat(),
                "source_language": source_language,
                "target_language": target_language,
                "character_count": len(original_text),
                "word_count": len(original_text.split())
            })
            
            return enhanced
            
        except Exception as e:
            logger.error(f"Error enhancing translation result: {e}")
            return result
    
    async def _calculate_quality_score(
        self,
        source_text: str,
        translated_text: str,
        source_language: str,
        target_language: str
    ) -> float:
        """Calculate translation quality score."""
        try:
            # Basic quality checks
            score = 0.5  # Base score
            
            # Length similarity (translated text shouldn't be too different in length)
            length_ratio = len(translated_text) / max(len(source_text), 1)
            if 0.5 <= length_ratio <= 2.0:
                score += 0.2
            
            # Check for untranslated content (same as source)
            if translated_text.strip() != source_text.strip():
                score += 0.2
            
            # Check for proper RTL content if target is RTL language
            if target_language in ['ur', 'ar', 'fa', 'he']:
                rtl_chars = sum(1 for c in translated_text if '\u0590' <= c <= '\u06FF')
                if rtl_chars > 0:
                    score += 0.1
            
            return min(score, 1.0)
            
        except Exception as e:
            logger.error(f"Error calculating quality score: {e}")
            return 0.5
    
    async def _add_to_memory(
        self,
        source_text: str,
        target_text: str,
        source_language: str,
        target_language: str,
        quality_score: Optional[float],
        context: Optional[str],
        domain: Optional[str],
        db: Session
    ) -> None:
        """Add translation to memory."""
        try:
            memory = TranslationMemory(
                source_text=source_text,
                target_text=target_text,
                source_language=source_language,
                target_language=target_language,
                context=context,
                domain=domain,
                quality_score=quality_score
            )
            
            db.add(memory)
            db.commit()
            
        except Exception as e:
            logger.error(f"Error adding to memory: {e}")
    
    async def _check_rate_limit(self, provider: TranslationProvider) -> bool:
        """Check if provider is within rate limits."""
        # Simplified rate limiting - in production, use Redis or similar
        current_time = datetime.utcnow()
        provider_limits = self.rate_limits.get(provider.value, {})
        
        # Reset if it's a new minute
        if (current_time.minute != provider_limits.get("minute", -1)):
            self.rate_limits[provider.value] = {
                "minute": current_time.minute,
                "count": 0
            }
            return True
        
        # Check current count against limit
        max_requests = 100  # Requests per minute
        current_count = provider_limits.get("count", 0)
        
        return current_count < max_requests
    
    async def _update_rate_limit(self, provider: TranslationProvider) -> None:
        """Update rate limit counter."""
        if provider.value not in self.rate_limits:
            self.rate_limits[provider.value] = {"minute": datetime.utcnow().minute, "count": 0}
        
        self.rate_limits[provider.value]["count"] += 1
    
    def _get_fallback_provider(self, provider: TranslationProvider) -> Optional[TranslationProvider]:
        """Get fallback provider."""
        fallback_map = {
            TranslationProvider.GOOGLE: TranslationProvider.AZURE,
            TranslationProvider.AZURE: TranslationProvider.AWS,
            TranslationProvider.AWS: TranslationProvider.OPENAI,
            TranslationProvider.OPENAI: TranslationProvider.GOOGLE,
        }
        return fallback_map.get(provider)


class BaseTranslationProvider:
    """Base class for translation providers."""
    
    async def translate(
        self,
        text: str,
        source_language: str,
        target_language: str,
        context: Optional[str] = None,
        terminology: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """Translate text."""
        raise NotImplementedError
    
    async def detect_language(self, text: str) -> Dict[str, Any]:
        """Detect language."""
        raise NotImplementedError
    
    async def get_supported_languages(self) -> List[Dict[str, str]]:
        """Get supported languages."""
        raise NotImplementedError


class GoogleTranslateProvider(BaseTranslationProvider):
    """Google Translate provider."""
    
    def __init__(self):
        self.api_key = getattr(settings, 'GOOGLE_TRANSLATE_API_KEY', None)
        self.base_url = "https://translation.googleapis.com/language/translate/v2"
    
    async def translate(
        self,
        text: str,
        source_language: str,
        target_language: str,
        context: Optional[str] = None,
        terminology: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """Translate using Google Translate."""
        try:
            if not self.api_key:
                raise Exception("Google Translate API key not configured")
            
            # Apply terminology replacements
            processed_text = text
            term_map = {}
            if terminology:
                for term, translation in terminology.items():
                    placeholder = f"__TERM_{len(term_map)}__"
                    term_map[placeholder] = translation
                    processed_text = processed_text.replace(term, placeholder)
            
            async with aiohttp.ClientSession() as session:
                params = {
                    'key': self.api_key,
                    'q': processed_text,
                    'source': source_language,
                    'target': target_language,
                    'format': 'text'
                }
                
                async with session.post(self.base_url, params=params) as response:
                    if response.status == 200:
                        data = await response.json()
                        translated_text = data['data']['translations'][0]['translatedText']
                        
                        # Restore terminology
                        for placeholder, translation in term_map.items():
                            translated_text = translated_text.replace(placeholder, translation)
                        
                        return {
                            "translated_text": translated_text,
                            "confidence": 0.8,  # Google doesn't provide confidence
                            "detected_source_language": data['data']['translations'][0].get('detectedSourceLanguage')
                        }
                    else:
                        raise Exception(f"Google Translate API error: {response.status}")
            
        except Exception as e:
            logger.error(f"Google Translate error: {e}")
            return {
                "translated_text": text,
                "confidence": 0.0,
                "error": str(e)
            }


class AzureTranslateProvider(BaseTranslationProvider):
    """Azure Translator provider."""
    
    def __init__(self):
        self.api_key = getattr(settings, 'AZURE_TRANSLATE_API_KEY', None)
        self.region = getattr(settings, 'AZURE_TRANSLATE_REGION', 'global')
        self.base_url = "https://api.cognitive.microsofttranslator.com"
    
    async def translate(
        self,
        text: str,
        source_language: str,
        target_language: str,
        context: Optional[str] = None,
        terminology: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """Translate using Azure Translator."""
        try:
            if not self.api_key:
                raise Exception("Azure Translate API key not configured")
            
            headers = {
                'Ocp-Apim-Subscription-Key': self.api_key,
                'Ocp-Apim-Subscription-Region': self.region,
                'Content-Type': 'application/json'
            }
            
            body = [{'text': text}]
            params = {
                'api-version': '3.0',
                'from': source_language,
                'to': target_language
            }
            
            async with aiohttp.ClientSession() as session:
                url = f"{self.base_url}/translate"
                async with session.post(url, headers=headers, params=params, json=body) as response:
                    if response.status == 200:
                        data = await response.json()
                        translation = data[0]['translations'][0]
                        
                        return {
                            "translated_text": translation['text'],
                            "confidence": translation.get('confidence', 0.8)
                        }
                    else:
                        raise Exception(f"Azure Translate API error: {response.status}")
            
        except Exception as e:
            logger.error(f"Azure Translate error: {e}")
            return {
                "translated_text": text,
                "confidence": 0.0,
                "error": str(e)
            }


class AWSTranslateProvider(BaseTranslationProvider):
    """AWS Translate provider."""
    
    def __init__(self):
        self.access_key = getattr(settings, 'AWS_ACCESS_KEY_ID', None)
        self.secret_key = getattr(settings, 'AWS_SECRET_ACCESS_KEY', None)
        self.region = getattr(settings, 'AWS_REGION', 'us-east-1')
    
    async def translate(
        self,
        text: str,
        source_language: str,
        target_language: str,
        context: Optional[str] = None,
        terminology: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """Translate using AWS Translate."""
        try:
            # AWS Translate implementation would go here
            # For now, return a mock response
            return {
                "translated_text": f"[AWS Translation of: {text}]",
                "confidence": 0.7,
                "provider": "aws"
            }
            
        except Exception as e:
            logger.error(f"AWS Translate error: {e}")
            return {
                "translated_text": text,
                "confidence": 0.0,
                "error": str(e)
            }


class OpenAITranslateProvider(BaseTranslationProvider):
    """OpenAI GPT-based translation provider."""
    
    def __init__(self):
        self.api_key = getattr(settings, 'OPENAI_API_KEY', None)
        self.base_url = "https://api.openai.com/v1/chat/completions"
    
    async def translate(
        self,
        text: str,
        source_language: str,
        target_language: str,
        context: Optional[str] = None,
        terminology: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        """Translate using OpenAI GPT."""
        try:
            if not self.api_key:
                raise Exception("OpenAI API key not configured")
            
            # Build prompt with context and terminology
            prompt = f"Translate the following text from {source_language} to {target_language}:"
            
            if context:
                prompt += f"\nContext: {context}"
            
            if terminology:
                terms_list = ", ".join([f"{k}: {v}" for k, v in terminology.items()])
                prompt += f"\nUse these specific translations for technical terms: {terms_list}"
            
            prompt += f"\n\nText to translate: {text}\n\nTranslation:"
            
            headers = {
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json'
            }
            
            body = {
                "model": "gpt-3.5-turbo",
                "messages": [
                    {"role": "system", "content": "You are a professional translator. Provide accurate, natural translations while preserving the original meaning and tone."},
                    {"role": "user", "content": prompt}
                ],
                "max_tokens": len(text) * 2,  # Rough estimate
                "temperature": 0.3  # Lower temperature for more consistent translations
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.post(self.base_url, headers=headers, json=body) as response:
                    if response.status == 200:
                        data = await response.json()
                        translated_text = data['choices'][0]['message']['content'].strip()
                        
                        return {
                            "translated_text": translated_text,
                            "confidence": 0.9,  # GPT generally provides high quality
                            "model": data.get('model', 'gpt-3.5-turbo'),
                            "usage": data.get('usage', {})
                        }
                    else:
                        raise Exception(f"OpenAI API error: {response.status}")
            
        except Exception as e:
            logger.error(f"OpenAI Translate error: {e}")
            return {
                "translated_text": text,
                "confidence": 0.0,
                "error": str(e)
            }


# Global AI translation service instance
ai_translation_service = AITranslationService()