"""Multilingual Search Engine with cross-language capabilities."""

import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple, Union
from datetime import datetime
from enum import Enum
import json
import re
from dataclasses import dataclass

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, text

from app.models.multilingual import (
    ContentTranslation, TranslationMemory, TerminologyGlossary, Language
)
from .ai_translation import ai_translation_service

logger = logging.getLogger(__name__)


class SearchType(str, Enum):
    """Search type options."""
    EXACT = "exact"
    FUZZY = "fuzzy"
    SEMANTIC = "semantic"
    CROSS_LANGUAGE = "cross_language"


class SearchScope(str, Enum):
    """Search scope options."""
    TITLE = "title"
    CONTENT = "content"
    ALL = "all"
    METADATA = "metadata"


@dataclass
class SearchResult:
    """Search result data structure."""
    id: str
    content_type: str
    content_id: str
    language: str
    title: str
    content_snippet: str
    relevance_score: float
    match_type: str
    highlights: List[str]
    metadata: Dict[str, Any]


@dataclass
class SearchQuery:
    """Search query data structure."""
    query: str
    language: str
    target_languages: List[str]
    search_type: SearchType
    search_scope: SearchScope
    filters: Dict[str, Any]
    limit: int
    offset: int


class MultilingualSearchEngine:
    """Advanced multilingual search engine."""
    
    def __init__(self):
        self.search_processors = {
            SearchType.EXACT: self._exact_search,
            SearchType.FUZZY: self._fuzzy_search,
            SearchType.SEMANTIC: self._semantic_search,
            SearchType.CROSS_LANGUAGE: self._cross_language_search
        }
        
        # Language-specific search configurations
        self.language_configs = {
            "ur": {
                "stemming": True,
                "stop_words": ["کا", "کی", "کے", "میں", "سے", "کو", "پر", "اور", "یا", "لیکن"],
                "synonyms": {
                    "پروگرام": ["کوڈ", "سافٹ ویئر", "ایپلیکیشن"],
                    "کمپیوٹر": ["حاسوب", "لیپ ٹاپ", "پی سی"],
                    "ڈیٹا": ["معلومات", "ڈیٹابیس", "فائل"]
                },
                "transliteration": True
            },
            "en": {
                "stemming": True,
                "stop_words": ["the", "a", "an", "and", "or", "but", "in", "on", "at", "to", "for"],
                "synonyms": {
                    "program": ["code", "software", "application"],
                    "computer": ["machine", "laptop", "pc"],
                    "data": ["information", "database", "file"]
                }
            }
        }
        
        # Search index cache
        self.search_cache = {}
        self.cache_ttl = 300  # 5 minutes
    
    async def search(
        self,
        query: SearchQuery,
        db: Session
    ) -> Dict[str, Any]:
        """Perform multilingual search."""
        try:
            # Validate query
            if not query.query.strip():
                return {
                    "results": [],
                    "total_count": 0,
                    "query_info": {
                        "original_query": query.query,
                        "error": "Empty query"
                    }
                }
            
            # Process query
            processed_query = await self._process_query(query, db)
            
            # Execute search based on type
            search_processor = self.search_processors.get(query.search_type)
            if not search_processor:
                raise ValueError(f"Unsupported search type: {query.search_type}")
            
            results = await search_processor(processed_query, db)
            
            # Post-process results
            processed_results = await self._post_process_results(
                results, processed_query, db
            )
            
            # Generate search analytics
            analytics = await self._generate_search_analytics(
                query, processed_results, db
            )
            
            return {
                "results": processed_results,
                "total_count": len(processed_results),
                "query_info": {
                    "original_query": query.query,
                    "processed_query": processed_query.query,
                    "search_type": query.search_type.value,
                    "languages_searched": query.target_languages,
                    "execution_time_ms": analytics.get("execution_time_ms", 0)
                },
                "analytics": analytics
            }
            
        except Exception as e:
            logger.error(f"Error in multilingual search: {e}")
            return {
                "results": [],
                "total_count": 0,
                "query_info": {
                    "original_query": query.query,
                    "error": str(e)
                }
            }
    
    async def _process_query(self, query: SearchQuery, db: Session) -> SearchQuery:
        """Process and enhance search query."""
        try:
            processed_query = query
            
            # Clean and normalize query
            cleaned_query = self._clean_query(query.query)
            
            # Apply language-specific processing
            if query.language in self.language_configs:
                config = self.language_configs[query.language]
                
                # Remove stop words
                if config.get("stop_words"):
                    cleaned_query = self._remove_stop_words(
                        cleaned_query, config["stop_words"]
                    )
                
                # Apply stemming (basic implementation)
                if config.get("stemming"):
                    cleaned_query = self._apply_stemming(cleaned_query, query.language)
                
                # Expand with synonyms
                if config.get("synonyms"):
                    cleaned_query = self._expand_synonyms(
                        cleaned_query, config["synonyms"]
                    )
            
            processed_query.query = cleaned_query
            return processed_query
            
        except Exception as e:
            logger.error(f"Error processing query: {e}")
            return query
    
    async def _exact_search(self, query: SearchQuery, db: Session) -> List[SearchResult]:
        """Perform exact text search."""
        try:
            results = []
            
            # Build base query
            base_query = db.query(ContentTranslation).filter(
                ContentTranslation.language_code.in_(query.target_languages)
            )
            
            # Apply scope filters
            if query.search_scope == SearchScope.TITLE:
                base_query = base_query.filter(
                    ContentTranslation.title.ilike(f"%{query.query}%")
                )
            elif query.search_scope == SearchScope.CONTENT:
                base_query = base_query.filter(
                    ContentTranslation.content.ilike(f"%{query.query}%")
                )
            else:  # ALL
                base_query = base_query.filter(
                    or_(
                        ContentTranslation.title.ilike(f"%{query.query}%"),
                        ContentTranslation.content.ilike(f"%{query.query}%")
                    )
                )
            
            # Apply additional filters
            if query.filters.get("content_type"):
                base_query = base_query.filter(
                    ContentTranslation.content_type == query.filters["content_type"]
                )
            
            if query.filters.get("status"):
                base_query = base_query.filter(
                    ContentTranslation.translation_status == query.filters["status"]
                )
            
            # Execute query with pagination
            translations = base_query.offset(query.offset).limit(query.limit).all()
            
            # Convert to search results
            for translation in translations:
                result = SearchResult(
                    id=str(translation.id),
                    content_type=translation.content_type,
                    content_id=translation.content_id,
                    language=translation.language_code,
                    title=translation.title or "Untitled",
                    content_snippet=self._generate_snippet(
                        translation.content or "", query.query
                    ),
                    relevance_score=self._calculate_relevance_score(
                        translation, query.query
                    ),
                    match_type="exact",
                    highlights=self._generate_highlights(
                        translation.title or "", translation.content or "", query.query
                    ),
                    metadata=translation.metadata or {}
                )
                results.append(result)
            
            # Sort by relevance
            results.sort(key=lambda x: x.relevance_score, reverse=True)
            
            return results
            
        except Exception as e:
            logger.error(f"Error in exact search: {e}")
            return []
    
    async def _fuzzy_search(self, query: SearchQuery, db: Session) -> List[SearchResult]:
        """Perform fuzzy text search."""
        try:
            # For fuzzy search, we'll use PostgreSQL's similarity functions
            # This is a simplified implementation
            
            results = []
            search_terms = query.query.split()
            
            for term in search_terms:
                # Use PostgreSQL similarity search
                similarity_query = text("""
                    SELECT id, content_type, content_id, language_code, title, content, metadata,
                           GREATEST(
                               SIMILARITY(title, :term),
                               SIMILARITY(content, :term)
                           ) as similarity_score
                    FROM content_translations 
                    WHERE language_code = ANY(:languages)
                    AND (
                        SIMILARITY(title, :term) > 0.3 
                        OR SIMILARITY(content, :term) > 0.2
                    )
                    ORDER BY similarity_score DESC
                    LIMIT :limit OFFSET :offset
                """)
                
                result_rows = db.execute(
                    similarity_query,
                    {
                        "term": term,
                        "languages": query.target_languages,
                        "limit": query.limit,
                        "offset": query.offset
                    }
                ).fetchall()
                
                for row in result_rows:
                    result = SearchResult(
                        id=str(row.id),
                        content_type=row.content_type,
                        content_id=row.content_id,
                        language=row.language_code,
                        title=row.title or "Untitled",
                        content_snippet=self._generate_snippet(
                            row.content or "", term
                        ),
                        relevance_score=float(row.similarity_score),
                        match_type="fuzzy",
                        highlights=self._generate_highlights(
                            row.title or "", row.content or "", term
                        ),
                        metadata=row.metadata or {}
                    )
                    results.append(result)
            
            # Remove duplicates and sort
            unique_results = {}
            for result in results:
                key = f"{result.content_id}_{result.language}"
                if key not in unique_results or result.relevance_score > unique_results[key].relevance_score:
                    unique_results[key] = result
            
            final_results = list(unique_results.values())
            final_results.sort(key=lambda x: x.relevance_score, reverse=True)
            
            return final_results[:query.limit]
            
        except Exception as e:
            logger.error(f"Error in fuzzy search: {e}")
            # Fallback to exact search
            return await self._exact_search(query, db)
    
    async def _semantic_search(self, query: SearchQuery, db: Session) -> List[SearchResult]:
        """Perform semantic search using embeddings."""
        try:
            # This would implement semantic search using embeddings
            # For now, we'll use enhanced keyword matching with synonyms
            
            results = []
            
            # Expand query with synonyms
            expanded_queries = [query.query]
            
            if query.language in self.language_configs:
                synonyms = self.language_configs[query.language].get("synonyms", {})
                for word in query.query.split():
                    if word.lower() in synonyms:
                        for synonym in synonyms[word.lower()]:
                            expanded_queries.append(
                                query.query.replace(word, synonym, 1)
                            )
            
            # Search with expanded queries
            for expanded_query in expanded_queries:
                expanded_search_query = SearchQuery(
                    query=expanded_query,
                    language=query.language,
                    target_languages=query.target_languages,
                    search_type=SearchType.EXACT,
                    search_scope=query.search_scope,
                    filters=query.filters,
                    limit=query.limit,
                    offset=query.offset
                )
                
                expanded_results = await self._exact_search(expanded_search_query, db)
                
                # Adjust relevance scores for semantic matches
                for result in expanded_results:
                    if result.content_snippet != query.query:
                        result.relevance_score *= 0.8  # Slightly lower score for semantic matches
                        result.match_type = "semantic"
                
                results.extend(expanded_results)
            
            # Remove duplicates and sort
            unique_results = {}
            for result in results:
                key = f"{result.content_id}_{result.language}"
                if key not in unique_results or result.relevance_score > unique_results[key].relevance_score:
                    unique_results[key] = result
            
            final_results = list(unique_results.values())
            final_results.sort(key=lambda x: x.relevance_score, reverse=True)
            
            return final_results[:query.limit]
            
        except Exception as e:
            logger.error(f"Error in semantic search: {e}")
            return await self._exact_search(query, db)
    
    async def _cross_language_search(self, query: SearchQuery, db: Session) -> List[SearchResult]:
        """Perform cross-language search with translation."""
        try:
            results = []
            
            # First, search in the original language
            original_results = await self._exact_search(query, db)
            results.extend(original_results)
            
            # Then translate query to other target languages and search
            for target_lang in query.target_languages:
                if target_lang == query.language:
                    continue
                
                try:
                    # Translate query to target language
                    translation_result = await ai_translation_service.translate_text(
                        text=query.query,
                        source_language=query.language,
                        target_language=target_lang,
                        db=db
                    )
                    
                    if translation_result.get("translated_text"):
                        translated_query = SearchQuery(
                            query=translation_result["translated_text"],
                            language=target_lang,
                            target_languages=[target_lang],
                            search_type=SearchType.EXACT,
                            search_scope=query.search_scope,
                            filters=query.filters,
                            limit=query.limit,
                            offset=query.offset
                        )
                        
                        translated_results = await self._exact_search(translated_query, db)
                        
                        # Mark as cross-language results
                        for result in translated_results:
                            result.match_type = "cross_language"
                            result.relevance_score *= 0.9  # Slightly lower score for translated matches
                        
                        results.extend(translated_results)
                
                except Exception as e:
                    logger.warning(f"Error translating query to {target_lang}: {e}")
                    continue
            
            # Also search in terminology glossary for technical terms
            terminology_results = await self._search_terminology(query, db)
            results.extend(terminology_results)
            
            # Remove duplicates and sort
            unique_results = {}
            for result in results:
                key = f"{result.content_id}_{result.language}"
                if key not in unique_results or result.relevance_score > unique_results[key].relevance_score:
                    unique_results[key] = result
            
            final_results = list(unique_results.values())
            final_results.sort(key=lambda x: x.relevance_score, reverse=True)
            
            return final_results[:query.limit]
            
        except Exception as e:
            logger.error(f"Error in cross-language search: {e}")
            return await self._exact_search(query, db)
    
    async def _search_terminology(self, query: SearchQuery, db: Session) -> List[SearchResult]:
        """Search in terminology glossary."""
        try:
            results = []
            
            # Search for terms in glossary
            glossary_entries = db.query(TerminologyGlossary).filter(
                or_(
                    TerminologyGlossary.term.ilike(f"%{query.query}%"),
                    TerminologyGlossary.translation.ilike(f"%{query.query}%")
                ),
                TerminologyGlossary.approved == True
            ).all()
            
            for entry in glossary_entries:
                result = SearchResult(
                    id=f"term_{entry.id}",
                    content_type="terminology",
                    content_id=str(entry.id),
                    language=entry.target_language,
                    title=f"{entry.term} → {entry.translation}",
                    content_snippet=entry.definition or f"Translation: {entry.translation}",
                    relevance_score=0.7,  # Fixed score for terminology
                    match_type="terminology",
                    highlights=[entry.term, entry.translation],
                    metadata={
                        "domain": entry.domain,
                        "source_language": entry.source_language,
                        "target_language": entry.target_language,
                        "context": entry.context
                    }
                )
                results.append(result)
            
            return results
            
        except Exception as e:
            logger.error(f"Error searching terminology: {e}")
            return []
    
    async def _post_process_results(
        self,
        results: List[SearchResult],
        query: SearchQuery,
        db: Session
    ) -> List[SearchResult]:
        """Post-process search results."""
        try:
            # Apply additional filtering
            filtered_results = []
            
            for result in results:
                # Skip low relevance results
                if result.relevance_score < 0.1:
                    continue
                
                # Apply quality filters
                if query.filters.get("min_quality"):
                    # This would check translation quality if available
                    pass
                
                filtered_results.append(result)
            
            # Enhance results with additional metadata
            for result in filtered_results:
                # Add language information
                language = db.query(Language).filter(
                    Language.code == result.language
                ).first()
                
                if language:
                    result.metadata.update({
                        "language_name": language.name,
                        "language_native_name": language.native_name,
                        "text_direction": language.direction.value
                    })
            
            return filtered_results
            
        except Exception as e:
            logger.error(f"Error post-processing results: {e}")
            return results
    
    def _clean_query(self, query: str) -> str:
        """Clean and normalize search query."""
        # Remove extra whitespace
        cleaned = re.sub(r'\s+', ' ', query.strip())
        
        # Remove special characters (keep basic punctuation)
        cleaned = re.sub(r'[^\w\s\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF.-]', '', cleaned)
        
        return cleaned
    
    def _remove_stop_words(self, query: str, stop_words: List[str]) -> str:
        """Remove stop words from query."""
        words = query.split()
        filtered_words = [word for word in words if word.lower() not in stop_words]
        return ' '.join(filtered_words)
    
    def _apply_stemming(self, query: str, language: str) -> str:
        """Apply basic stemming to query terms."""
        # This is a simplified stemming implementation
        # In production, you would use proper stemming libraries
        
        if language == "en":
            # Basic English stemming rules
            words = query.split()
            stemmed_words = []
            
            for word in words:
                if word.endswith('ing'):
                    stemmed_words.append(word[:-3])
                elif word.endswith('ed'):
                    stemmed_words.append(word[:-2])
                elif word.endswith('s') and len(word) > 3:
                    stemmed_words.append(word[:-1])
                else:
                    stemmed_words.append(word)
            
            return ' '.join(stemmed_words)
        
        return query
    
    def _expand_synonyms(self, query: str, synonyms: Dict[str, List[str]]) -> str:
        """Expand query with synonyms."""
        words = query.split()
        expanded_words = []
        
        for word in words:
            expanded_words.append(word)
            if word.lower() in synonyms:
                expanded_words.extend(synonyms[word.lower()])
        
        return ' '.join(expanded_words)
    
    def _generate_snippet(self, content: str, query: str, max_length: int = 200) -> str:
        """Generate content snippet with query context."""
        if not content:
            return ""
        
        # Find query position in content
        query_pos = content.lower().find(query.lower())
        
        if query_pos == -1:
            # Query not found, return beginning of content
            return content[:max_length] + ("..." if len(content) > max_length else "")
        
        # Calculate snippet boundaries
        start = max(0, query_pos - max_length // 2)
        end = min(len(content), start + max_length)
        
        snippet = content[start:end]
        
        # Add ellipsis if needed
        if start > 0:
            snippet = "..." + snippet
        if end < len(content):
            snippet = snippet + "..."
        
        return snippet
    
    def _calculate_relevance_score(self, translation: ContentTranslation, query: str) -> float:
        """Calculate relevance score for search result."""
        score = 0.0
        
        # Title match (higher weight)
        if translation.title and query.lower() in translation.title.lower():
            score += 0.5
        
        # Content match
        if translation.content and query.lower() in translation.content.lower():
            score += 0.3
        
        # Quality score bonus
        if translation.quality_score:
            score += translation.quality_score * 0.2
        
        # Recency bonus (newer content gets slight boost)
        if translation.updated_at:
            days_old = (datetime.utcnow() - translation.updated_at).days
            recency_bonus = max(0, (30 - days_old) / 30 * 0.1)
            score += recency_bonus
        
        return min(1.0, score)
    
    def _generate_highlights(self, title: str, content: str, query: str) -> List[str]:
        """Generate highlighted text snippets."""
        highlights = []
        
        # Find matches in title
        if title and query.lower() in title.lower():
            highlights.append(f"Title: ...{title}...")
        
        # Find matches in content
        if content:
            query_pos = content.lower().find(query.lower())
            if query_pos != -1:
                start = max(0, query_pos - 50)
                end = min(len(content), query_pos + len(query) + 50)
                highlight = content[start:end]
                highlights.append(f"Content: ...{highlight}...")
        
        return highlights
    
    async def _generate_search_analytics(
        self,
        query: SearchQuery,
        results: List[SearchResult],
        db: Session
    ) -> Dict[str, Any]:
        """Generate search analytics."""
        try:
            analytics = {
                "execution_time_ms": 0,  # Would be measured in actual implementation
                "total_results": len(results),
                "results_by_language": {},
                "results_by_type": {},
                "average_relevance": 0.0,
                "search_suggestions": []
            }
            
            # Count results by language
            for result in results:
                lang = result.language
                analytics["results_by_language"][lang] = analytics["results_by_language"].get(lang, 0) + 1
            
            # Count results by content type
            for result in results:
                content_type = result.content_type
                analytics["results_by_type"][content_type] = analytics["results_by_type"].get(content_type, 0) + 1
            
            # Calculate average relevance
            if results:
                analytics["average_relevance"] = sum(r.relevance_score for r in results) / len(results)
            
            # Generate search suggestions (basic implementation)
            if len(results) < 5:
                analytics["search_suggestions"] = [
                    "Try using different keywords",
                    "Check spelling and try again",
                    "Use broader search terms",
                    "Try searching in different languages"
                ]
            
            return analytics
            
        except Exception as e:
            logger.error(f"Error generating search analytics: {e}")
            return {}


# Global search engine instance
multilingual_search = MultilingualSearchEngine()