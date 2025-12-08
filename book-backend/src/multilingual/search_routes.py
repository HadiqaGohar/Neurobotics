"""API routes for multilingual search functionality."""

import logging
from typing import Dict, Any, List, Optional

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel

from app.core.database import get_db
from app.core.security import get_current_user
from app.models.database import User
from .search_engine import (
    multilingual_search, SearchQuery, SearchType, SearchScope
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/search", tags=["multilingual-search"])


# Pydantic models
class SearchRequest(BaseModel):
    query: str
    language: str = "en"
    target_languages: List[str] = ["en", "ur"]
    search_type: SearchType = SearchType.EXACT
    search_scope: SearchScope = SearchScope.ALL
    filters: Dict[str, Any] = {}
    limit: int = 20
    offset: int = 0


class SearchSuggestionRequest(BaseModel):
    partial_query: str
    language: str = "en"
    limit: int = 10


@router.post("/", response_model=Dict[str, Any])
async def search_content(
    request: SearchRequest,
    db: Session = Depends(get_db)
):
    """Search multilingual content."""
    try:
        # Validate request
        if not request.query.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Search query cannot be empty"
            )
        
        if request.limit > 100:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Limit cannot exceed 100"
            )
        
        # Create search query
        search_query = SearchQuery(
            query=request.query,
            language=request.language,
            target_languages=request.target_languages,
            search_type=request.search_type,
            search_scope=request.search_scope,
            filters=request.filters,
            limit=request.limit,
            offset=request.offset
        )
        
        # Execute search
        results = await multilingual_search.search(search_query, db)
        
        return results
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in content search: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/suggestions", response_model=List[str])
async def get_search_suggestions(
    q: str = Query(..., description="Partial search query"),
    language: str = Query("en", description="Search language"),
    limit: int = Query(10, le=20, description="Maximum number of suggestions"),
    db: Session = Depends(get_db)
):
    """Get search suggestions based on partial query."""
    try:
        if len(q.strip()) < 2:
            return []
        
        # This would implement autocomplete/suggestions
        # For now, return basic suggestions
        suggestions = []
        
        # Add common programming terms based on partial query
        programming_terms = {
            "en": [
                "programming", "python", "javascript", "database", "algorithm",
                "function", "variable", "class", "object", "array", "string",
                "loop", "condition", "method", "framework", "library"
            ],
            "ur": [
                "پروگرامنگ", "پائتھن", "جاوا اسکرپٹ", "ڈیٹابیس", "الگورتھم",
                "فنکشن", "متغیر", "کلاس", "آبجیکٹ", "صف", "سٹرنگ",
                "لوپ", "شرط", "میتھڈ", "فریم ورک", "لائبریری"
            ]
        }
        
        terms = programming_terms.get(language, programming_terms["en"])
        
        for term in terms:
            if q.lower() in term.lower():
                suggestions.append(term)
                if len(suggestions) >= limit:
                    break
        
        return suggestions
        
    except Exception as e:
        logger.error(f"Error getting search suggestions: {e}")
        return []


@router.get("/filters", response_model=Dict[str, Any])
async def get_search_filters(
    language: str = Query("en", description="Language for filter labels"),
    db: Session = Depends(get_db)
):
    """Get available search filters."""
    try:
        filters = {
            "content_types": [
                {"value": "chapter", "label": "Chapters" if language == "en" else "ابواب"},
                {"value": "section", "label": "Sections" if language == "en" else "حصے"},
                {"value": "example", "label": "Examples" if language == "en" else "مثالیں"},
                {"value": "exercise", "label": "Exercises" if language == "en" else "مشقیں"}
            ],
            "languages": [
                {"value": "en", "label": "English", "native_label": "English"},
                {"value": "ur", "label": "Urdu", "native_label": "اردو"}
            ],
            "search_types": [
                {
                    "value": SearchType.EXACT.value,
                    "label": "Exact Match" if language == "en" else "بالکل میچ",
                    "description": "Find exact matches" if language == "en" else "بالکل میچ تلاش کریں"
                },
                {
                    "value": SearchType.FUZZY.value,
                    "label": "Fuzzy Search" if language == "en" else "فزی سرچ",
                    "description": "Find similar matches" if language == "en" else "ملتے جلتے نتائج"
                },
                {
                    "value": SearchType.SEMANTIC.value,
                    "label": "Semantic Search" if language == "en" else "معنوی تلاش",
                    "description": "Find by meaning" if language == "en" else "معنی کے ذریعے تلاش"
                },
                {
                    "value": SearchType.CROSS_LANGUAGE.value,
                    "label": "Cross-Language" if language == "en" else "کراس لینگویج",
                    "description": "Search across languages" if language == "en" else "تمام زبانوں میں تلاش"
                }
            ],
            "search_scopes": [
                {
                    "value": SearchScope.ALL.value,
                    "label": "All Content" if language == "en" else "تمام مواد"
                },
                {
                    "value": SearchScope.TITLE.value,
                    "label": "Titles Only" if language == "en" else "صرف عنوانات"
                },
                {
                    "value": SearchScope.CONTENT.value,
                    "label": "Content Only" if language == "en" else "صرف مواد"
                }
            ]
        }
        
        return filters
        
    except Exception as e:
        logger.error(f"Error getting search filters: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/popular", response_model=List[Dict[str, Any]])
async def get_popular_searches(
    language: str = Query("en", description="Language for popular searches"),
    limit: int = Query(10, le=20, description="Number of popular searches"),
    db: Session = Depends(get_db)
):
    """Get popular search queries."""
    try:
        # This would typically come from search analytics
        # For now, return predefined popular searches
        
        popular_searches = {
            "en": [
                {"query": "python programming", "count": 150},
                {"query": "javascript functions", "count": 120},
                {"query": "database design", "count": 100},
                {"query": "algorithms", "count": 95},
                {"query": "web development", "count": 85},
                {"query": "data structures", "count": 80},
                {"query": "object oriented programming", "count": 75},
                {"query": "machine learning", "count": 70},
                {"query": "api development", "count": 65},
                {"query": "software testing", "count": 60}
            ],
            "ur": [
                {"query": "پائتھن پروگرامنگ", "count": 85},
                {"query": "جاوا اسکرپٹ فنکشنز", "count": 70},
                {"query": "ڈیٹابیس ڈیزائن", "count": 65},
                {"query": "الگورتھم", "count": 60},
                {"query": "ویب ڈیولپمنٹ", "count": 55},
                {"query": "ڈیٹا سٹرکچرز", "count": 50},
                {"query": "آبجیکٹ اورینٹڈ پروگرامنگ", "count": 45},
                {"query": "مشین لرننگ", "count": 40},
                {"query": "اے پی آئی ڈیولپمنٹ", "count": 35},
                {"query": "سافٹ ویئر ٹیسٹنگ", "count": 30}
            ]
        }
        
        searches = popular_searches.get(language, popular_searches["en"])
        return searches[:limit]
        
    except Exception as e:
        logger.error(f"Error getting popular searches: {e}")
        return []


@router.get("/analytics", response_model=Dict[str, Any])
async def get_search_analytics(
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Get search analytics (admin only)."""
    try:
        # This would implement comprehensive search analytics
        # For now, return mock data
        
        analytics = {
            "total_searches": 1250,
            "searches_by_language": {
                "en": 800,
                "ur": 450
            },
            "searches_by_type": {
                "exact": 600,
                "fuzzy": 300,
                "semantic": 200,
                "cross_language": 150
            },
            "top_queries": [
                {"query": "python programming", "count": 150, "language": "en"},
                {"query": "پائتھن پروگرامنگ", "count": 85, "language": "ur"},
                {"query": "javascript functions", "count": 120, "language": "en"}
            ],
            "search_success_rate": 0.85,
            "average_results_per_search": 12.5,
            "zero_result_queries": [
                {"query": "advanced quantum computing", "count": 5},
                {"query": "blockchain development", "count": 3}
            ]
        }
        
        return analytics
        
    except Exception as e:
        logger.error(f"Error getting search analytics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/feedback", response_model=Dict[str, str])
async def submit_search_feedback(
    query: str,
    result_id: str,
    feedback_type: str,  # "relevant", "not_relevant", "wrong_language"
    comment: Optional[str] = None,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Submit feedback on search results."""
    try:
        # This would store search feedback for improving results
        # For now, just log the feedback
        
        logger.info(f"Search feedback: query='{query}', result_id='{result_id}', "
                   f"feedback='{feedback_type}', user_id={current_user.id}")
        
        if comment:
            logger.info(f"Feedback comment: {comment}")
        
        return {"message": "Feedback submitted successfully"}
        
    except Exception as e:
        logger.error(f"Error submitting search feedback: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )