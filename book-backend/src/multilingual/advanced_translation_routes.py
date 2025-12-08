"""Advanced translation API routes."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .advanced_translation import (
    advanced_translation, TranslationDomain, ConfidenceLevel
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/advanced", tags=["advanced-translation"])


# Pydantic models
class AdvancedTranslationRequest(BaseModel):
    source_text: str = Field(..., description="Text to translate")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(..., description="Target language code")
    domain: Optional[str] = Field("technical", description="Translation domain")
    context: Optional[str] = Field(None, description="Additional context")
    user_preferences: Optional[Dict[str, Any]] = Field(None, description="User preferences")


class TranslationAlternativeResponse(BaseModel):
    text: str
    confidence: float
    source: str
    domain: str
    metadata: Dict[str, Any]


class ContextualTranslationResponse(BaseModel):
    primary_translation: str
    confidence: str
    alternatives: List[TranslationAlternativeResponse]
    domain: str
    context_factors: Dict[str, float]
    fallback_used: bool
    quality_score: float


@router.post("/translate", response_model=ContextualTranslationResponse)
async def advanced_translate(
    request: AdvancedTranslationRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Perform advanced context-aware translation."""
    try:
        # Validate domain
        try:
            domain = TranslationDomain(request.domain) if request.domain else TranslationDomain.TECHNICAL
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid domain: {request.domain}"
            )
        
        # Perform translation
        result = await advanced_translation.translate_with_context(
            source_text=request.source_text,
            source_language=request.source_language,
            target_language=request.target_language,
            domain=domain,
            context=request.context,
            user_preferences=request.user_preferences,
            db=db
        )
        
        # Convert alternatives to response format
        alternatives_response = [
            TranslationAlternativeResponse(
                text=alt.text,
                confidence=alt.confidence,
                source=alt.source,
                domain=alt.domain.value,
                metadata=alt.metadata
            )
            for alt in result.alternatives
        ]
        
        return ContextualTranslationResponse(
            primary_translation=result.primary_translation,
            confidence=result.confidence.value,
            alternatives=alternatives_response,
            domain=result.domain.value,
            context_factors=result.context_factors,
            fallback_used=result.fallback_used,
            quality_score=result.quality_score
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in advanced translation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Advanced translation failed"
        )


@router.get("/domains")
async def get_translation_domains():
    """Get available translation domains."""
    return {
        "domains": [
            {
                "value": domain.value,
                "label": domain.value.replace("_", " ").title(),
                "description": f"Optimized for {domain.value} content"
            }
            for domain in TranslationDomain
        ]
    }


@router.get("/confidence-levels")
async def get_confidence_levels():
    """Get confidence level definitions."""
    return {
        "levels": [
            {
                "value": level.value,
                "label": level.value.replace("_", " ").title(),
                "threshold": advanced_translation.confidence_thresholds[level],
                "description": f"Confidence level: {level.value}"
            }
            for level in ConfidenceLevel
        ]
    }