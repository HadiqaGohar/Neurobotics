"""API routes for cultural localization system."""

import logging
from typing import Dict, Any, Optional

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel

from app.core.database import get_db
from app.core.security import get_current_user
from app.models.database import User
from .cultural_localization import (
    cultural_localization, CulturalContext, LocalizationLevel
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/cultural", tags=["cultural-localization"])


# Pydantic models
class ContentLocalizationRequest(BaseModel):
    content: str
    target_language: str
    context: CulturalContext = CulturalContext.GENERAL
    level: LocalizationLevel = LocalizationLevel.MODERATE
    preserve_technical: bool = True


class CulturalValidationRequest(BaseModel):
    content: str
    target_language: str
    context: CulturalContext = CulturalContext.GENERAL


@router.post("/localize", response_model=Dict[str, Any])
async def localize_content(
    request: ContentLocalizationRequest,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Localize content for cultural context."""
    try:
        result = await cultural_localization.localize_content(
            content=request.content,
            target_language=request.target_language,
            context=request.context,
            level=request.level,
            preserve_technical=request.preserve_technical
        )
        
        return result
        
    except Exception as e:
        logger.error(f"Error localizing content: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/guidelines/{target_language}", response_model=Dict[str, Any])
async def get_cultural_guidelines(
    target_language: str,
    context: CulturalContext = Query(CulturalContext.GENERAL),
    current_user: User = Depends(get_current_user())
):
    """Get cultural guidelines for content creation."""
    try:
        guidelines = await cultural_localization.get_cultural_guidelines(
            target_language=target_language,
            context=context
        )
        
        return guidelines
        
    except Exception as e:
        logger.error(f"Error getting cultural guidelines: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/validate", response_model=Dict[str, Any])
async def validate_cultural_appropriateness(
    request: CulturalValidationRequest,
    current_user: User = Depends(get_current_user())
):
    """Validate content for cultural appropriateness."""
    try:
        validation_result = await cultural_localization.validate_cultural_appropriateness(
            content=request.content,
            target_language=request.target_language,
            context=request.context
        )
        
        return validation_result
        
    except Exception as e:
        logger.error(f"Error validating cultural appropriateness: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/examples/{content_type}/{target_language}", response_model=Dict[str, Any])
async def get_localized_examples(
    content_type: str,
    target_language: str,
    context: CulturalContext = Query(CulturalContext.TECHNICAL),
    current_user: User = Depends(get_current_user())
):
    """Generate culturally appropriate examples."""
    try:
        examples = await cultural_localization.generate_localized_examples(
            content_type=content_type,
            target_language=target_language,
            context=context
        )
        
        return examples
        
    except Exception as e:
        logger.error(f"Error generating localized examples: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/contexts", response_model=List[Dict[str, str]])
async def get_cultural_contexts():
    """Get available cultural contexts."""
    try:
        contexts = [
            {
                "value": context.value,
                "label": context.value.replace("_", " ").title(),
                "description": _get_context_description(context)
            }
            for context in CulturalContext
        ]
        
        return contexts
        
    except Exception as e:
        logger.error(f"Error getting cultural contexts: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/levels", response_model=List[Dict[str, str]])
async def get_localization_levels():
    """Get available localization levels."""
    try:
        levels = [
            {
                "value": level.value,
                "label": level.value.title(),
                "description": _get_level_description(level)
            }
            for level in LocalizationLevel
        ]
        
        return levels
        
    except Exception as e:
        logger.error(f"Error getting localization levels: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


def _get_context_description(context: CulturalContext) -> str:
    """Get description for cultural context."""
    descriptions = {
        CulturalContext.RELIGIOUS: "Content with religious considerations",
        CulturalContext.SOCIAL: "Social and interpersonal content",
        CulturalContext.EDUCATIONAL: "Educational and academic content",
        CulturalContext.BUSINESS: "Business and professional content",
        CulturalContext.TECHNICAL: "Technical and programming content",
        CulturalContext.GENERAL: "General purpose content"
    }
    return descriptions.get(context, "No description available")


def _get_level_description(level: LocalizationLevel) -> str:
    """Get description for localization level."""
    descriptions = {
        LocalizationLevel.MINIMAL: "Basic translation with minimal cultural adaptation",
        LocalizationLevel.MODERATE: "Translation with moderate cultural adaptation",
        LocalizationLevel.EXTENSIVE: "Full cultural adaptation and localization"
    }
    return descriptions.get(level, "No description available")