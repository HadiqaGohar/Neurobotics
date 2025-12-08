"""Language framework API routes."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .language_framework import language_framework, LanguageDirection, LanguageScript

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/languages", tags=["language-framework"])


# Pydantic models
class AddLanguageRequest(BaseModel):
    language_code: str = Field(..., description="ISO language code (2-3 letters)")
    language_name: str = Field(..., description="Language name in English")
    native_name: str = Field(..., description="Language name in native script")
    template: str = Field("ltr_latin_script", description="Configuration template to use")
    custom_config: Optional[Dict[str, Any]] = Field(None, description="Custom configuration overrides")


class LanguageConfigResponse(BaseModel):
    code: str
    name: str
    native_name: str
    direction: str
    script: str
    font_family: str
    fallback_fonts: List[str]
    enabled: bool


@router.get("/")
async def get_all_languages():
    """Get all language configurations."""
    try:
        languages = language_framework.get_all_languages()
        
        return {
            "languages": [
                {
                    "code": config.code,
                    "name": config.name,
                    "native_name": config.native_name,
                    "direction": config.direction.value,
                    "script": config.script.value,
                    "font_family": config.font_family,
                    "enabled": config.enabled
                }
                for config in languages.values()
            ],
            "total_count": len(languages)
        }
        
    except Exception as e:
        logger.error(f"Error getting languages: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get languages"
        )


@router.get("/enabled")
async def get_enabled_languages():
    """Get only enabled language configurations."""
    try:
        languages = language_framework.get_enabled_languages()
        
        return {
            "languages": [
                {
                    "code": config.code,
                    "name": config.name,
                    "native_name": config.native_name,
                    "direction": config.direction.value,
                    "script": config.script.value,
                    "font_family": config.font_family
                }
                for config in languages.values()
            ],
            "total_count": len(languages)
        }
        
    except Exception as e:
        logger.error(f"Error getting enabled languages: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get enabled languages"
        )


@router.get("/{language_code}")
async def get_language_config(language_code: str):
    """Get configuration for a specific language."""
    try:
        config = language_framework.get_language_config(language_code)
        
        if not config:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Language {language_code} not found"
            )
        
        return {
            "code": config.code,
            "name": config.name,
            "native_name": config.native_name,
            "direction": config.direction.value,
            "script": config.script.value,
            "font_family": config.font_family,
            "fallback_fonts": config.fallback_fonts,
            "pluralization_rules": config.pluralization_rules,
            "date_format": config.date_format,
            "number_format": config.number_format,
            "currency_format": config.currency_format,
            "cultural_adaptations": config.cultural_adaptations,
            "translation_services": config.translation_services,
            "quality_thresholds": config.quality_thresholds,
            "enabled": config.enabled
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting language config: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get language configuration"
        )


@router.post("/add")
async def add_new_language(
    request: AddLanguageRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Add a new language to the system."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Add the language
        result = await language_framework.add_new_language(
            language_code=request.language_code,
            language_name=request.language_name,
            native_name=request.native_name,
            template=request.template,
            custom_config=request.custom_config,
            db=db
        )
        
        return {
            "success": result.success,
            "language_code": result.language_code,
            "steps_completed": result.steps_completed,
            "steps_failed": result.steps_failed,
            "warnings": result.warnings,
            "next_steps": result.next_steps,
            "message": f"Language {request.language_code} {'added successfully' if result.success else 'setup completed with issues'}"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error adding new language: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to add new language"
        )


@router.put("/{language_code}/enable")
async def enable_language(
    language_code: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Enable a language."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        success = await language_framework.enable_language(language_code, db)
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Language {language_code} not found"
            )
        
        return {
            "success": True,
            "language_code": language_code,
            "message": f"Language {language_code} enabled successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error enabling language: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to enable language"
        )


@router.put("/{language_code}/disable")
async def disable_language(
    language_code: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Disable a language."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        success = await language_framework.disable_language(language_code, db)
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Language {language_code} not found"
            )
        
        return {
            "success": True,
            "language_code": language_code,
            "message": f"Language {language_code} disabled successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error disabling language: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to disable language"
        )


@router.get("/templates/")
async def get_language_templates():
    """Get available language configuration templates."""
    try:
        templates = language_framework.get_language_templates()
        
        return {
            "templates": templates,
            "total_count": len(templates)
        }
        
    except Exception as e:
        logger.error(f"Error getting language templates: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get language templates"
        )


@router.get("/{language_code}/validate")
async def validate_language_setup(
    language_code: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Validate that a language is properly set up."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        validation_result = await language_framework.validate_language_setup(
            language_code, db
        )
        
        return validation_result
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error validating language setup: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to validate language setup"
        )


@router.get("/scripts/")
async def get_supported_scripts():
    """Get supported writing scripts."""
    return {
        "scripts": [
            {
                "value": script.value,
                "name": script.value.title(),
                "description": f"{script.value.title()} script"
            }
            for script in LanguageScript
        ]
    }


@router.get("/directions/")
async def get_text_directions():
    """Get supported text directions."""
    return {
        "directions": [
            {
                "value": direction.value,
                "name": "Left-to-Right" if direction == LanguageDirection.LTR else "Right-to-Left",
                "description": f"Text flows from {'left to right' if direction == LanguageDirection.LTR else 'right to left'}"
            }
            for direction in LanguageDirection
        ]
    }