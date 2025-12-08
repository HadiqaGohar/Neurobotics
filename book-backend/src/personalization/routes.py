"""
FastAPI routes for the personalization service.
"""

import logging
from typing import Dict, Any, Optional
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from .models import (
    UserPreferences, PersonalizationRequest, PersonalizationResponse,
    PreferenceUpdateRequest, PersonalizationStats, SoftwareBackground, HardwareBackground
)
from .service import personalization_service
from app.core.database import get_db
from app.core.security import get_current_user
from app.models.database import User

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/personalization", tags=["personalization"])


@router.post("/preferences", response_model=Dict[str, str])
async def create_user_preferences(
    software_background: SoftwareBackground,
    hardware_background: HardwareBackground,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Create or update user preferences during signup or profile update."""
    try:
        preferences = UserPreferences(
            user_id=str(current_user.id),
            software_background=software_background,
            hardware_background=hardware_background
        )
        
        success = await personalization_service.save_user_preferences(preferences, db)
        
        if success:
            return {"message": "Preferences saved successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to save preferences"
            )
            
    except Exception as e:
        logger.error(f"Error creating user preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/preferences/signup", response_model=Dict[str, str])
async def save_signup_preferences(
    user_id: int,
    software_background: Optional[SoftwareBackground] = None,
    hardware_background: Optional[HardwareBackground] = None,
    db: Session = Depends(get_db)
):
    """Save user preferences during signup process (no auth required)."""
    try:
        # Create default backgrounds if not provided
        if not software_background:
            software_background = SoftwareBackground()
        if not hardware_background:
            hardware_background = HardwareBackground()
            
        preferences = UserPreferences(
            user_id=str(user_id),
            software_background=software_background,
            hardware_background=hardware_background
        )
        
        success = await personalization_service.save_user_preferences(preferences, db)
        
        if success:
            return {"message": "Signup preferences saved successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to save signup preferences"
            )
            
    except Exception as e:
        logger.error(f"Error saving signup preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/preferences", response_model=UserPreferences)
async def get_user_preferences(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get current user's personalization preferences."""
    try:
        preferences = await personalization_service.get_user_preferences(str(current_user.id), db)
        
        if not preferences:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User preferences not found"
            )
        
        return preferences
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting user preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.put("/preferences", response_model=Dict[str, str])
async def update_user_preferences(
    update_request: PreferenceUpdateRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Update user's personalization preferences."""
    try:
        # Get existing preferences
        existing_preferences = await personalization_service.get_user_preferences(str(current_user.id), db)
        
        if not existing_preferences:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User preferences not found. Please create preferences first."
            )
        
        # Update only provided fields
        if update_request.software_background:
            existing_preferences.software_background = update_request.software_background
        if update_request.hardware_background:
            existing_preferences.hardware_background = update_request.hardware_background
        if update_request.content_complexity:
            existing_preferences.content_complexity = update_request.content_complexity
        if update_request.explanation_depth:
            existing_preferences.explanation_depth = update_request.explanation_depth
        if update_request.example_style:
            existing_preferences.example_style = update_request.example_style
        
        # Save updated preferences
        success = await personalization_service.save_user_preferences(existing_preferences, db)
        
        if success:
            return {"message": "Preferences updated successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to update preferences"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating user preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/personalize", response_model=PersonalizationResponse)
async def personalize_content(
    request: PersonalizationRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Personalize content based on user preferences."""
    try:
        # Ensure the request is for the current user
        if request.user_id != str(current_user.id):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Cannot personalize content for another user"
            )
        
        response = await personalization_service.personalize_content(request, db)
        return response
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error personalizing content: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/content/{chapter_id}", response_model=PersonalizationResponse)
async def get_personalized_chapter(
    chapter_id: str,
    section_id: Optional[str] = None,
    force_regenerate: bool = False,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get personalized content for a specific chapter."""
    try:
        request = PersonalizationRequest(
            user_id=str(current_user.id),
            chapter_id=chapter_id,
            section_id=section_id,
            force_regenerate=force_regenerate
        )
        
        response = await personalization_service.personalize_content(request, db)
        return response
        
    except Exception as e:
        logger.error(f"Error getting personalized chapter: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/stats", response_model=PersonalizationStats)
async def get_personalization_stats(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get personalization usage statistics (admin only for now)."""
    try:
        # TODO: Add admin role check
        stats = await personalization_service.get_personalization_stats(db)
        
        return PersonalizationStats(
            total_requests=stats["total_requests"],
            cache_hit_rate=stats["cache_hit_rate"],
            average_generation_time_ms=stats["average_generation_time_ms"],
            popular_preferences=stats["popular_preferences"],
            content_variants_count=stats["content_variants_count"]
        )
        
    except Exception as e:
        logger.error(f"Error getting personalization stats: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.delete("/preferences", response_model=Dict[str, str])
async def delete_user_preferences(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Delete user's personalization preferences."""
    try:
        # TODO: Implement preference deletion
        # For now, just return a placeholder response
        return {"message": "Preference deletion not yet implemented"}
        
    except Exception as e:
        logger.error(f"Error deleting user preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/health")
async def personalization_health_check():
    """Health check endpoint for personalization service."""
    return {
        "status": "healthy",
        "service": "personalization",
        "timestamp": "2025-12-07T00:00:00Z"
    }


# Additional utility endpoints

@router.get("/categories/software")
async def get_software_categories():
    """Get available software development categories."""
    from .models import SoftwareCategory
    return {
        "categories": [category.value for category in SoftwareCategory]
    }


@router.get("/categories/hardware")
async def get_hardware_categories():
    """Get available hardware development categories."""
    from .models import HardwareCategory
    return {
        "categories": [category.value for category in HardwareCategory]
    }


@router.get("/languages")
async def get_supported_languages():
    """Get list of supported programming languages."""
    return {
        "languages": [
            "python", "javascript", "typescript", "java", "c++", "c", "c#",
            "go", "rust", "swift", "kotlin", "php", "ruby", "scala",
            "r", "matlab", "sql", "html", "css", "bash", "powershell"
        ]
    }


@router.get("/platforms")
async def get_supported_platforms():
    """Get list of supported hardware platforms."""
    return {
        "platforms": [
            "arduino", "raspberry_pi", "esp32", "esp8266", "stm32",
            "pic", "arm", "fpga", "beaglebone", "nvidia_jetson",
            "intel_nuc", "teensy", "micro_bit"
        ]
    }


@router.post("/preferences/validate", response_model=Dict[str, Any])
async def validate_preferences(
    software_background: Optional[SoftwareBackground] = None,
    hardware_background: Optional[HardwareBackground] = None,
    content_complexity: Optional[str] = None,
    explanation_depth: Optional[str] = None,
    example_style: Optional[str] = None
):
    """Validate user preferences without saving them."""
    try:
        validation_results = {
            "valid": True,
            "errors": [],
            "warnings": [],
            "suggestions": []
        }
        
        # Validate software background
        if software_background:
            if not software_background.categories:
                validation_results["warnings"].append("No software categories selected - content will use general examples")
            
            if not software_background.preferred_languages:
                validation_results["suggestions"].append("Consider selecting preferred programming languages for better code examples")
        
        # Validate hardware background
        if hardware_background:
            if not hardware_background.categories:
                validation_results["warnings"].append("No hardware categories selected - hardware examples will be generic")
            
            if not hardware_background.platforms:
                validation_results["suggestions"].append("Consider selecting hardware platforms you're familiar with")
        
        # Validate content preferences
        valid_complexities = ["simple", "moderate", "detailed", "comprehensive"]
        if content_complexity and content_complexity not in valid_complexities:
            validation_results["valid"] = False
            validation_results["errors"].append(f"Invalid content complexity. Must be one of: {valid_complexities}")
        
        valid_depths = ["overview", "standard", "detailed"]
        if explanation_depth and explanation_depth not in valid_depths:
            validation_results["valid"] = False
            validation_results["errors"].append(f"Invalid explanation depth. Must be one of: {valid_depths}")
        
        valid_styles = ["basic", "practical", "advanced"]
        if example_style and example_style not in valid_styles:
            validation_results["valid"] = False
            validation_results["errors"].append(f"Invalid example style. Must be one of: {valid_styles}")
        
        return validation_results
        
    except Exception as e:
        logger.error(f"Error validating preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during validation"
        )


@router.get("/preferences/recommendations")
async def get_preference_recommendations(
    current_categories: Optional[str] = None,
    experience_level: Optional[str] = None
):
    """Get personalized recommendations for preference settings."""
    try:
        recommendations = {
            "software_categories": [],
            "hardware_categories": [],
            "languages": [],
            "platforms": [],
            "content_settings": {}
        }
        
        # Parse current categories if provided
        categories = current_categories.split(",") if current_categories else []
        
        # Recommend based on experience level
        if experience_level == "beginner":
            recommendations["software_categories"] = ["web_development", "embedded_systems"]
            recommendations["hardware_categories"] = ["embedded_systems", "iot_devices"]
            recommendations["languages"] = ["python", "javascript", "c"]
            recommendations["platforms"] = ["arduino", "raspberry_pi"]
            recommendations["content_settings"] = {
                "content_complexity": "simple",
                "explanation_depth": "detailed",
                "example_style": "basic"
            }
        elif experience_level == "intermediate":
            recommendations["software_categories"] = ["web_development", "mobile_development", "embedded_systems"]
            recommendations["hardware_categories"] = ["embedded_systems", "iot_devices", "robotics"]
            recommendations["languages"] = ["python", "javascript", "c++", "java"]
            recommendations["platforms"] = ["arduino", "raspberry_pi", "esp32"]
            recommendations["content_settings"] = {
                "content_complexity": "moderate",
                "explanation_depth": "standard",
                "example_style": "practical"
            }
        elif experience_level in ["advanced", "expert"]:
            recommendations["software_categories"] = ["web_development", "mobile_development", "embedded_systems", "machine_learning"]
            recommendations["hardware_categories"] = ["embedded_systems", "iot_devices", "robotics", "electronics"]
            recommendations["languages"] = ["python", "javascript", "c++", "rust", "go"]
            recommendations["platforms"] = ["arduino", "raspberry_pi", "esp32", "stm32", "fpga"]
            recommendations["content_settings"] = {
                "content_complexity": "detailed" if experience_level == "advanced" else "comprehensive",
                "explanation_depth": "standard",
                "example_style": "advanced"
            }
        
        return recommendations
        
    except Exception as e:
        logger.error(f"Error getting preference recommendations: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/preferences/bulk-import", response_model=Dict[str, str])
async def bulk_import_preferences(
    preferences_data: Dict[str, Any],
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Bulk import user preferences from external source or backup."""
    try:
        # Validate the imported data structure
        required_fields = ["software_background", "hardware_background"]
        for field in required_fields:
            if field not in preferences_data:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Missing required field: {field}"
                )
        
        # Create preference objects from imported data
        software_bg = SoftwareBackground(**preferences_data["software_background"])
        hardware_bg = HardwareBackground(**preferences_data["hardware_background"])
        
        preferences = UserPreferences(
            user_id=str(current_user.id),
            software_background=software_bg,
            hardware_background=hardware_bg,
            content_complexity=preferences_data.get("content_complexity", "moderate"),
            explanation_depth=preferences_data.get("explanation_depth", "standard"),
            example_style=preferences_data.get("example_style", "practical")
        )
        
        success = await personalization_service.save_user_preferences(preferences, db)
        
        if success:
            return {"message": "Preferences imported successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to import preferences"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error importing preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during import"
        )


@router.get("/preferences/export")
async def export_user_preferences(
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Export user preferences for backup or transfer."""
    try:
        preferences = await personalization_service.get_user_preferences(str(current_user.id), db)
        
        if not preferences:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User preferences not found"
            )
        
        # Convert to exportable format
        export_data = {
            "user_id": preferences.user_id,
            "software_background": preferences.software_background.dict(),
            "hardware_background": preferences.hardware_background.dict(),
            "content_complexity": preferences.content_complexity,
            "explanation_depth": preferences.explanation_depth,
            "example_style": preferences.example_style,
            "exported_at": datetime.utcnow().isoformat(),
            "version": "1.0"
        }
        
        return export_data
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error exporting preferences: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during export"
        )


@router.post("/content/adapt", response_model=Dict[str, Any])
async def adapt_content_preview(
    content: Dict[str, Any],
    content_type: Optional[str] = "text",
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Preview how content would be adapted based on user preferences."""
    try:
        from .content_adapter import ContentType
        
        # Get user preferences
        preferences = await personalization_service.get_user_preferences(str(current_user.id), db)
        
        if not preferences:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User preferences not found. Please set up your preferences first."
            )
        
        # Map string content type to enum
        content_type_enum = ContentType.TEXT
        if content_type:
            try:
                content_type_enum = ContentType(content_type.lower())
            except ValueError:
                content_type_enum = ContentType.TEXT
        
        # Adapt the content
        from .content_adapter import content_adaptation_engine
        adapted_content = content_adaptation_engine.adapt_content(
            original_content=content,
            preferences=preferences,
            content_type=content_type_enum
        )
        
        return {
            "original_content": content,
            "adapted_content": adapted_content,
            "adaptation_summary": {
                "strategies_applied": adapted_content.get("adaptation_metadata", {}).get("strategies_applied", []),
                "personalization_score": adapted_content.get("adaptation_metadata", {}).get("personalization_score", 0.0),
                "target_experience": adapted_content.get("adaptation_metadata", {}).get("target_experience", "intermediate")
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error adapting content preview: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during content adaptation"
        )


@router.get("/adaptation/rules")
async def get_adaptation_rules():
    """Get available content adaptation rules and strategies."""
    try:
        from .content_adapter import AdaptationStrategy, ContentType
        
        return {
            "adaptation_strategies": [strategy.value for strategy in AdaptationStrategy],
            "content_types": [content_type.value for content_type in ContentType],
            "experience_levels": ["beginner", "intermediate", "advanced", "expert"],
            "complexity_levels": ["simple", "moderate", "detailed", "comprehensive"],
            "explanation_depths": ["overview", "standard", "detailed"],
            "example_styles": ["basic", "practical", "advanced"]
        }
        
    except Exception as e:
        logger.error(f"Error getting adaptation rules: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


# Content Variant Management Routes

@router.post("/variants", response_model=Dict[str, Any])
async def create_content_variant(
    chapter_id: str,
    variant_type: str,
    target_audience: Dict[str, Any],
    content: Dict[str, Any],
    section_id: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None,
    is_ai_generated: bool = False,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Create a new content variant."""
    try:
        from .variant_manager import variant_manager
        
        variant = await variant_manager.create_variant(
            db=db,
            chapter_id=chapter_id,
            section_id=section_id,
            variant_type=variant_type,
            target_audience=target_audience,
            content=content,
            metadata=metadata,
            is_ai_generated=is_ai_generated
        )
        
        if variant:
            return {
                "message": "Content variant created successfully",
                "content_id": variant.content_id,
                "variant": {
                    "id": variant.id,
                    "content_id": variant.content_id,
                    "chapter_id": variant.chapter_id,
                    "section_id": variant.section_id,
                    "variant_type": variant.variant_type,
                    "quality_score": variant.quality_score,
                    "created_at": variant.created_at.isoformat()
                }
            }
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to create content variant"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating content variant: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/variants/{content_id}", response_model=Dict[str, Any])
async def get_content_variant(
    content_id: str,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get a specific content variant."""
    try:
        from .variant_manager import variant_manager
        
        variant = await variant_manager.get_variant(db, content_id)
        
        if not variant:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Content variant not found"
            )
        
        return {
            "id": variant.id,
            "content_id": variant.content_id,
            "chapter_id": variant.chapter_id,
            "section_id": variant.section_id,
            "variant_type": variant.variant_type,
            "target_audience": variant.target_audience,
            "content": variant.content,
            "metadata": variant.content_metadata,
            "is_ai_generated": variant.is_ai_generated,
            "quality_score": variant.quality_score,
            "usage_count": variant.usage_count,
            "created_at": variant.created_at.isoformat(),
            "updated_at": variant.updated_at.isoformat()
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting content variant: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/variants/chapter/{chapter_id}", response_model=Dict[str, Any])
async def get_chapter_variants(
    chapter_id: str,
    section_id: Optional[str] = None,
    variant_type: Optional[str] = None,
    active_only: bool = True,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get all variants for a chapter."""
    try:
        from .variant_manager import variant_manager
        
        variants = await variant_manager.get_variants_for_chapter(
            db=db,
            chapter_id=chapter_id,
            section_id=section_id,
            variant_type=variant_type,
            active_only=active_only
        )
        
        return {
            "chapter_id": chapter_id,
            "section_id": section_id,
            "total_variants": len(variants),
            "variants": [
                {
                    "id": v.id,
                    "content_id": v.content_id,
                    "variant_type": v.variant_type,
                    "target_audience": v.target_audience,
                    "quality_score": v.quality_score,
                    "usage_count": v.usage_count,
                    "is_ai_generated": v.is_ai_generated,
                    "created_at": v.created_at.isoformat()
                }
                for v in variants
            ]
        }
        
    except Exception as e:
        logger.error(f"Error getting chapter variants: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.put("/variants/{content_id}", response_model=Dict[str, str])
async def update_content_variant(
    content_id: str,
    updates: Dict[str, Any],
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Update a content variant."""
    try:
        from .variant_manager import variant_manager
        
        variant = await variant_manager.update_variant(db, content_id, updates)
        
        if variant:
            return {"message": "Content variant updated successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Content variant not found"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating content variant: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.delete("/variants/{content_id}", response_model=Dict[str, str])
async def delete_content_variant(
    content_id: str,
    soft_delete: bool = True,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Delete a content variant."""
    try:
        from .variant_manager import variant_manager
        
        success = await variant_manager.delete_variant(db, content_id, soft_delete)
        
        if success:
            return {"message": f"Content variant {'archived' if soft_delete else 'deleted'} successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Content variant not found"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting content variant: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/variants/analytics", response_model=Dict[str, Any])
async def get_variant_analytics(
    chapter_id: Optional[str] = None,
    days: int = 30,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Get analytics for content variants."""
    try:
        from .variant_manager import variant_manager
        
        analytics = await variant_manager.get_variant_analytics(db, chapter_id, days)
        
        return {
            "period_days": days,
            "chapter_id": chapter_id,
            "analytics": analytics
        }
        
    except Exception as e:
        logger.error(f"Error getting variant analytics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


# Personalization Rules Management Routes

@router.get("/rules", response_model=Dict[str, Any])
async def get_personalization_rules(
    current_user: User = Depends(get_current_user())
):
    """Get all personalization rules and their statistics."""
    try:
        from .rules_engine import rules_engine
        
        rules_info = []
        for rule_id, rule in rules_engine.rules.items():
            rules_info.append({
                "rule_id": rule.rule_id,
                "name": rule.name,
                "description": rule.description,
                "rule_type": rule.rule_type.value,
                "priority": rule.priority.value,
                "weight": rule.weight,
                "enabled": rule.enabled
            })
        
        stats = rules_engine.get_rule_stats()
        
        return {
            "rules": rules_info,
            "statistics": stats
        }
        
    except Exception as e:
        logger.error(f"Error getting personalization rules: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/rules/{rule_id}/enable", response_model=Dict[str, str])
async def enable_personalization_rule(
    rule_id: str,
    current_user: User = Depends(get_current_user())
):
    """Enable a personalization rule."""
    try:
        from .rules_engine import rules_engine
        
        success = rules_engine.enable_rule(rule_id)
        
        if success:
            return {"message": f"Rule {rule_id} enabled successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Rule not found"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error enabling rule {rule_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/rules/{rule_id}/disable", response_model=Dict[str, str])
async def disable_personalization_rule(
    rule_id: str,
    current_user: User = Depends(get_current_user())
):
    """Disable a personalization rule."""
    try:
        from .rules_engine import rules_engine
        
        success = rules_engine.disable_rule(rule_id)
        
        if success:
            return {"message": f"Rule {rule_id} disabled successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Rule not found"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error disabling rule {rule_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/rules/test", response_model=Dict[str, Any])
async def test_personalization_rules(
    content: Dict[str, Any],
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user())
):
    """Test personalization rules on sample content."""
    try:
        from .rules_engine import rules_engine
        
        # Get user preferences
        preferences = await personalization_service.get_user_preferences(str(current_user.id), db)
        
        if not preferences:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User preferences not found"
            )
        
        # Execute rules on content
        personalized_content, rule_results = rules_engine.execute_rules(
            content=content,
            user_preferences=preferences,
            context={"test_mode": True}
        )
        
        return {
            "original_content": content,
            "personalized_content": personalized_content,
            "rule_execution_results": [
                {
                    "rule_id": result.rule_id,
                    "executed": result.executed,
                    "score": result.score,
                    "execution_time_ms": result.execution_time_ms,
                    "modifications_count": len(result.modifications)
                }
                for result in rule_results
            ],
            "summary": {
                "total_rules_executed": len([r for r in rule_results if r.executed]),
                "total_score": sum(r.score for r in rule_results),
                "total_execution_time_ms": sum(r.execution_time_ms for r in rule_results)
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error testing personalization rules: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


# Cache Management Routes

@router.get("/cache/stats", response_model=Dict[str, Any])
async def get_cache_statistics(
    current_user: User = Depends(get_current_user())
):
    """Get cache performance statistics."""
    try:
        stats = await personalization_service.get_cache_stats()
        return stats
        
    except Exception as e:
        logger.error(f"Error getting cache statistics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/cache/warm", response_model=Dict[str, Any])
async def warm_cache(
    popular_chapters: List[str],
    common_preferences: List[Dict[str, Any]],
    current_user: User = Depends(get_current_user())
):
    """Warm cache with popular content combinations."""
    try:
        warmed_count = await personalization_service.warm_cache_for_popular_content(
            popular_chapters, common_preferences
        )
        
        return {
            "message": "Cache warming completed",
            "warmed_items": warmed_count,
            "chapters": popular_chapters,
            "preference_combinations": len(common_preferences)
        }
        
    except Exception as e:
        logger.error(f"Error warming cache: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/cache/cleanup", response_model=Dict[str, Any])
async def cleanup_cache(
    current_user: User = Depends(get_current_user())
):
    """Clean up expired cache entries."""
    try:
        cleaned_count = await personalization_service.cleanup_expired_cache()
        
        return {
            "message": "Cache cleanup completed",
            "cleaned_items": cleaned_count
        }
        
    except Exception as e:
        logger.error(f"Error cleaning up cache: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.delete("/cache/user/{user_id}", response_model=Dict[str, str])
async def invalidate_user_cache(
    user_id: str,
    current_user: User = Depends(get_current_user())
):
    """Invalidate cache for a specific user."""
    try:
        # Only allow users to invalidate their own cache or admin users
        if str(current_user.id) != user_id:
            # TODO: Add admin role check
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Cannot invalidate cache for another user"
            )
        
        personalization_service._invalidate_user_cache(user_id)
        
        return {"message": f"Cache invalidated for user {user_id}"}
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error invalidating user cache: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/performance/optimization", response_model=Dict[str, Any])
async def get_performance_optimization_suggestions(
    current_user: User = Depends(get_current_user())
):
    """Get performance optimization suggestions."""
    try:
        from .cache_manager import performance_optimizer
        
        optimization_data = performance_optimizer.optimize_cache_warming()
        optimization_stats = performance_optimizer.get_optimization_stats()
        
        return {
            "optimization_suggestions": optimization_data,
            "performance_stats": optimization_stats,
            "recommendations": [
                "Monitor cache hit rates and adjust TTL values accordingly",
                "Pre-warm cache with popular content combinations",
                "Implement cache invalidation strategies for content updates",
                "Consider Redis for distributed caching in production"
            ]
        }
        
    except Exception as e:
        logger.error(f"Error getting optimization suggestions: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )