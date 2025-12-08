"""Deployment and rollout API routes."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query, BackgroundTasks
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .deployment_system import (
    deployment_system, RolloutStage, FeatureFlag, DeploymentStatus
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/deployment", tags=["multilingual-deployment"])


# Pydantic models for request/response
class DeploymentRequest(BaseModel):
    version: str = Field(..., description="Version to deploy")
    stage: str = Field("internal", description="Initial rollout stage")


class FeatureFlagRequest(BaseModel):
    feature: str = Field(..., description="Feature flag name")
    enabled: bool = Field(..., description="Enable or disable feature")
    percentage: Optional[float] = Field(None, description="Percentage of users")


@router.post("/start")
async def start_deployment(
    request: DeploymentRequest,
    background_tasks: BackgroundTasks,
    current_user: User = Depends(get_current_user)
):
    """Start a new deployment."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Validate stage
        try:
            stage = RolloutStage(request.stage)
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid rollout stage: {request.stage}"
            )
        
        # Start deployment
        deployment_id = await deployment_system.start_deployment(
            version=request.version,
            stage=stage
        )
        
        return {
            "success": True,
            "deployment_id": deployment_id,
            "version": request.version,
            "stage": stage.value,
            "message": "Deployment started successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error starting deployment: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to start deployment"
        )


@router.get("/status/{deployment_id}")
async def get_deployment_status(
    deployment_id: str,
    current_user: User = Depends(get_current_user)
):
    """Get status of a specific deployment."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        status_info = deployment_system.get_deployment_status(deployment_id)
        
        if not status_info:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Deployment not found"
            )
        
        return status_info
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting deployment status: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get deployment status"
        )


@router.get("/health/{deployment_id}")
async def check_deployment_health(
    deployment_id: str,
    current_user: User = Depends(get_current_user)
):
    """Check health of a deployment."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        health_status = await deployment_system.check_deployment_health(deployment_id)
        
        if "error" in health_status:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=health_status["error"]
            )
        
        return health_status
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error checking deployment health: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to check deployment health"
        )


@router.get("/list")
async def list_deployments(
    current_user: User = Depends(get_current_user)
):
    """List all deployments."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        deployments = deployment_system.get_all_deployments()
        
        return {
            "deployments": deployments,
            "total_count": len(deployments)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error listing deployments: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to list deployments"
        )


@router.post("/rollback/{deployment_id}")
async def rollback_deployment(
    deployment_id: str,
    reason: str = Query(..., description="Reason for rollback"),
    current_user: User = Depends(get_current_user)
):
    """Manually rollback a deployment."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Trigger rollback
        await deployment_system._trigger_rollback(deployment_id, reason)
        
        return {
            "success": True,
            "deployment_id": deployment_id,
            "message": f"Deployment rolled back: {reason}"
        }
        
    except Exception as e:
        logger.error(f"Error rolling back deployment: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to rollback deployment"
        )


@router.get("/feature-flags")
async def get_feature_flags(
    current_user: User = Depends(get_current_user)
):
    """Get current feature flag status."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        flags = {}
        for flag in FeatureFlag:
            flags[flag.value] = deployment_system.feature_flags.get(flag, False)
        
        return {
            "feature_flags": flags,
            "updated_at": datetime.utcnow().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error getting feature flags: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get feature flags"
        )


@router.post("/feature-flags")
async def update_feature_flag(
    request: FeatureFlagRequest,
    current_user: User = Depends(get_current_user)
):
    """Update a feature flag."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Validate feature flag
        try:
            feature = FeatureFlag(request.feature)
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid feature flag: {request.feature}"
            )
        
        # Update feature flag
        percentage = request.percentage if request.percentage is not None else 100.0
        
        if request.enabled:
            await deployment_system._enable_feature_flags([feature], percentage)
        else:
            deployment_system.feature_flags[feature] = False
            
            if deployment_system.redis_client:
                import json
                import asyncio
                key = f"feature_flag:{feature.value}"
                flag_config = {
                    "enabled": False,
                    "percentage": 0,
                    "updated_at": datetime.utcnow().isoformat()
                }
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: deployment_system.redis_client.set(key, json.dumps(flag_config))
                )
        
        return {
            "success": True,
            "feature": feature.value,
            "enabled": request.enabled,
            "percentage": percentage if request.enabled else 0,
            "message": f"Feature flag {feature.value} updated"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating feature flag: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update feature flag"
        )


@router.get("/feature-flags/user/{user_id}")
async def check_user_feature_flags(
    user_id: int,
    current_user: User = Depends(get_current_user)
):
    """Check which features are enabled for a specific user."""
    try:
        # Users can check their own flags, admins can check any user
        if user_id != current_user.id and not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to check this user's feature flags"
            )
        
        # Get user segments (this would come from user profile/database)
        user_segments = ["active_users"]  # Default segment
        
        # Check if user is in special segments
        if getattr(current_user, 'is_admin', False):
            user_segments.append("internal_team")
        
        # You could add more logic here to determine user segments
        # based on user properties, subscription, location, etc.
        
        enabled_features = {}
        for feature in FeatureFlag:
            enabled = await deployment_system.is_feature_enabled_for_user(
                user_id=user_id,
                feature=feature,
                user_segments=user_segments
            )
            enabled_features[feature.value] = enabled
        
        return {
            "user_id": user_id,
            "user_segments": user_segments,
            "enabled_features": enabled_features,
            "checked_at": datetime.utcnow().isoformat()
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error checking user feature flags: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to check user feature flags"
        )


@router.get("/rollout-config")
async def get_rollout_config(
    current_user: User = Depends(get_current_user)
):
    """Get rollout configuration for all stages."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        configs = {}
        for stage, config in deployment_system.rollout_configs.items():
            configs[stage.value] = {
                "percentage": config.percentage,
                "duration_hours": config.duration_hours,
                "success_criteria": config.success_criteria,
                "rollback_criteria": config.rollback_criteria,
                "user_segments": config.user_segments,
                "feature_flags": [flag.value for flag in config.feature_flags]
            }
        
        return {
            "rollout_configs": configs,
            "stages": [stage.value for stage in RolloutStage if stage != RolloutStage.ROLLBACK]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting rollout config: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get rollout config"
        )


@router.get("/metrics/adoption")
async def get_adoption_metrics(
    feature: Optional[str] = Query(None, description="Specific feature to check"),
    current_user: User = Depends(get_current_user)
):
    """Get feature adoption metrics."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # This would integrate with analytics system
        # For now, return mock data
        metrics = {
            "total_users": 10000,
            "multilingual_users": 2500,
            "adoption_rate": 25.0,
            "by_feature": {
                "language_switching": {
                    "users": 2200,
                    "adoption_rate": 22.0,
                    "daily_usage": 1500
                },
                "content_translation": {
                    "users": 1800,
                    "adoption_rate": 18.0,
                    "daily_usage": 1200
                },
                "rtl_layout": {
                    "users": 2100,
                    "adoption_rate": 21.0,
                    "daily_usage": 1400
                },
                "community_features": {
                    "users": 800,
                    "adoption_rate": 8.0,
                    "daily_usage": 400
                }
            },
            "by_language": {
                "ur": {
                    "users": 2000,
                    "percentage": 80.0
                },
                "ar": {
                    "users": 300,
                    "percentage": 12.0
                },
                "other": {
                    "users": 200,
                    "percentage": 8.0
                }
            },
            "generated_at": datetime.utcnow().isoformat()
        }
        
        if feature:
            feature_metrics = metrics["by_feature"].get(feature)
            if not feature_metrics:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail=f"Metrics not found for feature: {feature}"
                )
            return {
                "feature": feature,
                "metrics": feature_metrics,
                "generated_at": metrics["generated_at"]
            }
        
        return metrics
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting adoption metrics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get adoption metrics"
        )