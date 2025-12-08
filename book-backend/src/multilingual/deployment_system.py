"""Production deployment and gradual rollout system for multilingual features."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict
import uuid
import hashlib

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func
import redis
from fastapi import BackgroundTasks

from app.models.database import User
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class RolloutStage(str, Enum):
    """Rollout stages for gradual deployment."""
    INTERNAL = "internal"
    BETA = "beta"
    GRADUAL = "gradual"
    FULL = "full"
    ROLLBACK = "rollback"


class FeatureFlag(str, Enum):
    """Feature flags for multilingual features."""
    LANGUAGE_SWITCHING = "language_switching"
    CONTENT_TRANSLATION = "content_translation"
    RTL_LAYOUT = "rtl_layout"
    COMMUNITY_FEATURES = "community_features"
    SEARCH_MULTILINGUAL = "search_multilingual"
    PERFORMANCE_OPTIMIZATION = "performance_optimization"
    MONITORING_ANALYTICS = "monitoring_analytics"


class DeploymentStatus(str, Enum):
    """Deployment status."""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    ROLLED_BACK = "rolled_back"


@dataclass
class RolloutConfig:
    """Configuration for gradual rollout."""
    stage: RolloutStage
    percentage: float  # Percentage of users to include
    duration_hours: int  # How long to stay in this stage
    success_criteria: Dict[str, float]  # Metrics that must be met
    rollback_criteria: Dict[str, float]  # Metrics that trigger rollback
    user_segments: List[str]  # User segments to target
    feature_flags: List[FeatureFlag]  # Features to enable


@dataclass
class DeploymentRecord:
    """Record of a deployment."""
    id: str
    version: str
    stage: RolloutStage
    status: DeploymentStatus
    started_at: datetime
    completed_at: Optional[datetime]
    config: RolloutConfig
    metrics: Dict[str, float]
    issues: List[str]
    rollback_reason: Optional[str]


class MultilingualDeploymentSystem:
    """Production deployment and rollout system."""
    
    def __init__(self):
        self.redis_client = None
        self.deployments = []
        self.feature_flags = {}
        self.user_segments = {}
        self.rollout_configs = self._initialize_rollout_configs()
        
        # Initialize Redis for feature flags
        self._init_redis()
        self._initialize_feature_flags()
    
    def _init_redis(self):
        """Initialize Redis connection for feature flags."""
        try:
            self.redis_client = redis.Redis(
                host=getattr(settings, 'REDIS_HOST', 'localhost'),
                port=getattr(settings, 'REDIS_PORT', 6379),
                db=getattr(settings, 'REDIS_FEATURE_FLAGS_DB', 2),
                decode_responses=True
            )
            self.redis_client.ping()
            logger.info("Redis connection established for feature flags")
        except Exception as e:
            logger.warning(f"Redis connection failed for feature flags: {e}")
            self.redis_client = None
    
    def _initialize_feature_flags(self):
        """Initialize feature flags with default values."""
        default_flags = {
            FeatureFlag.LANGUAGE_SWITCHING: False,
            FeatureFlag.CONTENT_TRANSLATION: False,
            FeatureFlag.RTL_LAYOUT: False,
            FeatureFlag.COMMUNITY_FEATURES: False,
            FeatureFlag.SEARCH_MULTILINGUAL: False,
            FeatureFlag.PERFORMANCE_OPTIMIZATION: False,
            FeatureFlag.MONITORING_ANALYTICS: False
        }
        
        for flag, default_value in default_flags.items():
            self.feature_flags[flag] = default_value
            if self.redis_client:
                try:
                    # Set flag in Redis if not exists
                    key = f"feature_flag:{flag.value}"
                    if not self.redis_client.exists(key):
                        self.redis_client.set(key, json.dumps(default_value))
                except Exception as e:
                    logger.error(f"Error setting feature flag {flag}: {e}")
    
    def _initialize_rollout_configs(self) -> Dict[RolloutStage, RolloutConfig]:
        """Initialize rollout configurations for each stage."""
        return {
            RolloutStage.INTERNAL: RolloutConfig(
                stage=RolloutStage.INTERNAL,
                percentage=0.1,  # 0.1% - internal team only
                duration_hours=24,
                success_criteria={
                    "error_rate": 1.0,  # < 1% error rate
                    "response_time": 2000,  # < 2s response time
                    "user_satisfaction": 4.0  # > 4.0 rating
                },
                rollback_criteria={
                    "error_rate": 5.0,  # > 5% error rate
                    "response_time": 5000,  # > 5s response time
                    "critical_errors": 1  # Any critical error
                },
                user_segments=["internal_team", "qa_team"],
                feature_flags=[
                    FeatureFlag.LANGUAGE_SWITCHING,
                    FeatureFlag.CONTENT_TRANSLATION,
                    FeatureFlag.RTL_LAYOUT
                ]
            ),
            
            RolloutStage.BETA: RolloutConfig(
                stage=RolloutStage.BETA,
                percentage=1.0,  # 1% - beta users
                duration_hours=72,
                success_criteria={
                    "error_rate": 2.0,
                    "response_time": 2500,
                    "user_satisfaction": 3.8,
                    "feature_adoption": 50.0  # > 50% of beta users try features
                },
                rollback_criteria={
                    "error_rate": 5.0,
                    "response_time": 5000,
                    "user_satisfaction": 2.0,
                    "critical_errors": 3
                },
                user_segments=["beta_testers", "urdu_speakers"],
                feature_flags=[
                    FeatureFlag.LANGUAGE_SWITCHING,
                    FeatureFlag.CONTENT_TRANSLATION,
                    FeatureFlag.RTL_LAYOUT,
                    FeatureFlag.COMMUNITY_FEATURES
                ]
            ),
            
            RolloutStage.GRADUAL: RolloutConfig(
                stage=RolloutStage.GRADUAL,
                percentage=10.0,  # 10% - gradual rollout
                duration_hours=168,  # 1 week
                success_criteria={
                    "error_rate": 2.5,
                    "response_time": 3000,
                    "user_satisfaction": 3.5,
                    "feature_adoption": 30.0,
                    "retention_rate": 85.0  # > 85% retention
                },
                rollback_criteria={
                    "error_rate": 5.0,
                    "response_time": 5000,
                    "user_satisfaction": 2.5,
                    "retention_rate": 70.0,  # < 70% retention
                    "critical_errors": 5
                },
                user_segments=["active_users", "urdu_speakers", "new_users"],
                feature_flags=[
                    FeatureFlag.LANGUAGE_SWITCHING,
                    FeatureFlag.CONTENT_TRANSLATION,
                    FeatureFlag.RTL_LAYOUT,
                    FeatureFlag.COMMUNITY_FEATURES,
                    FeatureFlag.SEARCH_MULTILINGUAL
                ]
            ),
            
            RolloutStage.FULL: RolloutConfig(
                stage=RolloutStage.FULL,
                percentage=100.0,  # 100% - full rollout
                duration_hours=0,  # Permanent
                success_criteria={
                    "error_rate": 3.0,
                    "response_time": 3000,
                    "user_satisfaction": 3.0,
                    "feature_adoption": 25.0
                },
                rollback_criteria={
                    "error_rate": 8.0,
                    "response_time": 8000,
                    "user_satisfaction": 2.0,
                    "critical_errors": 10
                },
                user_segments=["all_users"],
                feature_flags=list(FeatureFlag)
            )
        }
    
    async def start_deployment(
        self,
        version: str,
        stage: RolloutStage = RolloutStage.INTERNAL
    ) -> str:
        """Start a new deployment."""
        try:
            deployment_id = str(uuid.uuid4())
            config = self.rollout_configs[stage]
            
            deployment = DeploymentRecord(
                id=deployment_id,
                version=version,
                stage=stage,
                status=DeploymentStatus.IN_PROGRESS,
                started_at=datetime.utcnow(),
                completed_at=None,
                config=config,
                metrics={},
                issues=[],
                rollback_reason=None
            )
            
            self.deployments.append(deployment)
            
            # Enable feature flags for this stage
            await self._enable_feature_flags(config.feature_flags, config.percentage)
            
            # Set up user segments
            await self._configure_user_segments(config.user_segments, config.percentage)
            
            logger.info(f"Started deployment {deployment_id} for version {version} at stage {stage.value}")
            
            return deployment_id
            
        except Exception as e:
            logger.error(f"Error starting deployment: {e}")
            raise
    
    async def _enable_feature_flags(
        self,
        flags: List[FeatureFlag],
        percentage: float
    ) -> None:
        """Enable feature flags for specified percentage of users."""
        try:
            for flag in flags:
                flag_config = {
                    "enabled": True,
                    "percentage": percentage,
                    "updated_at": datetime.utcnow().isoformat()
                }
                
                self.feature_flags[flag] = True
                
                if self.redis_client:
                    key = f"feature_flag:{flag.value}"
                    await asyncio.get_event_loop().run_in_executor(
                        None,
                        lambda: self.redis_client.set(key, json.dumps(flag_config))
                    )
                
                logger.info(f"Enabled feature flag {flag.value} for {percentage}% of users")
                
        except Exception as e:
            logger.error(f"Error enabling feature flags: {e}")
    
    async def _configure_user_segments(
        self,
        segments: List[str],
        percentage: float
    ) -> None:
        """Configure user segments for rollout."""
        try:
            segment_config = {
                "segments": segments,
                "percentage": percentage,
                "updated_at": datetime.utcnow().isoformat()
            }
            
            if self.redis_client:
                key = "user_segments:multilingual_rollout"
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.set(key, json.dumps(segment_config))
                )
            
            logger.info(f"Configured user segments {segments} for {percentage}% rollout")
            
        except Exception as e:
            logger.error(f"Error configuring user segments: {e}")
    
    async def check_deployment_health(self, deployment_id: str) -> Dict[str, Any]:
        """Check health of a deployment."""
        try:
            deployment = next(
                (d for d in self.deployments if d.id == deployment_id),
                None
            )
            
            if not deployment:
                return {"error": "Deployment not found"}
            
            # Get current metrics (would integrate with monitoring system)
            current_metrics = await self._get_current_metrics()
            
            # Update deployment metrics
            deployment.metrics = current_metrics
            
            # Check success criteria
            success_met = self._check_success_criteria(
                current_metrics,
                deployment.config.success_criteria
            )
            
            # Check rollback criteria
            rollback_needed = self._check_rollback_criteria(
                current_metrics,
                deployment.config.rollback_criteria
            )
            
            health_status = {
                "deployment_id": deployment_id,
                "stage": deployment.stage.value,
                "status": deployment.status.value,
                "metrics": current_metrics,
                "success_criteria_met": success_met,
                "rollback_needed": rollback_needed,
                "duration_hours": (datetime.utcnow() - deployment.started_at).total_seconds() / 3600,
                "max_duration_hours": deployment.config.duration_hours
            }
            
            # Auto-advance or rollback if needed
            if rollback_needed:
                await self._trigger_rollback(deployment_id, "Rollback criteria met")
                health_status["action_taken"] = "rollback_triggered"
            elif success_met and self._should_advance_stage(deployment):
                next_stage = self._get_next_stage(deployment.stage)
                if next_stage:
                    await self._advance_to_next_stage(deployment_id, next_stage)
                    health_status["action_taken"] = f"advanced_to_{next_stage.value}"
            
            return health_status
            
        except Exception as e:
            logger.error(f"Error checking deployment health: {e}")
            return {"error": str(e)}
    
    async def _get_current_metrics(self) -> Dict[str, float]:
        """Get current system metrics."""
        # This would integrate with the monitoring system
        # For now, return mock metrics
        return {
            "error_rate": 0.5,  # 0.5%
            "response_time": 1500,  # 1.5s
            "user_satisfaction": 4.2,
            "feature_adoption": 65.0,
            "retention_rate": 88.0,
            "critical_errors": 0
        }
    
    def _check_success_criteria(
        self,
        metrics: Dict[str, float],
        criteria: Dict[str, float]
    ) -> bool:
        """Check if success criteria are met."""
        for metric, threshold in criteria.items():
            if metric not in metrics:
                continue
            
            current_value = metrics[metric]
            
            # Different metrics have different success conditions
            if metric in ["error_rate", "response_time", "critical_errors"]:
                # Lower is better
                if current_value > threshold:
                    return False
            else:
                # Higher is better
                if current_value < threshold:
                    return False
        
        return True
    
    def _check_rollback_criteria(
        self,
        metrics: Dict[str, float],
        criteria: Dict[str, float]
    ) -> bool:
        """Check if rollback criteria are met."""
        for metric, threshold in criteria.items():
            if metric not in metrics:
                continue
            
            current_value = metrics[metric]
            
            # Different metrics have different rollback conditions
            if metric in ["error_rate", "response_time", "critical_errors"]:
                # Higher values trigger rollback
                if current_value >= threshold:
                    return True
            else:
                # Lower values trigger rollback
                if current_value <= threshold:
                    return True
        
        return False
    
    def _should_advance_stage(self, deployment: DeploymentRecord) -> bool:
        """Check if deployment should advance to next stage."""
        # Check if minimum duration has passed
        duration = datetime.utcnow() - deployment.started_at
        min_duration = timedelta(hours=deployment.config.duration_hours)
        
        return duration >= min_duration
    
    def _get_next_stage(self, current_stage: RolloutStage) -> Optional[RolloutStage]:
        """Get the next rollout stage."""
        stage_order = [
            RolloutStage.INTERNAL,
            RolloutStage.BETA,
            RolloutStage.GRADUAL,
            RolloutStage.FULL
        ]
        
        try:
            current_index = stage_order.index(current_stage)
            if current_index < len(stage_order) - 1:
                return stage_order[current_index + 1]
        except ValueError:
            pass
        
        return None
    
    async def _advance_to_next_stage(
        self,
        deployment_id: str,
        next_stage: RolloutStage
    ) -> None:
        """Advance deployment to next stage."""
        try:
            deployment = next(
                (d for d in self.deployments if d.id == deployment_id),
                None
            )
            
            if not deployment:
                return
            
            # Update deployment record
            deployment.stage = next_stage
            deployment.config = self.rollout_configs[next_stage]
            
            # Enable feature flags for new stage
            await self._enable_feature_flags(
                deployment.config.feature_flags,
                deployment.config.percentage
            )
            
            # Update user segments
            await self._configure_user_segments(
                deployment.config.user_segments,
                deployment.config.percentage
            )
            
            logger.info(f"Advanced deployment {deployment_id} to stage {next_stage.value}")
            
        except Exception as e:
            logger.error(f"Error advancing deployment stage: {e}")
    
    async def _trigger_rollback(
        self,
        deployment_id: str,
        reason: str
    ) -> None:
        """Trigger rollback of deployment."""
        try:
            deployment = next(
                (d for d in self.deployments if d.id == deployment_id),
                None
            )
            
            if not deployment:
                return
            
            # Update deployment record
            deployment.status = DeploymentStatus.ROLLED_BACK
            deployment.rollback_reason = reason
            deployment.completed_at = datetime.utcnow()
            
            # Disable all feature flags
            await self._disable_all_feature_flags()
            
            # Reset user segments
            await self._reset_user_segments()
            
            logger.warning(f"Rolled back deployment {deployment_id}: {reason}")
            
        except Exception as e:
            logger.error(f"Error triggering rollback: {e}")
    
    async def _disable_all_feature_flags(self) -> None:
        """Disable all multilingual feature flags."""
        try:
            for flag in FeatureFlag:
                self.feature_flags[flag] = False
                
                if self.redis_client:
                    key = f"feature_flag:{flag.value}"
                    flag_config = {
                        "enabled": False,
                        "percentage": 0,
                        "updated_at": datetime.utcnow().isoformat()
                    }
                    await asyncio.get_event_loop().run_in_executor(
                        None,
                        lambda: self.redis_client.set(key, json.dumps(flag_config))
                    )
            
            logger.info("Disabled all multilingual feature flags")
            
        except Exception as e:
            logger.error(f"Error disabling feature flags: {e}")
    
    async def _reset_user_segments(self) -> None:
        """Reset user segments to default."""
        try:
            if self.redis_client:
                key = "user_segments:multilingual_rollout"
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.delete(key)
                )
            
            logger.info("Reset user segments")
            
        except Exception as e:
            logger.error(f"Error resetting user segments: {e}")
    
    async def is_feature_enabled_for_user(
        self,
        user_id: int,
        feature: FeatureFlag,
        user_segments: Optional[List[str]] = None
    ) -> bool:
        """Check if feature is enabled for specific user."""
        try:
            # Get feature flag configuration
            if self.redis_client:
                key = f"feature_flag:{feature.value}"
                flag_data = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.get(key)
                )
                
                if flag_data:
                    flag_config = json.loads(flag_data)
                    if not flag_config.get("enabled", False):
                        return False
                    
                    percentage = flag_config.get("percentage", 0)
                else:
                    return False
            else:
                # Fallback to in-memory flags
                if not self.feature_flags.get(feature, False):
                    return False
                percentage = 100.0  # Default to 100% if no Redis
            
            # Check user segments
            if user_segments:
                segment_key = "user_segments:multilingual_rollout"
                if self.redis_client:
                    segment_data = await asyncio.get_event_loop().run_in_executor(
                        None,
                        lambda: self.redis_client.get(segment_key)
                    )
                    
                    if segment_data:
                        segment_config = json.loads(segment_data)
                        allowed_segments = segment_config.get("segments", [])
                        
                        # Check if user belongs to allowed segments
                        if not any(segment in allowed_segments for segment in user_segments):
                            return False
            
            # Use consistent hash to determine if user is in rollout percentage
            user_hash = hashlib.md5(f"{user_id}:{feature.value}".encode()).hexdigest()
            hash_value = int(user_hash[:8], 16) % 10000  # 0-9999
            threshold = int(percentage * 100)  # Convert percentage to 0-10000 scale
            
            return hash_value < threshold
            
        except Exception as e:
            logger.error(f"Error checking feature flag for user {user_id}: {e}")
            return False
    
    def get_deployment_status(self, deployment_id: str) -> Optional[Dict[str, Any]]:
        """Get status of a deployment."""
        deployment = next(
            (d for d in self.deployments if d.id == deployment_id),
            None
        )
        
        if not deployment:
            return None
        
        return {
            "id": deployment.id,
            "version": deployment.version,
            "stage": deployment.stage.value,
            "status": deployment.status.value,
            "started_at": deployment.started_at.isoformat(),
            "completed_at": deployment.completed_at.isoformat() if deployment.completed_at else None,
            "percentage": deployment.config.percentage,
            "metrics": deployment.metrics,
            "issues": deployment.issues,
            "rollback_reason": deployment.rollback_reason
        }
    
    def get_all_deployments(self) -> List[Dict[str, Any]]:
        """Get all deployment records."""
        return [
            {
                "id": d.id,
                "version": d.version,
                "stage": d.stage.value,
                "status": d.status.value,
                "started_at": d.started_at.isoformat(),
                "completed_at": d.completed_at.isoformat() if d.completed_at else None,
                "percentage": d.config.percentage
            }
            for d in self.deployments
        ]


# Global deployment system instance
deployment_system = MultilingualDeploymentSystem()