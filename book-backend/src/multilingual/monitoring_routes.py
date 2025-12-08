"""Monitoring and analytics API routes for multilingual system."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime, timedelta

from fastapi import APIRouter, Depends, HTTPException, status, Query, BackgroundTasks
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .monitoring_system import monitoring_system, MetricType, AlertSeverity

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/monitoring", tags=["multilingual-monitoring"])


# Pydantic models for request/response
class MetricRequest(BaseModel):
    name: str = Field(..., description="Metric name")
    value: float = Field(..., description="Metric value")
    metric_type: str = Field(..., description="Type of metric")
    unit: str = Field("", description="Unit of measurement")
    tags: Optional[Dict[str, str]] = Field(None, description="Metric tags")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata")


class UserEngagementRequest(BaseModel):
    action: str = Field(..., description="User action")
    language_code: str = Field(..., description="Language code")
    content_type: Optional[str] = Field(None, description="Content type")
    content_id: Optional[str] = Field(None, description="Content ID")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata")


class AlertResponse(BaseModel):
    id: str
    timestamp: str
    severity: str
    title: str
    description: str
    metric_name: str
    current_value: float
    threshold_value: float
    resolved: bool


class MetricsResponse(BaseModel):
    time_range: str
    start_time: str
    end_time: str
    metrics: Dict[str, Any]
    alerts: List[AlertResponse]
    summary: Dict[str, Any]


@router.post("/metrics")
async def record_metric(
    request: MetricRequest,
    background_tasks: BackgroundTasks,
    current_user: User = Depends(get_current_user)
):
    """Record a metric event."""
    try:
        # Validate metric type
        try:
            metric_type = MetricType(request.metric_type)
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid metric type: {request.metric_type}"
            )
        
        # Record metric in background
        background_tasks.add_task(
            monitoring_system.record_metric,
            name=request.name,
            value=request.value,
            metric_type=metric_type,
            unit=request.unit,
            tags=request.tags,
            metadata=request.metadata
        )
        
        return {
            "success": True,
            "message": "Metric recorded successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error recording metric: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to record metric"
        )


@router.post("/user-engagement")
async def track_user_engagement(
    request: UserEngagementRequest,
    background_tasks: BackgroundTasks,
    current_user: User = Depends(get_current_user)
):
    """Track user engagement with multilingual features."""
    try:
        # Track engagement in background
        background_tasks.add_task(
            monitoring_system.track_user_engagement,
            user_id=current_user.id,
            action=request.action,
            language_code=request.language_code,
            content_type=request.content_type,
            content_id=request.content_id,
            metadata=request.metadata
        )
        
        return {
            "success": True,
            "message": "User engagement tracked successfully"
        }
        
    except Exception as e:
        logger.error(f"Error tracking user engagement: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to track user engagement"
        )


@router.get("/dashboard", response_model=MetricsResponse)
async def get_metrics_dashboard(
    time_range: str = Query("1h", regex="^(5m|15m|1h|6h|24h|7d|30d)$"),
    metric_names: Optional[str] = Query(None, description="Comma-separated metric names"),
    current_user: User = Depends(get_current_user)
):
    """Get metrics dashboard data."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Parse metric names
        metric_list = None
        if metric_names:
            metric_list = [name.strip() for name in metric_names.split(',')]
        
        dashboard_data = await monitoring_system.get_metrics_dashboard(
            time_range=time_range,
            metric_names=metric_list
        )
        
        if "error" in dashboard_data:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=dashboard_data["error"]
            )
        
        return dashboard_data
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting metrics dashboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get metrics dashboard"
        )


@router.get("/alerts")
async def get_alerts(
    severity: Optional[str] = Query(None, regex="^(low|medium|high|critical)$"),
    resolved: Optional[bool] = Query(None),
    limit: int = Query(50, ge=1, le=1000),
    current_user: User = Depends(get_current_user)
):
    """Get system alerts."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Filter alerts
        alerts = list(monitoring_system.alerts_buffer)
        
        if severity:
            severity_enum = AlertSeverity(severity)
            alerts = [alert for alert in alerts if alert.severity == severity_enum]
        
        if resolved is not None:
            alerts = [alert for alert in alerts if alert.resolved == resolved]
        
        # Sort by timestamp (most recent first)
        alerts.sort(key=lambda x: x.timestamp, reverse=True)
        
        # Limit results
        alerts = alerts[:limit]
        
        # Convert to response format
        alert_responses = [
            AlertResponse(
                id=alert.id,
                timestamp=alert.timestamp.isoformat(),
                severity=alert.severity.value,
                title=alert.title,
                description=alert.description,
                metric_name=alert.metric_name,
                current_value=alert.current_value,
                threshold_value=alert.threshold_value,
                resolved=alert.resolved
            )
            for alert in alerts
        ]
        
        return {
            "alerts": alert_responses,
            "total_count": len(alert_responses),
            "filters": {
                "severity": severity,
                "resolved": resolved
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting alerts: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get alerts"
        )


@router.put("/alerts/{alert_id}/resolve")
async def resolve_alert(
    alert_id: str,
    current_user: User = Depends(get_current_user)
):
    """Resolve an alert."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Find and resolve alert
        for alert in monitoring_system.alerts_buffer:
            if alert.id == alert_id:
                alert.resolved = True
                alert.resolved_at = datetime.utcnow()
                
                return {
                    "success": True,
                    "message": "Alert resolved successfully",
                    "alert_id": alert_id
                }
        
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Alert not found"
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error resolving alert: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to resolve alert"
        )


@router.get("/analytics/report")
async def get_analytics_report(
    report_type: str = Query("daily", regex="^(daily|weekly|monthly)$"),
    date: Optional[str] = Query(None, description="Date in YYYY-MM-DD format"),
    current_user: User = Depends(get_current_user)
):
    """Get analytics report."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Parse date
        report_date = None
        if date:
            try:
                report_date = datetime.strptime(date, "%Y-%m-%d")
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Invalid date format. Use YYYY-MM-DD"
                )
        
        report = await monitoring_system.generate_analytics_report(
            report_type=report_type,
            date=report_date
        )
        
        if "error" in report:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=report["error"]
            )
        
        return report
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting analytics report: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get analytics report"
        )


@router.get("/health")
async def get_system_health():
    """Get system health status."""
    try:
        # Get recent metrics for health calculation
        dashboard_data = await monitoring_system.get_metrics_dashboard(
            time_range="15m"
        )
        
        health_status = {
            "status": dashboard_data.get("summary", {}).get("system_health", "unknown"),
            "timestamp": datetime.utcnow().isoformat(),
            "active_alerts": dashboard_data.get("summary", {}).get("active_alerts", 0),
            "critical_alerts": dashboard_data.get("summary", {}).get("critical_alerts", 0),
            "metrics_count": dashboard_data.get("summary", {}).get("total_metrics", 0)
        }
        
        # Add component health
        components = {
            "translation_service": "healthy",
            "cache_system": "healthy",
            "search_engine": "healthy",
            "monitoring_system": "healthy"
        }
        
        # Check component health based on metrics
        metrics = dashboard_data.get("metrics", {})
        
        if "translation_load_time" in metrics:
            avg_time = metrics["translation_load_time"].get("average", 0)
            if avg_time > 2.0:
                components["translation_service"] = "degraded"
            elif avg_time > 3.0:
                components["translation_service"] = "unhealthy"
        
        if "cache_hit_ratio" in metrics:
            hit_ratio = metrics["cache_hit_ratio"].get("average", 100)
            if hit_ratio < 50:
                components["cache_system"] = "degraded"
            elif hit_ratio < 30:
                components["cache_system"] = "unhealthy"
        
        if "search_response_time" in metrics:
            search_time = metrics["search_response_time"].get("average", 0)
            if search_time > 500:
                components["search_engine"] = "degraded"
            elif search_time > 1000:
                components["search_engine"] = "unhealthy"
        
        health_status["components"] = components
        
        return health_status
        
    except Exception as e:
        logger.error(f"Error getting system health: {e}")
        return {
            "status": "unknown",
            "timestamp": datetime.utcnow().isoformat(),
            "error": str(e)
        }


@router.get("/usage-statistics")
async def get_usage_statistics(
    current_user: User = Depends(get_current_user)
):
    """Get usage statistics for multilingual features."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Get current usage metrics
        usage_stats = {
            "current_metrics": monitoring_system.usage_metrics.copy(),
            "timestamp": datetime.utcnow().isoformat()
        }
        
        # Add calculated metrics
        total_interactions = sum(monitoring_system.usage_metrics.values())
        usage_stats["total_interactions"] = total_interactions
        
        # Add language distribution (from recent metrics)
        language_distribution = {}
        for metric_name, data_points in monitoring_system.metric_aggregations.items():
            if metric_name == "user_engagement":
                for point in data_points[-1000:]:  # Last 1000 interactions
                    lang = point.get("tags", {}).get("language_code", "unknown")
                    language_distribution[lang] = language_distribution.get(lang, 0) + 1
        
        usage_stats["language_distribution"] = language_distribution
        
        return usage_stats
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting usage statistics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get usage statistics"
        )


@router.post("/test-alert")
async def create_test_alert(
    severity: str = Query("medium", regex="^(low|medium|high|critical)$"),
    current_user: User = Depends(get_current_user)
):
    """Create a test alert (for testing purposes)."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        # Create test alert
        await monitoring_system._create_alert(
            metric_name="test_metric",
            severity=AlertSeverity(severity),
            current_value=100.0,
            threshold_value=50.0,
            tags={"test": "true", "created_by": current_user.username}
        )
        
        return {
            "success": True,
            "message": f"Test alert created with severity: {severity}"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating test alert: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create test alert"
        )