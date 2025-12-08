"""Production monitoring and analytics system for multilingual features."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
import uuid
from dataclasses import dataclass, asdict
from collections import defaultdict, deque
import statistics

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, desc
import redis
from fastapi import BackgroundTasks
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

from app.models.multilingual import ContentTranslation, TranslationMemory, Language
from app.models.database import User
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class AlertSeverity(str, Enum):
    """Alert severity levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class MetricType(str, Enum):
    """Types of metrics to track."""
    PERFORMANCE = "performance"
    USAGE = "usage"
    QUALITY = "quality"
    ERROR = "error"
    BUSINESS = "business"


@dataclass
class MetricEvent:
    """Represents a metric event."""
    id: str
    timestamp: datetime
    metric_type: MetricType
    name: str
    value: float
    unit: str
    tags: Dict[str, str]
    metadata: Dict[str, Any]


@dataclass
class Alert:
    """Represents a monitoring alert."""
    id: str
    timestamp: datetime
    severity: AlertSeverity
    title: str
    description: str
    metric_name: str
    current_value: float
    threshold_value: float
    tags: Dict[str, str]
    resolved: bool = False
    resolved_at: Optional[datetime] = None


class MultilingualMonitoringSystem:
    """Production monitoring and analytics system for multilingual features."""
    
    def __init__(self):
        self.redis_client = None
        self.metrics_buffer = deque(maxlen=10000)  # In-memory buffer for metrics
        self.alerts_buffer = deque(maxlen=1000)    # In-memory buffer for alerts
        self.alert_rules = {}
        self.metric_aggregations = defaultdict(list)
        
        # Performance thresholds
        self.performance_thresholds = {
            "translation_load_time": {"warning": 1.5, "critical": 2.0},
            "language_switch_time": {"warning": 0.8, "critical": 1.0},
            "search_response_time": {"warning": 400, "critical": 500},
            "font_load_time": {"warning": 0.8, "critical": 1.0},
            "cache_hit_ratio": {"warning": 70, "critical": 50},
            "error_rate": {"warning": 1, "critical": 5},
            "translation_quality": {"warning": 85, "critical": 80}
        }
        
        # Usage tracking
        self.usage_metrics = {
            "daily_active_users": 0,
            "language_switches": 0,
            "translations_viewed": 0,
            "community_contributions": 0,
            "search_queries": 0
        }
        
        self._init_redis()
        self._setup_alert_rules()
    
    def _init_redis(self):
        """Initialize Redis connection for metrics storage."""
        try:
            self.redis_client = redis.Redis(
                host=getattr(settings, 'REDIS_HOST', 'localhost'),
                port=getattr(settings, 'REDIS_PORT', 6379),
                db=getattr(settings, 'REDIS_METRICS_DB', 1),
                decode_responses=True
            )
            self.redis_client.ping()
            logger.info("Redis connection established for monitoring")
        except Exception as e:
            logger.warning(f"Redis connection failed for monitoring: {e}")
            self.redis_client = None
    
    def _setup_alert_rules(self):
        """Set up alert rules for different metrics."""
        self.alert_rules = {
            "translation_load_time": {
                "warning_threshold": 1.5,
                "critical_threshold": 2.0,
                "evaluation_window": 300,  # 5 minutes
                "min_samples": 10
            },
            "language_switch_time": {
                "warning_threshold": 0.8,
                "critical_threshold": 1.0,
                "evaluation_window": 300,
                "min_samples": 5
            },
            "search_response_time": {
                "warning_threshold": 400,
                "critical_threshold": 500,
                "evaluation_window": 300,
                "min_samples": 10
            },
            "error_rate": {
                "warning_threshold": 1.0,
                "critical_threshold": 5.0,
                "evaluation_window": 600,  # 10 minutes
                "min_samples": 1
            },
            "cache_hit_ratio": {
                "warning_threshold": 70,
                "critical_threshold": 50,
                "evaluation_window": 900,  # 15 minutes
                "min_samples": 100,
                "invert": True  # Lower values are worse
            },
            "translation_quality": {
                "warning_threshold": 85,
                "critical_threshold": 80,
                "evaluation_window": 3600,  # 1 hour
                "min_samples": 5,
                "invert": True
            }
        }
    
    async def record_metric(
        self,
        name: str,
        value: float,
        metric_type: MetricType,
        unit: str = "",
        tags: Optional[Dict[str, str]] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> None:
        """Record a metric event."""
        try:
            event = MetricEvent(
                id=str(uuid.uuid4()),
                timestamp=datetime.utcnow(),
                metric_type=metric_type,
                name=name,
                value=value,
                unit=unit,
                tags=tags or {},
                metadata=metadata or {}
            )
            
            # Add to buffer
            self.metrics_buffer.append(event)
            
            # Store in Redis if available
            if self.redis_client:
                await self._store_metric_in_redis(event)
            
            # Update aggregations
            self.metric_aggregations[name].append({
                "timestamp": event.timestamp,
                "value": value,
                "tags": tags or {}
            })
            
            # Keep only recent data in memory
            if len(self.metric_aggregations[name]) > 1000:
                self.metric_aggregations[name] = self.metric_aggregations[name][-1000:]
            
            # Check for alerts
            await self._check_alert_conditions(name, value, tags or {})
            
            logger.debug(f"Recorded metric: {name}={value} {unit}")
            
        except Exception as e:
            logger.error(f"Error recording metric {name}: {e}")
    
    async def _store_metric_in_redis(self, event: MetricEvent) -> None:
        """Store metric event in Redis."""
        try:
            # Store individual event
            key = f"metrics:{event.name}:{event.timestamp.strftime('%Y%m%d%H')}"
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.lpush(key, json.dumps(asdict(event), default=str))
            )
            
            # Set expiration (keep for 7 days)
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.expire(key, 604800)
            )
            
            # Update aggregated metrics
            await self._update_aggregated_metrics(event)
            
        except Exception as e:
            logger.error(f"Error storing metric in Redis: {e}")
    
    async def _update_aggregated_metrics(self, event: MetricEvent) -> None:
        """Update aggregated metrics in Redis."""
        try:
            # Daily aggregation
            date_key = event.timestamp.strftime('%Y%m%d')
            agg_key = f"metrics:daily:{event.name}:{date_key}"
            
            # Increment counter or update value based on metric type
            if event.metric_type == MetricType.USAGE:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.incr(agg_key)
                )
            else:
                # Store for later aggregation
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.lpush(f"{agg_key}:values", event.value)
                )
            
            # Set expiration
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.expire(agg_key, 2592000)  # 30 days
            )
            
        except Exception as e:
            logger.error(f"Error updating aggregated metrics: {e}")
    
    async def _check_alert_conditions(
        self,
        metric_name: str,
        current_value: float,
        tags: Dict[str, str]
    ) -> None:
        """Check if metric value triggers any alerts."""
        try:
            if metric_name not in self.alert_rules:
                return
            
            rule = self.alert_rules[metric_name]
            
            # Get recent values for evaluation
            recent_values = [
                item["value"] for item in self.metric_aggregations[metric_name]
                if (datetime.utcnow() - item["timestamp"]).total_seconds() <= rule["evaluation_window"]
            ]
            
            if len(recent_values) < rule["min_samples"]:
                return
            
            # Calculate average for evaluation
            avg_value = statistics.mean(recent_values)
            
            # Determine if alert should be triggered
            severity = None
            threshold_value = None
            
            invert = rule.get("invert", False)
            
            if invert:
                # Lower values are worse (e.g., cache hit ratio, quality score)
                if avg_value <= rule["critical_threshold"]:
                    severity = AlertSeverity.CRITICAL
                    threshold_value = rule["critical_threshold"]
                elif avg_value <= rule["warning_threshold"]:
                    severity = AlertSeverity.HIGH
                    threshold_value = rule["warning_threshold"]
            else:
                # Higher values are worse (e.g., response time, error rate)
                if avg_value >= rule["critical_threshold"]:
                    severity = AlertSeverity.CRITICAL
                    threshold_value = rule["critical_threshold"]
                elif avg_value >= rule["warning_threshold"]:
                    severity = AlertSeverity.HIGH
                    threshold_value = rule["warning_threshold"]
            
            if severity:
                await self._create_alert(
                    metric_name=metric_name,
                    severity=severity,
                    current_value=avg_value,
                    threshold_value=threshold_value,
                    tags=tags
                )
                
        except Exception as e:
            logger.error(f"Error checking alert conditions for {metric_name}: {e}")
    
    async def _create_alert(
        self,
        metric_name: str,
        severity: AlertSeverity,
        current_value: float,
        threshold_value: float,
        tags: Dict[str, str]
    ) -> None:
        """Create and send an alert."""
        try:
            # Check if similar alert already exists (avoid spam)
            recent_alerts = [
                alert for alert in self.alerts_buffer
                if (alert.metric_name == metric_name and 
                    not alert.resolved and
                    (datetime.utcnow() - alert.timestamp).total_seconds() < 3600)  # 1 hour
            ]
            
            if recent_alerts:
                logger.debug(f"Similar alert already exists for {metric_name}")
                return
            
            alert = Alert(
                id=str(uuid.uuid4()),
                timestamp=datetime.utcnow(),
                severity=severity,
                title=f"Multilingual System Alert: {metric_name}",
                description=self._generate_alert_description(
                    metric_name, current_value, threshold_value, severity
                ),
                metric_name=metric_name,
                current_value=current_value,
                threshold_value=threshold_value,
                tags=tags
            )
            
            # Add to buffer
            self.alerts_buffer.append(alert)
            
            # Store in Redis
            if self.redis_client:
                await self._store_alert_in_redis(alert)
            
            # Send notifications
            await self._send_alert_notifications(alert)
            
            logger.warning(f"Alert created: {alert.title} - {alert.description}")
            
        except Exception as e:
            logger.error(f"Error creating alert: {e}")
    
    def _generate_alert_description(
        self,
        metric_name: str,
        current_value: float,
        threshold_value: float,
        severity: AlertSeverity
    ) -> str:
        """Generate alert description."""
        descriptions = {
            "translation_load_time": f"Translation loading time is {current_value:.2f}s, exceeding {threshold_value}s threshold",
            "language_switch_time": f"Language switching time is {current_value:.2f}s, exceeding {threshold_value}s threshold",
            "search_response_time": f"Search response time is {current_value:.0f}ms, exceeding {threshold_value}ms threshold",
            "error_rate": f"Error rate is {current_value:.1f}%, exceeding {threshold_value}% threshold",
            "cache_hit_ratio": f"Cache hit ratio is {current_value:.1f}%, below {threshold_value}% threshold",
            "translation_quality": f"Translation quality score is {current_value:.1f}%, below {threshold_value}% threshold"
        }
        
        return descriptions.get(
            metric_name,
            f"{metric_name} value {current_value} exceeds threshold {threshold_value}"
        )
    
    async def _store_alert_in_redis(self, alert: Alert) -> None:
        """Store alert in Redis."""
        try:
            key = f"alerts:{alert.timestamp.strftime('%Y%m%d')}"
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.lpush(key, json.dumps(asdict(alert), default=str))
            )
            
            # Set expiration (keep for 30 days)
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.expire(key, 2592000)
            )
            
        except Exception as e:
            logger.error(f"Error storing alert in Redis: {e}")
    
    async def _send_alert_notifications(self, alert: Alert) -> None:
        """Send alert notifications via email/Slack/etc."""
        try:
            # Email notification
            if hasattr(settings, 'ALERT_EMAIL_RECIPIENTS'):
                await self._send_email_alert(alert)
            
            # Slack notification (if configured)
            if hasattr(settings, 'SLACK_WEBHOOK_URL'):
                await self._send_slack_alert(alert)
            
            # Log alert
            logger.error(f"ALERT [{alert.severity.upper()}]: {alert.title} - {alert.description}")
            
        except Exception as e:
            logger.error(f"Error sending alert notifications: {e}")
    
    async def _send_email_alert(self, alert: Alert) -> None:
        """Send email alert notification."""
        try:
            if not hasattr(settings, 'SMTP_SERVER'):
                return
            
            msg = MIMEMultipart()
            msg['From'] = getattr(settings, 'ALERT_EMAIL_FROM', 'alerts@example.com')
            msg['To'] = ', '.join(getattr(settings, 'ALERT_EMAIL_RECIPIENTS', []))
            msg['Subject'] = f"[{alert.severity.upper()}] {alert.title}"
            
            body = f"""
            Alert Details:
            - Severity: {alert.severity.upper()}
            - Metric: {alert.metric_name}
            - Current Value: {alert.current_value}
            - Threshold: {alert.threshold_value}
            - Time: {alert.timestamp.isoformat()}
            - Description: {alert.description}
            
            Tags: {json.dumps(alert.tags, indent=2)}
            
            Please investigate and resolve this issue.
            """
            
            msg.attach(MIMEText(body, 'plain'))
            
            # Send email
            server = smtplib.SMTP(settings.SMTP_SERVER, getattr(settings, 'SMTP_PORT', 587))
            server.starttls()
            if hasattr(settings, 'SMTP_USERNAME'):
                server.login(settings.SMTP_USERNAME, settings.SMTP_PASSWORD)
            
            server.send_message(msg)
            server.quit()
            
            logger.info(f"Email alert sent for {alert.metric_name}")
            
        except Exception as e:
            logger.error(f"Error sending email alert: {e}")
    
    async def _send_slack_alert(self, alert: Alert) -> None:
        """Send Slack alert notification."""
        try:
            import aiohttp
            
            webhook_url = getattr(settings, 'SLACK_WEBHOOK_URL', None)
            if not webhook_url:
                return
            
            color_map = {
                AlertSeverity.LOW: "#36a64f",
                AlertSeverity.MEDIUM: "#ff9500", 
                AlertSeverity.HIGH: "#ff6b35",
                AlertSeverity.CRITICAL: "#ff0000"
            }
            
            payload = {
                "attachments": [
                    {
                        "color": color_map.get(alert.severity, "#ff0000"),
                        "title": alert.title,
                        "text": alert.description,
                        "fields": [
                            {
                                "title": "Metric",
                                "value": alert.metric_name,
                                "short": True
                            },
                            {
                                "title": "Current Value",
                                "value": str(alert.current_value),
                                "short": True
                            },
                            {
                                "title": "Threshold",
                                "value": str(alert.threshold_value),
                                "short": True
                            },
                            {
                                "title": "Severity",
                                "value": alert.severity.upper(),
                                "short": True
                            }
                        ],
                        "ts": int(alert.timestamp.timestamp())
                    }
                ]
            }
            
            async with aiohttp.ClientSession() as session:
                async with session.post(webhook_url, json=payload) as response:
                    if response.status == 200:
                        logger.info(f"Slack alert sent for {alert.metric_name}")
                    else:
                        logger.error(f"Failed to send Slack alert: {response.status}")
                        
        except Exception as e:
            logger.error(f"Error sending Slack alert: {e}")
    
    async def get_metrics_dashboard(
        self,
        time_range: str = "1h",
        metric_names: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """Get metrics data for dashboard."""
        try:
            # Parse time range
            time_delta = self._parse_time_range(time_range)
            start_time = datetime.utcnow() - time_delta
            
            dashboard_data = {
                "time_range": time_range,
                "start_time": start_time.isoformat(),
                "end_time": datetime.utcnow().isoformat(),
                "metrics": {},
                "alerts": [],
                "summary": {}
            }
            
            # Get metrics data
            if metric_names is None:
                metric_names = list(self.metric_aggregations.keys())
            
            for metric_name in metric_names:
                metric_data = [
                    item for item in self.metric_aggregations[metric_name]
                    if item["timestamp"] >= start_time
                ]
                
                if metric_data:
                    values = [item["value"] for item in metric_data]
                    dashboard_data["metrics"][metric_name] = {
                        "current": values[-1] if values else 0,
                        "average": statistics.mean(values),
                        "min": min(values),
                        "max": max(values),
                        "count": len(values),
                        "data_points": metric_data[-100:]  # Last 100 points
                    }
            
            # Get recent alerts
            recent_alerts = [
                alert for alert in self.alerts_buffer
                if alert.timestamp >= start_time
            ]
            
            dashboard_data["alerts"] = [
                {
                    "id": alert.id,
                    "timestamp": alert.timestamp.isoformat(),
                    "severity": alert.severity.value,
                    "title": alert.title,
                    "description": alert.description,
                    "metric_name": alert.metric_name,
                    "resolved": alert.resolved
                }
                for alert in recent_alerts
            ]
            
            # Generate summary
            dashboard_data["summary"] = {
                "total_metrics": len(dashboard_data["metrics"]),
                "active_alerts": len([a for a in recent_alerts if not a.resolved]),
                "critical_alerts": len([a for a in recent_alerts if a.severity == AlertSeverity.CRITICAL and not a.resolved]),
                "system_health": self._calculate_system_health(dashboard_data["metrics"])
            }
            
            return dashboard_data
            
        except Exception as e:
            logger.error(f"Error getting metrics dashboard: {e}")
            return {"error": str(e)}
    
    def _parse_time_range(self, time_range: str) -> timedelta:
        """Parse time range string to timedelta."""
        time_map = {
            "5m": timedelta(minutes=5),
            "15m": timedelta(minutes=15),
            "1h": timedelta(hours=1),
            "6h": timedelta(hours=6),
            "24h": timedelta(hours=24),
            "7d": timedelta(days=7),
            "30d": timedelta(days=30)
        }
        
        return time_map.get(time_range, timedelta(hours=1))
    
    def _calculate_system_health(self, metrics: Dict[str, Any]) -> str:
        """Calculate overall system health score."""
        try:
            health_scores = []
            
            # Performance health
            if "translation_load_time" in metrics:
                avg_time = metrics["translation_load_time"]["average"]
                score = max(0, 100 - (avg_time - 1.0) * 50)  # 100% at 1s, 0% at 3s
                health_scores.append(score)
            
            # Cache health
            if "cache_hit_ratio" in metrics:
                ratio = metrics["cache_hit_ratio"]["average"]
                health_scores.append(ratio)
            
            # Error health
            if "error_rate" in metrics:
                error_rate = metrics["error_rate"]["average"]
                score = max(0, 100 - error_rate * 20)  # 100% at 0%, 0% at 5%
                health_scores.append(score)
            
            # Quality health
            if "translation_quality" in metrics:
                quality = metrics["translation_quality"]["average"]
                health_scores.append(quality)
            
            if not health_scores:
                return "unknown"
            
            overall_health = statistics.mean(health_scores)
            
            if overall_health >= 90:
                return "excellent"
            elif overall_health >= 75:
                return "good"
            elif overall_health >= 60:
                return "fair"
            elif overall_health >= 40:
                return "poor"
            else:
                return "critical"
                
        except Exception as e:
            logger.error(f"Error calculating system health: {e}")
            return "unknown"
    
    async def track_user_engagement(
        self,
        user_id: int,
        action: str,
        language_code: str,
        content_type: Optional[str] = None,
        content_id: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> None:
        """Track user engagement with multilingual features."""
        try:
            tags = {
                "user_id": str(user_id),
                "action": action,
                "language_code": language_code
            }
            
            if content_type:
                tags["content_type"] = content_type
            if content_id:
                tags["content_id"] = content_id
            
            await self.record_metric(
                name="user_engagement",
                value=1,
                metric_type=MetricType.USAGE,
                unit="count",
                tags=tags,
                metadata=metadata
            )
            
            # Update usage counters
            if action == "language_switch":
                self.usage_metrics["language_switches"] += 1
            elif action == "view_translation":
                self.usage_metrics["translations_viewed"] += 1
            elif action == "search":
                self.usage_metrics["search_queries"] += 1
            elif action == "contribute_translation":
                self.usage_metrics["community_contributions"] += 1
                
        except Exception as e:
            logger.error(f"Error tracking user engagement: {e}")
    
    async def generate_analytics_report(
        self,
        report_type: str = "daily",
        date: Optional[datetime] = None
    ) -> Dict[str, Any]:
        """Generate analytics report."""
        try:
            if date is None:
                date = datetime.utcnow()
            
            report = {
                "report_type": report_type,
                "date": date.strftime("%Y-%m-%d"),
                "generated_at": datetime.utcnow().isoformat(),
                "metrics": {},
                "insights": [],
                "recommendations": []
            }
            
            # Get metrics for the period
            if report_type == "daily":
                start_time = date.replace(hour=0, minute=0, second=0, microsecond=0)
                end_time = start_time + timedelta(days=1)
            elif report_type == "weekly":
                start_time = date - timedelta(days=date.weekday())
                end_time = start_time + timedelta(days=7)
            elif report_type == "monthly":
                start_time = date.replace(day=1, hour=0, minute=0, second=0, microsecond=0)
                if date.month == 12:
                    end_time = start_time.replace(year=date.year + 1, month=1)
                else:
                    end_time = start_time.replace(month=date.month + 1)
            
            # Aggregate metrics for the period
            for metric_name, data_points in self.metric_aggregations.items():
                period_data = [
                    item for item in data_points
                    if start_time <= item["timestamp"] < end_time
                ]
                
                if period_data:
                    values = [item["value"] for item in period_data]
                    report["metrics"][metric_name] = {
                        "total_samples": len(values),
                        "average": statistics.mean(values),
                        "median": statistics.median(values),
                        "min": min(values),
                        "max": max(values),
                        "std_dev": statistics.stdev(values) if len(values) > 1 else 0
                    }
            
            # Generate insights
            report["insights"] = self._generate_insights(report["metrics"])
            
            # Generate recommendations
            report["recommendations"] = self._generate_recommendations(report["metrics"])
            
            return report
            
        except Exception as e:
            logger.error(f"Error generating analytics report: {e}")
            return {"error": str(e)}
    
    def _generate_insights(self, metrics: Dict[str, Any]) -> List[str]:
        """Generate insights from metrics data."""
        insights = []
        
        try:
            # Performance insights
            if "translation_load_time" in metrics:
                avg_time = metrics["translation_load_time"]["average"]
                if avg_time < 1.0:
                    insights.append("Translation loading performance is excellent (< 1s average)")
                elif avg_time > 2.0:
                    insights.append("Translation loading performance needs improvement (> 2s average)")
            
            # Usage insights
            if "user_engagement" in metrics:
                engagement = metrics["user_engagement"]["total_samples"]
                if engagement > 1000:
                    insights.append(f"High user engagement with {engagement} multilingual interactions")
                elif engagement < 100:
                    insights.append("Low user engagement with multilingual features")
            
            # Quality insights
            if "translation_quality" in metrics:
                quality = metrics["translation_quality"]["average"]
                if quality > 90:
                    insights.append("Translation quality is excellent (>90% average)")
                elif quality < 80:
                    insights.append("Translation quality needs attention (<80% average)")
            
        except Exception as e:
            logger.error(f"Error generating insights: {e}")
        
        return insights
    
    def _generate_recommendations(self, metrics: Dict[str, Any]) -> List[str]:
        """Generate recommendations based on metrics."""
        recommendations = []
        
        try:
            # Performance recommendations
            if "translation_load_time" in metrics:
                avg_time = metrics["translation_load_time"]["average"]
                if avg_time > 1.5:
                    recommendations.append("Consider implementing more aggressive caching for translations")
                    recommendations.append("Optimize translation API response times")
            
            # Cache recommendations
            if "cache_hit_ratio" in metrics:
                hit_ratio = metrics["cache_hit_ratio"]["average"]
                if hit_ratio < 70:
                    recommendations.append("Increase cache TTL or preload more content")
                    recommendations.append("Review cache invalidation strategy")
            
            # Error rate recommendations
            if "error_rate" in metrics:
                error_rate = metrics["error_rate"]["average"]
                if error_rate > 2:
                    recommendations.append("Investigate and fix translation service errors")
                    recommendations.append("Implement better error handling and fallbacks")
            
        except Exception as e:
            logger.error(f"Error generating recommendations: {e}")
        
        return recommendations


# Global monitoring system instance
monitoring_system = MultilingualMonitoringSystem()