"""
Monitoring and logging for the personalization service.
"""

import logging
import time
import json
from typing import Dict, Any, Optional
from datetime import datetime, timedelta
from functools import wraps
from contextlib import contextmanager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)


class PersonalizationMetrics:
    """Metrics collection for personalization service."""
    
    def __init__(self):
        self.metrics = {
            "requests_total": 0,
            "requests_successful": 0,
            "requests_failed": 0,
            "cache_hits": 0,
            "cache_misses": 0,
            "generation_times": [],
            "error_counts": {},
            "user_preferences_created": 0,
            "user_preferences_updated": 0,
            "content_variants_created": 0,
            "ai_generations": 0
        }
        self.start_time = time.time()
    
    def increment_counter(self, metric_name: str, value: int = 1):
        """Increment a counter metric."""
        if metric_name in self.metrics:
            self.metrics[metric_name] += value
    
    def record_generation_time(self, time_ms: int):
        """Record content generation time."""
        self.metrics["generation_times"].append(time_ms)
        # Keep only last 1000 measurements
        if len(self.metrics["generation_times"]) > 1000:
            self.metrics["generation_times"] = self.metrics["generation_times"][-1000:]
    
    def record_error(self, error_type: str):
        """Record an error occurrence."""
        if error_type not in self.metrics["error_counts"]:
            self.metrics["error_counts"][error_type] = 0
        self.metrics["error_counts"][error_type] += 1
    
    def get_metrics_summary(self) -> Dict[str, Any]:
        """Get summary of all metrics."""
        uptime_seconds = time.time() - self.start_time
        
        avg_generation_time = 0
        if self.metrics["generation_times"]:
            avg_generation_time = sum(self.metrics["generation_times"]) / len(self.metrics["generation_times"])
        
        cache_hit_rate = 0
        total_cache_requests = self.metrics["cache_hits"] + self.metrics["cache_misses"]
        if total_cache_requests > 0:
            cache_hit_rate = self.metrics["cache_hits"] / total_cache_requests
        
        success_rate = 0
        if self.metrics["requests_total"] > 0:
            success_rate = self.metrics["requests_successful"] / self.metrics["requests_total"]
        
        return {
            "uptime_seconds": uptime_seconds,
            "requests": {
                "total": self.metrics["requests_total"],
                "successful": self.metrics["requests_successful"],
                "failed": self.metrics["requests_failed"],
                "success_rate": success_rate
            },
            "cache": {
                "hits": self.metrics["cache_hits"],
                "misses": self.metrics["cache_misses"],
                "hit_rate": cache_hit_rate
            },
            "performance": {
                "avg_generation_time_ms": avg_generation_time,
                "total_generations": len(self.metrics["generation_times"])
            },
            "content": {
                "preferences_created": self.metrics["user_preferences_created"],
                "preferences_updated": self.metrics["user_preferences_updated"],
                "variants_created": self.metrics["content_variants_created"],
                "ai_generations": self.metrics["ai_generations"]
            },
            "errors": self.metrics["error_counts"]
        }


# Global metrics instance
metrics = PersonalizationMetrics()


def monitor_performance(operation_name: str):
    """Decorator to monitor function performance."""
    def decorator(func):
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                metrics.increment_counter("requests_total")
                result = await func(*args, **kwargs)
                metrics.increment_counter("requests_successful")
                
                # Record timing
                execution_time_ms = int((time.time() - start_time) * 1000)
                metrics.record_generation_time(execution_time_ms)
                
                logger.info(f"{operation_name} completed successfully in {execution_time_ms}ms")
                return result
                
            except Exception as e:
                metrics.increment_counter("requests_failed")
                metrics.record_error(type(e).__name__)
                logger.error(f"{operation_name} failed: {e}")
                raise
        
        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                metrics.increment_counter("requests_total")
                result = func(*args, **kwargs)
                metrics.increment_counter("requests_successful")
                
                # Record timing
                execution_time_ms = int((time.time() - start_time) * 1000)
                metrics.record_generation_time(execution_time_ms)
                
                logger.info(f"{operation_name} completed successfully in {execution_time_ms}ms")
                return result
                
            except Exception as e:
                metrics.increment_counter("requests_failed")
                metrics.record_error(type(e).__name__)
                logger.error(f"{operation_name} failed: {e}")
                raise
        
        # Return appropriate wrapper based on function type
        if hasattr(func, '__code__') and func.__code__.co_flags & 0x80:  # CO_COROUTINE
            return async_wrapper
        else:
            return sync_wrapper
    
    return decorator


@contextmanager
def log_operation(operation_name: str, **context):
    """Context manager for logging operations with timing."""
    start_time = time.time()
    logger.info(f"Starting {operation_name}", extra=context)
    
    try:
        yield
        execution_time = time.time() - start_time
        logger.info(f"Completed {operation_name} in {execution_time:.3f}s", extra=context)
        
    except Exception as e:
        execution_time = time.time() - start_time
        logger.error(f"Failed {operation_name} after {execution_time:.3f}s: {e}", extra=context)
        raise


class PersonalizationLogger:
    """Structured logging for personalization events."""
    
    def __init__(self):
        self.logger = logging.getLogger("personalization")
    
    def log_preference_created(self, user_id: str, preferences: Dict[str, Any]):
        """Log user preference creation."""
        self.logger.info(
            "User preferences created",
            extra={
                "event_type": "preference_created",
                "user_id": user_id,
                "software_categories": preferences.get("software_background", {}).get("categories", []),
                "hardware_categories": preferences.get("hardware_background", {}).get("categories", []),
                "experience_level": preferences.get("software_background", {}).get("experience_level", "unknown")
            }
        )
        metrics.increment_counter("user_preferences_created")
    
    def log_preference_updated(self, user_id: str, updated_fields: List[str]):
        """Log user preference update."""
        self.logger.info(
            "User preferences updated",
            extra={
                "event_type": "preference_updated",
                "user_id": user_id,
                "updated_fields": updated_fields
            }
        )
        metrics.increment_counter("user_preferences_updated")
    
    def log_content_personalized(self, user_id: str, chapter_id: str, personalization_type: str, cache_hit: bool, generation_time_ms: int):
        """Log content personalization event."""
        self.logger.info(
            "Content personalized",
            extra={
                "event_type": "content_personalized",
                "user_id": user_id,
                "chapter_id": chapter_id,
                "personalization_type": personalization_type,
                "cache_hit": cache_hit,
                "generation_time_ms": generation_time_ms
            }
        )
        
        if cache_hit:
            metrics.increment_counter("cache_hits")
        else:
            metrics.increment_counter("cache_misses")
    
    def log_content_variant_created(self, content_id: str, variant_type: str, is_ai_generated: bool):
        """Log content variant creation."""
        self.logger.info(
            "Content variant created",
            extra={
                "event_type": "content_variant_created",
                "content_id": content_id,
                "variant_type": variant_type,
                "is_ai_generated": is_ai_generated
            }
        )
        metrics.increment_counter("content_variants_created")
        
        if is_ai_generated:
            metrics.increment_counter("ai_generations")
    
    def log_error(self, error_type: str, error_message: str, context: Optional[Dict[str, Any]] = None):
        """Log error with context."""
        self.logger.error(
            f"Personalization error: {error_message}",
            extra={
                "event_type": "error",
                "error_type": error_type,
                "error_message": error_message,
                "context": context or {}
            }
        )
        metrics.record_error(error_type)


# Global logger instance
personalization_logger = PersonalizationLogger()


class HealthChecker:
    """Health check utilities for personalization service."""
    
    @staticmethod
    def check_service_health() -> Dict[str, Any]:
        """Perform comprehensive health check."""
        health_status = {
            "status": "healthy",
            "timestamp": datetime.utcnow().isoformat(),
            "checks": {}
        }
        
        try:
            # Check metrics collection
            health_status["checks"]["metrics"] = {
                "status": "healthy",
                "total_requests": metrics.metrics["requests_total"]
            }
            
            # Check cache performance
            cache_hit_rate = 0
            total_cache_requests = metrics.metrics["cache_hits"] + metrics.metrics["cache_misses"]
            if total_cache_requests > 0:
                cache_hit_rate = metrics.metrics["cache_hits"] / total_cache_requests
            
            cache_status = "healthy" if cache_hit_rate > 0.5 else "degraded"
            health_status["checks"]["cache"] = {
                "status": cache_status,
                "hit_rate": cache_hit_rate
            }
            
            # Check error rate
            error_rate = 0
            if metrics.metrics["requests_total"] > 0:
                error_rate = metrics.metrics["requests_failed"] / metrics.metrics["requests_total"]
            
            error_status = "healthy" if error_rate < 0.1 else "unhealthy"
            health_status["checks"]["errors"] = {
                "status": error_status,
                "error_rate": error_rate
            }
            
            # Overall status
            if any(check["status"] == "unhealthy" for check in health_status["checks"].values()):
                health_status["status"] = "unhealthy"
            elif any(check["status"] == "degraded" for check in health_status["checks"].values()):
                health_status["status"] = "degraded"
            
        except Exception as e:
            health_status["status"] = "unhealthy"
            health_status["error"] = str(e)
        
        return health_status
    
    @staticmethod
    def check_database_health() -> Dict[str, Any]:
        """Check database connectivity and performance."""
        try:
            from ..database.database import get_db
            
            db = next(get_db())
            
            # Test basic connectivity
            start_time = time.time()
            result = db.execute("SELECT 1").fetchone()
            query_time_ms = int((time.time() - start_time) * 1000)
            
            if result:
                return {
                    "status": "healthy",
                    "query_time_ms": query_time_ms,
                    "connection": "active"
                }
            else:
                return {
                    "status": "unhealthy",
                    "error": "Query returned no result"
                }
                
        except Exception as e:
            return {
                "status": "unhealthy",
                "error": str(e)
            }


def get_personalization_metrics() -> Dict[str, Any]:
    """Get current personalization metrics."""
    return metrics.get_metrics_summary()


def reset_metrics():
    """Reset all metrics (for testing purposes)."""
    global metrics
    metrics = PersonalizationMetrics()