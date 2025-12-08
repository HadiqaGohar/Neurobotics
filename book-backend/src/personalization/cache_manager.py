"""
Caching and performance optimization for personalization system.
"""

import logging
import json
import hashlib
import time
from typing import Dict, Any, Optional, List, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


class CacheType(str, Enum):
    """Types of cache storage."""
    MEMORY = "memory"
    REDIS = "redis"
    DATABASE = "database"


class CacheStrategy(str, Enum):
    """Cache invalidation strategies."""
    TTL = "ttl"  # Time to live
    LRU = "lru"  # Least recently used
    LFU = "lfu"  # Least frequently used
    MANUAL = "manual"  # Manual invalidation


@dataclass
class CacheEntry:
    """Cache entry with metadata."""
    key: str
    data: Any
    created_at: datetime
    last_accessed: datetime
    access_count: int
    ttl_seconds: Optional[int] = None
    tags: List[str] = None
    
    def __post_init__(self):
        if self.tags is None:
            self.tags = []
    
    def is_expired(self) -> bool:
        """Check if cache entry is expired."""
        if self.ttl_seconds is None:
            return False
        return (datetime.utcnow() - self.created_at).total_seconds() > self.ttl_seconds
    
    def touch(self):
        """Update last accessed time and increment access count."""
        self.last_accessed = datetime.utcnow()
        self.access_count += 1


@dataclass
class CacheStats:
    """Cache performance statistics."""
    total_requests: int = 0
    cache_hits: int = 0
    cache_misses: int = 0
    evictions: int = 0
    total_size_bytes: int = 0
    average_access_time_ms: float = 0.0
    
    @property
    def hit_rate(self) -> float:
        """Calculate cache hit rate."""
        if self.total_requests == 0:
            return 0.0
        return self.cache_hits / self.total_requests
    
    @property
    def miss_rate(self) -> float:
        """Calculate cache miss rate."""
        return 1.0 - self.hit_rate


class PersonalizationCacheManager:
    """Advanced cache manager for personalization system."""
    
    def __init__(
        self,
        max_size: int = 10000,
        default_ttl: int = 3600,
        cache_strategy: CacheStrategy = CacheStrategy.LRU
    ):
        self.max_size = max_size
        self.default_ttl = default_ttl
        self.cache_strategy = cache_strategy
        
        # In-memory cache storage
        self.cache: Dict[str, CacheEntry] = {}
        self.stats = CacheStats()
        
        # Performance monitoring
        self.performance_metrics = {
            "cache_operations": [],
            "invalidation_events": [],
            "performance_alerts": []
        }
        
        # Cache warming configuration
        self.warming_config = {
            "enabled": True,
            "common_preferences": [],
            "popular_chapters": []
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get item from cache."""
        start_time = time.time()
        
        try:
            self.stats.total_requests += 1
            
            if key in self.cache:
                entry = self.cache[key]
                
                # Check if expired
                if entry.is_expired():
                    del self.cache[key]
                    self.stats.cache_misses += 1
                    self._record_operation("miss_expired", key, time.time() - start_time)
                    return default
                
                # Update access info
                entry.touch()
                self.stats.cache_hits += 1
                
                access_time = (time.time() - start_time) * 1000
                self._record_operation("hit", key, access_time)
                self._update_average_access_time(access_time)
                
                return entry.data
            
            else:
                self.stats.cache_misses += 1
                self._record_operation("miss", key, time.time() - start_time)
                return default
                
        except Exception as e:
            logger.error(f"Error getting cache key {key}: {e}")
            return default
    
    def set(
        self,
        key: str,
        value: Any,
        ttl: Optional[int] = None,
        tags: Optional[List[str]] = None
    ) -> bool:
        """Set item in cache."""
        try:
            # Check if we need to evict items
            if len(self.cache) >= self.max_size:
                self._evict_items()
            
            # Create cache entry
            entry = CacheEntry(
                key=key,
                data=value,
                created_at=datetime.utcnow(),
                last_accessed=datetime.utcnow(),
                access_count=0,
                ttl_seconds=ttl or self.default_ttl,
                tags=tags or []
            )
            
            self.cache[key] = entry
            self._update_cache_size()
            self._record_operation("set", key, 0)
            
            return True
            
        except Exception as e:
            logger.error(f"Error setting cache key {key}: {e}")
            return False
    
    def delete(self, key: str) -> bool:
        """Delete item from cache."""
        try:
            if key in self.cache:
                del self.cache[key]
                self._update_cache_size()
                self._record_operation("delete", key, 0)
                return True
            return False
            
        except Exception as e:
            logger.error(f"Error deleting cache key {key}: {e}")
            return False
    
    def clear(self) -> bool:
        """Clear all cache entries."""
        try:
            self.cache.clear()
            self.stats = CacheStats()
            self._record_operation("clear_all", "", 0)
            return True
            
        except Exception as e:
            logger.error(f"Error clearing cache: {e}")
            return False
    
    def invalidate_by_tags(self, tags: List[str]) -> int:
        """Invalidate cache entries by tags."""
        try:
            invalidated_count = 0
            keys_to_delete = []
            
            for key, entry in self.cache.items():
                if any(tag in entry.tags for tag in tags):
                    keys_to_delete.append(key)
            
            for key in keys_to_delete:
                if self.delete(key):
                    invalidated_count += 1
            
            self._record_invalidation_event("tag_based", tags, invalidated_count)
            return invalidated_count
            
        except Exception as e:
            logger.error(f"Error invalidating by tags {tags}: {e}")
            return 0
    
    def invalidate_by_pattern(self, pattern: str) -> int:
        """Invalidate cache entries matching pattern."""
        try:
            import re
            invalidated_count = 0
            keys_to_delete = []
            
            regex = re.compile(pattern)
            for key in self.cache.keys():
                if regex.match(key):
                    keys_to_delete.append(key)
            
            for key in keys_to_delete:
                if self.delete(key):
                    invalidated_count += 1
            
            self._record_invalidation_event("pattern_based", [pattern], invalidated_count)
            return invalidated_count
            
        except Exception as e:
            logger.error(f"Error invalidating by pattern {pattern}: {e}")
            return 0
    
    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics."""
        return {
            "cache_stats": asdict(self.stats),
            "cache_size": len(self.cache),
            "max_size": self.max_size,
            "memory_usage_mb": self.stats.total_size_bytes / (1024 * 1024),
            "performance_metrics": {
                "recent_operations": self.performance_metrics["cache_operations"][-100:],
                "recent_invalidations": self.performance_metrics["invalidation_events"][-50:],
                "alerts": self.performance_metrics["performance_alerts"][-20:]
            }
        }
    
    def warm_cache(self, warming_data: List[Dict[str, Any]]) -> int:
        """Warm cache with common data."""
        try:
            warmed_count = 0
            
            for item in warming_data:
                key = item.get("key")
                value = item.get("value")
                ttl = item.get("ttl")
                tags = item.get("tags", [])
                
                if key and value is not None:
                    if self.set(key, value, ttl, tags):
                        warmed_count += 1
            
            logger.info(f"Cache warmed with {warmed_count} items")
            return warmed_count
            
        except Exception as e:
            logger.error(f"Error warming cache: {e}")
            return 0
    
    def cleanup_expired(self) -> int:
        """Clean up expired cache entries."""
        try:
            expired_keys = []
            
            for key, entry in self.cache.items():
                if entry.is_expired():
                    expired_keys.append(key)
            
            for key in expired_keys:
                del self.cache[key]
            
            if expired_keys:
                self._update_cache_size()
                self._record_operation("cleanup_expired", f"{len(expired_keys)}_items", 0)
            
            return len(expired_keys)
            
        except Exception as e:
            logger.error(f"Error cleaning up expired entries: {e}")
            return 0
    
    def _evict_items(self):
        """Evict items based on cache strategy."""
        try:
            if self.cache_strategy == CacheStrategy.LRU:
                self._evict_lru()
            elif self.cache_strategy == CacheStrategy.LFU:
                self._evict_lfu()
            else:
                self._evict_oldest()
                
        except Exception as e:
            logger.error(f"Error evicting items: {e}")
    
    def _evict_lru(self):
        """Evict least recently used items."""
        if not self.cache:
            return
        
        # Sort by last accessed time
        sorted_items = sorted(
            self.cache.items(),
            key=lambda x: x[1].last_accessed
        )
        
        # Evict oldest 10% or at least 1 item
        evict_count = max(1, len(self.cache) // 10)
        
        for i in range(evict_count):
            key, _ = sorted_items[i]
            del self.cache[key]
            self.stats.evictions += 1
    
    def _evict_lfu(self):
        """Evict least frequently used items."""
        if not self.cache:
            return
        
        # Sort by access count
        sorted_items = sorted(
            self.cache.items(),
            key=lambda x: x[1].access_count
        )
        
        # Evict least used 10% or at least 1 item
        evict_count = max(1, len(self.cache) // 10)
        
        for i in range(evict_count):
            key, _ = sorted_items[i]
            del self.cache[key]
            self.stats.evictions += 1
    
    def _evict_oldest(self):
        """Evict oldest items."""
        if not self.cache:
            return
        
        # Sort by creation time
        sorted_items = sorted(
            self.cache.items(),
            key=lambda x: x[1].created_at
        )
        
        # Evict oldest 10% or at least 1 item
        evict_count = max(1, len(self.cache) // 10)
        
        for i in range(evict_count):
            key, _ = sorted_items[i]
            del self.cache[key]
            self.stats.evictions += 1
    
    def _update_cache_size(self):
        """Update cache size statistics."""
        try:
            total_size = 0
            for entry in self.cache.values():
                # Rough estimation of memory usage
                total_size += len(json.dumps(entry.data, default=str).encode('utf-8'))
            
            self.stats.total_size_bytes = total_size
            
        except Exception as e:
            logger.error(f"Error updating cache size: {e}")
    
    def _update_average_access_time(self, access_time_ms: float):
        """Update average access time."""
        if self.stats.cache_hits == 1:
            self.stats.average_access_time_ms = access_time_ms
        else:
            # Running average
            self.stats.average_access_time_ms = (
                (self.stats.average_access_time_ms * (self.stats.cache_hits - 1) + access_time_ms) /
                self.stats.cache_hits
            )
    
    def _record_operation(self, operation: str, key: str, duration_ms: float):
        """Record cache operation for monitoring."""
        operation_record = {
            "timestamp": datetime.utcnow().isoformat(),
            "operation": operation,
            "key": key,
            "duration_ms": duration_ms
        }
        
        self.performance_metrics["cache_operations"].append(operation_record)
        
        # Keep only recent operations
        if len(self.performance_metrics["cache_operations"]) > 1000:
            self.performance_metrics["cache_operations"] = self.performance_metrics["cache_operations"][-500:]
        
        # Check for performance alerts
        if duration_ms > 100:  # Alert if operation takes more than 100ms
            self._add_performance_alert(f"Slow cache operation: {operation} took {duration_ms:.2f}ms")
    
    def _record_invalidation_event(self, event_type: str, criteria: List[str], count: int):
        """Record cache invalidation event."""
        event_record = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": event_type,
            "criteria": criteria,
            "invalidated_count": count
        }
        
        self.performance_metrics["invalidation_events"].append(event_record)
        
        # Keep only recent events
        if len(self.performance_metrics["invalidation_events"]) > 200:
            self.performance_metrics["invalidation_events"] = self.performance_metrics["invalidation_events"][-100:]
    
    def _add_performance_alert(self, message: str):
        """Add performance alert."""
        alert = {
            "timestamp": datetime.utcnow().isoformat(),
            "message": message,
            "severity": "warning"
        }
        
        self.performance_metrics["performance_alerts"].append(alert)
        
        # Keep only recent alerts
        if len(self.performance_metrics["performance_alerts"]) > 100:
            self.performance_metrics["performance_alerts"] = self.performance_metrics["performance_alerts"][-50:]


class PersonalizationCacheKeys:
    """Cache key generators for personalization system."""
    
    @staticmethod
    def user_preferences(user_id: str) -> str:
        """Generate cache key for user preferences."""
        return f"user_prefs:{user_id}"
    
    @staticmethod
    def personalized_content(user_id: str, chapter_id: str, section_id: Optional[str] = None) -> str:
        """Generate cache key for personalized content."""
        section_part = f":{section_id}" if section_id else ""
        return f"content:{user_id}:{chapter_id}{section_part}"
    
    @staticmethod
    def content_variant(chapter_id: str, variant_type: str, target_audience_hash: str) -> str:
        """Generate cache key for content variant."""
        return f"variant:{chapter_id}:{variant_type}:{target_audience_hash}"
    
    @staticmethod
    def rule_execution_result(user_id: str, content_hash: str) -> str:
        """Generate cache key for rule execution results."""
        return f"rules:{user_id}:{content_hash}"
    
    @staticmethod
    def adaptation_result(preferences_hash: str, content_hash: str) -> str:
        """Generate cache key for content adaptation results."""
        return f"adaptation:{preferences_hash}:{content_hash}"
    
    @staticmethod
    def generate_hash(data: Any) -> str:
        """Generate hash for cache key."""
        try:
            json_str = json.dumps(data, sort_keys=True, default=str)
            return hashlib.md5(json_str.encode()).hexdigest()[:16]
        except Exception:
            return hashlib.md5(str(data).encode()).hexdigest()[:16]


class PerformanceOptimizer:
    """Performance optimization utilities for personalization system."""
    
    def __init__(self, cache_manager: PersonalizationCacheManager):
        self.cache_manager = cache_manager
        self.optimization_stats = {
            "pre_generation_count": 0,
            "batch_operations": 0,
            "query_optimizations": 0
        }
    
    def pre_generate_common_content(
        self,
        common_preferences: List[Dict[str, Any]],
        popular_chapters: List[str]
    ) -> int:
        """Pre-generate content for common preference combinations."""
        try:
            generated_count = 0
            
            for prefs in common_preferences:
                for chapter_id in popular_chapters:
                    # Generate cache key
                    prefs_hash = PersonalizationCacheKeys.generate_hash(prefs)
                    cache_key = f"pregenerated:{chapter_id}:{prefs_hash}"
                    
                    # Check if already cached
                    if self.cache_manager.get(cache_key) is None:
                        # Generate placeholder content (in real implementation, this would call the adaptation engine)
                        placeholder_content = {
                            "chapter_id": chapter_id,
                            "preferences_applied": prefs,
                            "generated_at": datetime.utcnow().isoformat(),
                            "content": f"Pre-generated content for chapter {chapter_id}"
                        }
                        
                        # Cache with longer TTL for pre-generated content
                        if self.cache_manager.set(
                            cache_key,
                            placeholder_content,
                            ttl=7200,  # 2 hours
                            tags=["pregenerated", f"chapter:{chapter_id}"]
                        ):
                            generated_count += 1
            
            self.optimization_stats["pre_generation_count"] += generated_count
            logger.info(f"Pre-generated {generated_count} content variants")
            return generated_count
            
        except Exception as e:
            logger.error(f"Error pre-generating content: {e}")
            return 0
    
    def batch_cache_operations(self, operations: List[Dict[str, Any]]) -> Dict[str, int]:
        """Perform batch cache operations for better performance."""
        try:
            results = {"success": 0, "failed": 0}
            
            for operation in operations:
                op_type = operation.get("type")
                key = operation.get("key")
                
                try:
                    if op_type == "set":
                        value = operation.get("value")
                        ttl = operation.get("ttl")
                        tags = operation.get("tags")
                        
                        if self.cache_manager.set(key, value, ttl, tags):
                            results["success"] += 1
                        else:
                            results["failed"] += 1
                    
                    elif op_type == "delete":
                        if self.cache_manager.delete(key):
                            results["success"] += 1
                        else:
                            results["failed"] += 1
                    
                    elif op_type == "get":
                        # For batch gets, we just check existence
                        if self.cache_manager.get(key) is not None:
                            results["success"] += 1
                        else:
                            results["failed"] += 1
                
                except Exception as e:
                    logger.error(f"Error in batch operation {op_type} for key {key}: {e}")
                    results["failed"] += 1
            
            self.optimization_stats["batch_operations"] += 1
            return results
            
        except Exception as e:
            logger.error(f"Error in batch cache operations: {e}")
            return {"success": 0, "failed": len(operations)}
    
    def optimize_cache_warming(self) -> Dict[str, Any]:
        """Optimize cache warming based on usage patterns."""
        try:
            stats = self.cache_manager.get_stats()
            cache_operations = stats.get("performance_metrics", {}).get("recent_operations", [])
            
            # Analyze access patterns
            key_patterns = {}
            for operation in cache_operations:
                if operation.get("operation") == "hit":
                    key = operation.get("key", "")
                    pattern = self._extract_key_pattern(key)
                    key_patterns[pattern] = key_patterns.get(pattern, 0) + 1
            
            # Identify most accessed patterns
            popular_patterns = sorted(
                key_patterns.items(),
                key=lambda x: x[1],
                reverse=True
            )[:10]
            
            optimization_suggestions = {
                "popular_patterns": popular_patterns,
                "cache_hit_rate": stats["cache_stats"]["hit_rate"],
                "recommendations": []
            }
            
            # Generate recommendations
            if stats["cache_stats"]["hit_rate"] < 0.7:
                optimization_suggestions["recommendations"].append(
                    "Consider increasing cache size or TTL values"
                )
            
            if len(popular_patterns) > 0:
                optimization_suggestions["recommendations"].append(
                    f"Pre-warm cache with patterns: {[p[0] for p in popular_patterns[:3]]}"
                )
            
            return optimization_suggestions
            
        except Exception as e:
            logger.error(f"Error optimizing cache warming: {e}")
            return {}
    
    def _extract_key_pattern(self, key: str) -> str:
        """Extract pattern from cache key."""
        try:
            parts = key.split(":")
            if len(parts) >= 2:
                return f"{parts[0]}:*"
            return key
        except Exception:
            return "unknown"
    
    def get_optimization_stats(self) -> Dict[str, Any]:
        """Get performance optimization statistics."""
        return {
            "optimization_stats": self.optimization_stats,
            "cache_performance": self.cache_manager.get_stats()
        }


# Global cache manager and optimizer instances
cache_manager = PersonalizationCacheManager(
    max_size=10000,
    default_ttl=3600,
    cache_strategy=CacheStrategy.LRU
)

performance_optimizer = PerformanceOptimizer(cache_manager)