"""Performance optimization system for multilingual content."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
import hashlib
import gzip
import pickle

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func
import redis
from fastapi import BackgroundTasks

from app.models.multilingual import ContentTranslation, TranslationMemory, Language
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class CacheStrategy(str, Enum):
    """Cache strategy types."""
    AGGRESSIVE = "aggressive"
    MODERATE = "moderate"
    CONSERVATIVE = "conservative"


class ContentType(str, Enum):
    """Content types for optimization."""
    TRANSLATION = "translation"
    FONT = "font"
    SEARCH_INDEX = "search_index"
    USER_PREFERENCES = "user_preferences"


class MultilingualPerformanceOptimizer:
    """Performance optimization system for multilingual content."""
    
    def __init__(self):
        self.redis_client = None
        self.cache_strategies = {
            CacheStrategy.AGGRESSIVE: {
                "translation_ttl": 86400 * 7,  # 7 days
                "font_ttl": 86400 * 30,        # 30 days
                "search_ttl": 3600 * 6,        # 6 hours
                "user_pref_ttl": 86400 * 3     # 3 days
            },
            CacheStrategy.MODERATE: {
                "translation_ttl": 86400 * 3,  # 3 days
                "font_ttl": 86400 * 14,        # 14 days
                "search_ttl": 3600 * 2,        # 2 hours
                "user_pref_ttl": 86400         # 1 day
            },
            CacheStrategy.CONSERVATIVE: {
                "translation_ttl": 3600 * 12,  # 12 hours
                "font_ttl": 86400 * 7,         # 7 days
                "search_ttl": 3600,            # 1 hour
                "user_pref_ttl": 3600 * 6      # 6 hours
            }
        }
        self.current_strategy = CacheStrategy.MODERATE
        
        # Performance metrics
        self.metrics = {
            "cache_hits": 0,
            "cache_misses": 0,
            "translation_load_times": [],
            "font_load_times": [],
            "search_times": []
        }
        
        # Initialize Redis connection
        self._init_redis()
    
    def _init_redis(self):
        """Initialize Redis connection."""
        try:
            self.redis_client = redis.Redis(
                host=getattr(settings, 'REDIS_HOST', 'localhost'),
                port=getattr(settings, 'REDIS_PORT', 6379),
                db=getattr(settings, 'REDIS_DB', 0),
                decode_responses=True
            )
            # Test connection
            self.redis_client.ping()
            logger.info("Redis connection established for multilingual caching")
        except Exception as e:
            logger.warning(f"Redis connection failed: {e}. Using in-memory cache.")
            self.redis_client = None
    
    def _generate_cache_key(self, content_type: ContentType, **kwargs) -> str:
        """Generate cache key for content."""
        key_parts = [content_type.value]
        
        # Sort kwargs for consistent key generation
        for key, value in sorted(kwargs.items()):
            if value is not None:
                key_parts.append(f"{key}:{value}")
        
        key_string = ":".join(key_parts)
        # Hash long keys to avoid Redis key length limits
        if len(key_string) > 200:
            key_string = hashlib.md5(key_string.encode()).hexdigest()
        
        return f"multilingual:{key_string}"
    
    def _get_ttl(self, content_type: ContentType) -> int:
        """Get TTL for content type based on current strategy."""
        strategy_config = self.cache_strategies[self.current_strategy]
        
        ttl_mapping = {
            ContentType.TRANSLATION: strategy_config["translation_ttl"],
            ContentType.FONT: strategy_config["font_ttl"],
            ContentType.SEARCH_INDEX: strategy_config["search_ttl"],
            ContentType.USER_PREFERENCES: strategy_config["user_pref_ttl"]
        }
        
        return ttl_mapping.get(content_type, 3600)  # Default 1 hour
    
    async def cache_translation(
        self,
        content_type: str,
        content_id: str,
        language_code: str,
        translation_data: Dict[str, Any],
        quality_score: Optional[float] = None
    ) -> bool:
        """Cache translation with compression and metadata."""
        try:
            cache_key = self._generate_cache_key(
                ContentType.TRANSLATION,
                content_type=content_type,
                content_id=content_id,
                language_code=language_code
            )
            
            # Prepare cache data with metadata
            cache_data = {
                "translation": translation_data,
                "quality_score": quality_score,
                "cached_at": datetime.utcnow().isoformat(),
                "version": "1.0"
            }
            
            # Compress data for storage efficiency
            compressed_data = gzip.compress(
                json.dumps(cache_data).encode('utf-8')
            )
            
            if self.redis_client:
                ttl = self._get_ttl(ContentType.TRANSLATION)
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.setex(
                        cache_key,
                        ttl,
                        compressed_data
                    )
                )
            
            logger.debug(f"Cached translation: {cache_key}")
            return True
            
        except Exception as e:
            logger.error(f"Error caching translation: {e}")
            return False
    
    async def get_cached_translation(
        self,
        content_type: str,
        content_id: str,
        language_code: str
    ) -> Optional[Dict[str, Any]]:
        """Retrieve cached translation with decompression."""
        try:
            cache_key = self._generate_cache_key(
                ContentType.TRANSLATION,
                content_type=content_type,
                content_id=content_id,
                language_code=language_code
            )
            
            if not self.redis_client:
                self.metrics["cache_misses"] += 1
                return None
            
            # Get compressed data
            compressed_data = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.get(cache_key)
            )
            
            if not compressed_data:
                self.metrics["cache_misses"] += 1
                return None
            
            # Decompress and parse
            if isinstance(compressed_data, str):
                compressed_data = compressed_data.encode('utf-8')
            
            decompressed_data = gzip.decompress(compressed_data)
            cache_data = json.loads(decompressed_data.decode('utf-8'))
            
            self.metrics["cache_hits"] += 1
            logger.debug(f"Cache hit for translation: {cache_key}")
            
            return cache_data
            
        except Exception as e:
            logger.error(f"Error retrieving cached translation: {e}")
            self.metrics["cache_misses"] += 1
            return None
    
    async def preload_translations(
        self,
        content_list: List[Tuple[str, str, str]],  # (content_type, content_id, language_code)
        db: Session,
        background_tasks: BackgroundTasks
    ) -> Dict[str, Any]:
        """Preload translations into cache."""
        try:
            preload_results = {
                "total": len(content_list),
                "cached": 0,
                "errors": 0,
                "skipped": 0
            }
            
            for content_type, content_id, language_code in content_list:
                try:
                    # Check if already cached
                    cached = await self.get_cached_translation(
                        content_type, content_id, language_code
                    )
                    
                    if cached:
                        preload_results["skipped"] += 1
                        continue
                    
                    # Load from database
                    translation = db.query(ContentTranslation).filter(
                        and_(
                            ContentTranslation.content_type == content_type,
                            ContentTranslation.content_id == content_id,
                            ContentTranslation.language_code == language_code
                        )
                    ).first()
                    
                    if translation:
                        translation_data = {
                            "id": translation.id,
                            "title": translation.title,
                            "content": translation.content,
                            "status": translation.translation_status.value,
                            "quality_score": translation.quality_score,
                            "updated_at": translation.updated_at.isoformat()
                        }
                        
                        # Cache in background
                        background_tasks.add_task(
                            self.cache_translation,
                            content_type,
                            content_id,
                            language_code,
                            translation_data,
                            translation.quality_score
                        )
                        
                        preload_results["cached"] += 1
                    else:
                        preload_results["skipped"] += 1
                        
                except Exception as e:
                    logger.error(f"Error preloading translation {content_type}:{content_id}:{language_code}: {e}")
                    preload_results["errors"] += 1
            
            return preload_results
            
        except Exception as e:
            logger.error(f"Error in preload_translations: {e}")
            return {"error": str(e)}
    
    async def optimize_font_loading(
        self,
        language_code: str,
        font_variants: List[str] = None
    ) -> Dict[str, Any]:
        """Optimize font loading for specific language."""
        try:
            if font_variants is None:
                font_variants = ["regular", "bold"]
            
            optimization_result = {
                "language_code": language_code,
                "optimized_fonts": [],
                "preload_urls": [],
                "fallback_fonts": []
            }
            
            # Define font configurations for different languages
            font_configs = {
                "ur": {
                    "primary": "Noto Nastaliq Urdu",
                    "fallback": ["Arial Unicode MS", "Tahoma", "sans-serif"],
                    "variants": {
                        "regular": {"weight": 400, "style": "normal"},
                        "bold": {"weight": 700, "style": "normal"}
                    },
                    "subset": "urdu",
                    "display": "swap"
                },
                "ar": {
                    "primary": "Noto Sans Arabic",
                    "fallback": ["Arial Unicode MS", "Tahoma", "sans-serif"],
                    "variants": {
                        "regular": {"weight": 400, "style": "normal"},
                        "bold": {"weight": 700, "style": "normal"}
                    },
                    "subset": "arabic",
                    "display": "swap"
                },
                "en": {
                    "primary": "Inter",
                    "fallback": ["system-ui", "-apple-system", "sans-serif"],
                    "variants": {
                        "regular": {"weight": 400, "style": "normal"},
                        "bold": {"weight": 700, "style": "normal"}
                    },
                    "subset": "latin",
                    "display": "swap"
                }
            }
            
            config = font_configs.get(language_code, font_configs["en"])
            
            # Generate optimized font URLs
            for variant in font_variants:
                if variant in config["variants"]:
                    variant_config = config["variants"][variant]
                    
                    # Generate Google Fonts URL with optimizations
                    font_url = self._generate_optimized_font_url(
                        config["primary"],
                        variant_config["weight"],
                        config["subset"],
                        config["display"]
                    )
                    
                    optimization_result["preload_urls"].append({
                        "url": font_url,
                        "variant": variant,
                        "preload": True,
                        "crossorigin": "anonymous"
                    })
            
            optimization_result["fallback_fonts"] = config["fallback"]
            optimization_result["optimized_fonts"] = [config["primary"]]
            
            # Cache font optimization result
            await self._cache_font_optimization(language_code, optimization_result)
            
            return optimization_result
            
        except Exception as e:
            logger.error(f"Error optimizing font loading for {language_code}: {e}")
            return {"error": str(e)}
    
    def _generate_optimized_font_url(
        self,
        font_family: str,
        weight: int,
        subset: str,
        display: str
    ) -> str:
        """Generate optimized Google Fonts URL."""
        # Replace spaces with +
        family_param = font_family.replace(" ", "+")
        
        # Build URL with optimizations
        url = f"https://fonts.googleapis.com/css2?"
        url += f"family={family_param}:wght@{weight}"
        url += f"&subset={subset}"
        url += f"&display={display}"
        
        return url
    
    async def _cache_font_optimization(
        self,
        language_code: str,
        optimization_result: Dict[str, Any]
    ) -> None:
        """Cache font optimization result."""
        try:
            cache_key = self._generate_cache_key(
                ContentType.FONT,
                language_code=language_code
            )
            
            if self.redis_client:
                ttl = self._get_ttl(ContentType.FONT)
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.setex(
                        cache_key,
                        ttl,
                        json.dumps(optimization_result)
                    )
                )
                
        except Exception as e:
            logger.error(f"Error caching font optimization: {e}")
    
    async def implement_lazy_loading(
        self,
        content_items: List[Dict[str, Any]],
        viewport_threshold: float = 0.1
    ) -> Dict[str, Any]:
        """Implement lazy loading strategy for translation resources."""
        try:
            lazy_loading_config = {
                "viewport_threshold": viewport_threshold,
                "loading_strategy": "intersection_observer",
                "preload_count": 3,  # Number of items to preload
                "batch_size": 5,     # Items to load per batch
                "content_items": []
            }
            
            for i, item in enumerate(content_items):
                item_config = {
                    "id": item.get("id"),
                    "content_type": item.get("content_type"),
                    "language_code": item.get("language_code"),
                    "priority": "high" if i < 3 else "normal",
                    "lazy_load": i >= 3,
                    "estimated_size": self._estimate_content_size(item)
                }
                
                lazy_loading_config["content_items"].append(item_config)
            
            return lazy_loading_config
            
        except Exception as e:
            logger.error(f"Error implementing lazy loading: {e}")
            return {"error": str(e)}
    
    def _estimate_content_size(self, content_item: Dict[str, Any]) -> int:
        """Estimate content size in bytes."""
        try:
            size = 0
            
            # Estimate based on content length
            if "content" in content_item and content_item["content"]:
                # Assume UTF-8 encoding, Urdu characters may be 2-3 bytes
                content_length = len(content_item["content"])
                size += content_length * 2.5  # Average for mixed content
            
            if "title" in content_item and content_item["title"]:
                size += len(content_item["title"]) * 2.5
            
            # Add metadata overhead
            size += 500  # Estimated metadata size
            
            return int(size)
            
        except Exception as e:
            logger.error(f"Error estimating content size: {e}")
            return 1000  # Default estimate
    
    async def setup_cdn_optimization(
        self,
        content_types: List[str],
        regions: List[str] = None
    ) -> Dict[str, Any]:
        """Set up CDN optimization for multilingual content."""
        try:
            if regions is None:
                regions = ["us-east-1", "eu-west-1", "ap-south-1"]  # Default regions
            
            cdn_config = {
                "enabled": True,
                "regions": regions,
                "content_types": content_types,
                "cache_policies": {},
                "compression": {
                    "enabled": True,
                    "algorithms": ["gzip", "brotli"],
                    "min_size": 1024  # Minimum size to compress
                },
                "edge_locations": []
            }
            
            # Define cache policies for different content types
            for content_type in content_types:
                if content_type == "translation":
                    cdn_config["cache_policies"][content_type] = {
                        "ttl": 86400 * 7,  # 7 days
                        "browser_cache": 3600 * 24,  # 24 hours
                        "vary_headers": ["Accept-Language", "Accept-Encoding"]
                    }
                elif content_type == "font":
                    cdn_config["cache_policies"][content_type] = {
                        "ttl": 86400 * 30,  # 30 days
                        "browser_cache": 86400 * 7,  # 7 days
                        "vary_headers": ["Accept-Encoding"]
                    }
                elif content_type == "search_index":
                    cdn_config["cache_policies"][content_type] = {
                        "ttl": 3600 * 6,  # 6 hours
                        "browser_cache": 3600,  # 1 hour
                        "vary_headers": ["Accept-Language"]
                    }
            
            # Generate edge location configurations
            for region in regions:
                edge_config = {
                    "region": region,
                    "endpoint": f"https://cdn-{region}.example.com",
                    "health_check": f"https://cdn-{region}.example.com/health",
                    "priority": 1 if region == "us-east-1" else 2
                }
                cdn_config["edge_locations"].append(edge_config)
            
            return cdn_config
            
        except Exception as e:
            logger.error(f"Error setting up CDN optimization: {e}")
            return {"error": str(e)}
    
    async def monitor_performance(self) -> Dict[str, Any]:
        """Monitor multilingual performance metrics."""
        try:
            # Calculate cache hit ratio
            total_requests = self.metrics["cache_hits"] + self.metrics["cache_misses"]
            cache_hit_ratio = (
                self.metrics["cache_hits"] / max(total_requests, 1)
            ) * 100
            
            # Calculate average load times
            avg_translation_time = (
                sum(self.metrics["translation_load_times"]) / 
                max(len(self.metrics["translation_load_times"]), 1)
            )
            
            avg_font_time = (
                sum(self.metrics["font_load_times"]) / 
                max(len(self.metrics["font_load_times"]), 1)
            )
            
            avg_search_time = (
                sum(self.metrics["search_times"]) / 
                max(len(self.metrics["search_times"]), 1)
            )
            
            performance_report = {
                "timestamp": datetime.utcnow().isoformat(),
                "cache_performance": {
                    "hit_ratio": round(cache_hit_ratio, 2),
                    "total_hits": self.metrics["cache_hits"],
                    "total_misses": self.metrics["cache_misses"],
                    "total_requests": total_requests
                },
                "load_times": {
                    "avg_translation_time": round(avg_translation_time, 3),
                    "avg_font_time": round(avg_font_time, 3),
                    "avg_search_time": round(avg_search_time, 3)
                },
                "optimization_status": {
                    "cache_strategy": self.current_strategy.value,
                    "redis_connected": self.redis_client is not None,
                    "cdn_enabled": True  # Would be dynamic in real implementation
                },
                "recommendations": []
            }
            
            # Generate performance recommendations
            if cache_hit_ratio < 70:
                performance_report["recommendations"].append({
                    "type": "cache_optimization",
                    "message": "Cache hit ratio is below 70%. Consider increasing cache TTL or preloading more content.",
                    "priority": "high"
                })
            
            if avg_translation_time > 2.0:
                performance_report["recommendations"].append({
                    "type": "translation_performance",
                    "message": "Translation load time exceeds 2 seconds. Consider implementing more aggressive caching.",
                    "priority": "medium"
                })
            
            if avg_font_time > 1.0:
                performance_report["recommendations"].append({
                    "type": "font_optimization",
                    "message": "Font load time exceeds 1 second. Consider font subsetting or preloading.",
                    "priority": "medium"
                })
            
            return performance_report
            
        except Exception as e:
            logger.error(f"Error monitoring performance: {e}")
            return {"error": str(e)}
    
    def record_load_time(self, content_type: ContentType, load_time: float) -> None:
        """Record load time for performance monitoring."""
        try:
            if content_type == ContentType.TRANSLATION:
                self.metrics["translation_load_times"].append(load_time)
                # Keep only last 100 measurements
                if len(self.metrics["translation_load_times"]) > 100:
                    self.metrics["translation_load_times"] = self.metrics["translation_load_times"][-100:]
            
            elif content_type == ContentType.FONT:
                self.metrics["font_load_times"].append(load_time)
                if len(self.metrics["font_load_times"]) > 100:
                    self.metrics["font_load_times"] = self.metrics["font_load_times"][-100:]
            
            elif content_type == ContentType.SEARCH_INDEX:
                self.metrics["search_times"].append(load_time)
                if len(self.metrics["search_times"]) > 100:
                    self.metrics["search_times"] = self.metrics["search_times"][-100:]
                    
        except Exception as e:
            logger.error(f"Error recording load time: {e}")
    
    async def clear_cache(
        self,
        content_type: Optional[ContentType] = None,
        language_code: Optional[str] = None
    ) -> Dict[str, Any]:
        """Clear cache with optional filters."""
        try:
            if not self.redis_client:
                return {"error": "Redis not available"}
            
            # Build pattern for cache keys to delete
            if content_type and language_code:
                pattern = f"multilingual:{content_type.value}:*:{language_code}*"
            elif content_type:
                pattern = f"multilingual:{content_type.value}:*"
            elif language_code:
                pattern = f"multilingual:*:{language_code}*"
            else:
                pattern = "multilingual:*"
            
            # Get matching keys
            keys = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.redis_client.keys(pattern)
            )
            
            # Delete keys
            deleted_count = 0
            if keys:
                deleted_count = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self.redis_client.delete(*keys)
                )
            
            return {
                "cleared": True,
                "deleted_keys": deleted_count,
                "pattern": pattern
            }
            
        except Exception as e:
            logger.error(f"Error clearing cache: {e}")
            return {"error": str(e)}


# Global performance optimizer instance
performance_optimizer = MultilingualPerformanceOptimizer()