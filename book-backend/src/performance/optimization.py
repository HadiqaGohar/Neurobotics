"""
Performance optimization utilities for the RAG chatbot system.
"""

import asyncio
import time
from functools import wraps
from typing import Dict, Any, Optional, Callable
import logging
from collections import defaultdict, deque
import threading

logger = logging.getLogger(__name__)


class PerformanceMonitor:
    """Monitor and track performance metrics."""
    
    def __init__(self):
        self.metrics = defaultdict(list)
        self.lock = threading.Lock()
    
    def record_metric(self, name: str, value: float, metadata: Optional[Dict] = None):
        """Record a performance metric."""
        with self.lock:
            self.metrics[name].append({
                'value': value,
                'timestamp': time.time(),
                'metadata': metadata or {}
            })
            
            # Keep only last 1000 entries per metric
            if len(self.metrics[name]) > 1000:
                self.metrics[name] = self.metrics[name][-1000:]
    
    def get_stats(self, name: str) -> Dict[str, float]:
        """Get statistics for a metric."""
        with self.lock:
            values = [m['value'] for m in self.metrics[name]]
            
            if not values:
                return {}
            
            return {
                'count': len(values),
                'mean': sum(values) / len(values),
                'min': min(values),
                'max': max(values),
                'recent': values[-10:] if len(values) >= 10 else values
            }
    
    def get_all_stats(self) -> Dict[str, Dict[str, float]]:
        """Get statistics for all metrics."""
        return {name: self.get_stats(name) for name in self.metrics.keys()}


# Global performance monitor
performance_monitor = PerformanceMonitor()


def monitor_performance(metric_name: str):
    """Decorator to monitor function performance."""
    def decorator(func: Callable):
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                success = True
                error = None
            except Exception as e:
                success = False
                error = str(e)
                raise
            finally:
                duration = time.time() - start_time
                performance_monitor.record_metric(
                    metric_name,
                    duration,
                    {'success': success, 'error': error}
                )
            return result
        
        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                success = True
                error = None
            except Exception as e:
                success = False
                error = str(e)
                raise
            finally:
                duration = time.time() - start_time
                performance_monitor.record_metric(
                    metric_name,
                    duration,
                    {'success': success, 'error': error}
                )
            return result
        
        return async_wrapper if asyncio.iscoroutinefunction(func) else sync_wrapper
    return decorator


class ConnectionPool:
    """Simple connection pool for database connections."""
    
    def __init__(self, create_connection: Callable, max_size: int = 10):
        self.create_connection = create_connection
        self.max_size = max_size
        self.pool = deque()
        self.active_connections = 0
        self.lock = asyncio.Lock()
    
    async def get_connection(self):
        """Get a connection from the pool."""
        async with self.lock:
            if self.pool:
                return self.pool.popleft()
            elif self.active_connections < self.max_size:
                self.active_connections += 1
                return await self.create_connection()
            else:
                # Wait for a connection to be returned
                while not self.pool and self.active_connections >= self.max_size:
                    await asyncio.sleep(0.01)
                return self.pool.popleft() if self.pool else await self.create_connection()
    
    async def return_connection(self, connection):
        """Return a connection to the pool."""
        async with self.lock:
            if len(self.pool) < self.max_size:
                self.pool.append(connection)
            else:
                # Close excess connections
                await self.close_connection(connection)
                self.active_connections -= 1
    
    async def close_connection(self, connection):
        """Close a connection."""
        try:
            if hasattr(connection, 'close'):
                await connection.close()
        except Exception as e:
            logger.error(f"Error closing connection: {e}")


class Cache:
    """Simple in-memory cache with TTL support."""
    
    def __init__(self, default_ttl: int = 300):  # 5 minutes default
        self.cache = {}
        self.timestamps = {}
        self.default_ttl = default_ttl
        self.lock = threading.Lock()
    
    def get(self, key: str) -> Optional[Any]:
        """Get value from cache."""
        with self.lock:
            if key not in self.cache:
                return None
            
            # Check if expired
            if time.time() - self.timestamps[key] > self.default_ttl:
                del self.cache[key]
                del self.timestamps[key]
                return None
            
            return self.cache[key]
    
    def set(self, key: str, value: Any, ttl: Optional[int] = None):
        """Set value in cache."""
        with self.lock:
            self.cache[key] = value
            self.timestamps[key] = time.time()
    
    def delete(self, key: str):
        """Delete value from cache."""
        with self.lock:
            self.cache.pop(key, None)
            self.timestamps.pop(key, None)
    
    def clear_expired(self):
        """Clear expired entries."""
        current_time = time.time()
        with self.lock:
            expired_keys = [
                key for key, timestamp in self.timestamps.items()
                if current_time - timestamp > self.default_ttl
            ]
            
            for key in expired_keys:
                self.cache.pop(key, None)
                self.timestamps.pop(key, None)
    
    def size(self) -> int:
        """Get cache size."""
        return len(self.cache)


# Global cache instance
cache = Cache()


class BatchProcessor:
    """Process items in batches to improve performance."""
    
    def __init__(self, batch_size: int = 10, max_wait: float = 1.0):
        self.batch_size = batch_size
        self.max_wait = max_wait
        self.queue = []
        self.futures = []
        self.lock = asyncio.Lock()
        self.last_process_time = time.time()
    
    async def add_item(self, item: Any, processor: Callable) -> Any:
        """Add item to batch for processing."""
        future = asyncio.Future()
        
        async with self.lock:
            self.queue.append((item, future))
            self.futures.append(future)
            
            # Process if batch is full or max wait time exceeded
            should_process = (
                len(self.queue) >= self.batch_size or
                time.time() - self.last_process_time > self.max_wait
            )
            
            if should_process:
                await self._process_batch(processor)
        
        return await future
    
    async def _process_batch(self, processor: Callable):
        """Process the current batch."""
        if not self.queue:
            return
        
        batch_items = [item for item, _ in self.queue]
        batch_futures = [future for _, future in self.queue]
        
        self.queue.clear()
        self.last_process_time = time.time()
        
        try:
            # Process batch
            results = await processor(batch_items)
            
            # Set results for futures
            for future, result in zip(batch_futures, results):
                if not future.done():
                    future.set_result(result)
                    
        except Exception as e:
            # Set exception for all futures
            for future in batch_futures:
                if not future.done():
                    future.set_exception(e)


def optimize_database_query(query: str) -> str:
    """Optimize database queries for better performance."""
    # Add LIMIT if not present
    if 'LIMIT' not in query.upper() and 'SELECT' in query.upper():
        query += ' LIMIT 1000'
    
    # Add indexes hints (database-specific)
    # This is a simplified example
    return query


async def warm_up_services():
    """Warm up services for better initial performance."""
    logger.info("Warming up services...")
    
    try:
        # Warm up embedding service
        from book_backend.src.services.embedding_service import EmbeddingService
        embedding_service = EmbeddingService()
        await embedding_service.generate_embeddings(["warmup text"])
        
        logger.info("Services warmed up successfully")
    except Exception as e:
        logger.error(f"Service warmup failed: {e}")


def get_performance_report() -> Dict[str, Any]:
    """Get comprehensive performance report."""
    return {
        'metrics': performance_monitor.get_all_stats(),
        'cache_size': cache.size(),
        'timestamp': time.time()
    }