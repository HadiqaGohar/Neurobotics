"""Rate limiting utilities for authentication endpoints"""

import time
from typing import Dict, Optional
from collections import defaultdict, deque
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)


class RateLimiter:
    """Simple in-memory rate limiter for authentication endpoints"""
    
    def __init__(self):
        # Store request timestamps for each client
        self._requests: Dict[str, deque] = defaultdict(deque)
        # Store failed login attempts
        self._failed_attempts: Dict[str, deque] = defaultdict(deque)
        # Store account lockouts
        self._lockouts: Dict[str, datetime] = {}
    
    def is_allowed(
        self, 
        client_id: str, 
        max_requests: int = 10, 
        window_minutes: int = 15,
        action: str = "general"
    ) -> bool:
        """
        Check if request is allowed based on rate limits
        
        Args:
            client_id: Unique identifier for the client (IP, user ID, etc.)
            max_requests: Maximum number of requests allowed
            window_minutes: Time window in minutes
            action: Type of action (login, register, etc.)
        
        Returns:
            True if request is allowed, False otherwise
        """
        now = datetime.now()
        window_start = now - timedelta(minutes=window_minutes)
        
        # Clean old requests
        requests = self._requests[client_id]
        while requests and requests[0] < window_start:
            requests.popleft()
        
        # Check if limit exceeded
        if len(requests) >= max_requests:
            logger.warning(f"Rate limit exceeded for {client_id}: {len(requests)}/{max_requests} in {window_minutes}min")
            return False
        
        # Add current request
        requests.append(now)
        return True
    
    def record_failed_login(self, identifier: str) -> bool:
        """
        Record a failed login attempt and check if account should be locked
        
        Args:
            identifier: Email or user identifier
            
        Returns:
            True if account should be locked, False otherwise
        """
        now = datetime.now()
        window_start = now - timedelta(minutes=15)  # 15-minute window
        
        # Clean old failed attempts
        attempts = self._failed_attempts[identifier]
        while attempts and attempts[0] < window_start:
            attempts.popleft()
        
        # Add current failed attempt
        attempts.append(now)
        
        # Check if should lock account (5 failed attempts in 15 minutes)
        if len(attempts) >= 5:
            self._lockouts[identifier] = now + timedelta(minutes=30)  # Lock for 30 minutes
            logger.warning(f"Account locked for {identifier} due to {len(attempts)} failed attempts")
            return True
        
        return False
    
    def is_account_locked(self, identifier: str) -> bool:
        """
        Check if account is currently locked
        
        Args:
            identifier: Email or user identifier
            
        Returns:
            True if account is locked, False otherwise
        """
        if identifier not in self._lockouts:
            return False
        
        lockout_until = self._lockouts[identifier]
        if datetime.now() > lockout_until:
            # Lockout expired, remove it
            del self._lockouts[identifier]
            return False
        
        return True
    
    def clear_failed_attempts(self, identifier: str):
        """Clear failed login attempts for successful login"""
        if identifier in self._failed_attempts:
            self._failed_attempts[identifier].clear()
        if identifier in self._lockouts:
            del self._lockouts[identifier]
    
    def get_lockout_time_remaining(self, identifier: str) -> Optional[int]:
        """
        Get remaining lockout time in seconds
        
        Args:
            identifier: Email or user identifier
            
        Returns:
            Remaining seconds or None if not locked
        """
        if identifier not in self._lockouts:
            return None
        
        lockout_until = self._lockouts[identifier]
        remaining = lockout_until - datetime.now()
        
        if remaining.total_seconds() <= 0:
            del self._lockouts[identifier]
            return None
        
        return int(remaining.total_seconds())


# Global rate limiter instance
rate_limiter = RateLimiter()


def get_client_ip(request) -> str:
    """Extract client IP from request"""
    # Check for forwarded headers first (for reverse proxies)
    forwarded_for = request.headers.get('X-Forwarded-For')
    if forwarded_for:
        return forwarded_for.split(',')[0].strip()
    
    real_ip = request.headers.get('X-Real-IP')
    if real_ip:
        return real_ip
    
    # Fallback to direct client IP
    return request.client.host if hasattr(request, 'client') else 'unknown'


def check_rate_limit(
    request, 
    max_requests: int = 10, 
    window_minutes: int = 15,
    action: str = "general"
) -> bool:
    """
    Convenience function to check rate limit for a request
    
    Args:
        request: FastAPI request object
        max_requests: Maximum requests allowed
        window_minutes: Time window in minutes
        action: Action type for logging
        
    Returns:
        True if allowed, False if rate limited
    """
    client_ip = get_client_ip(request)
    return rate_limiter.is_allowed(client_ip, max_requests, window_minutes, action)