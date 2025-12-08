"""
Security utilities for the RAG chatbot system.
"""

import re
import html
from typing import List, Optional
import hashlib
import secrets
from datetime import datetime, timedelta
import jwt
import os


class SecurityValidator:
    """Security validation utilities."""
    
    # Dangerous patterns to detect
    DANGEROUS_PATTERNS = [
        r'<script[^>]*>.*?</script>',  # Script tags
        r'javascript:',  # JavaScript URLs
        r'on\w+\s*=',  # Event handlers
        r'<iframe[^>]*>.*?</iframe>',  # Iframes
        r'<object[^>]*>.*?</object>',  # Objects
        r'<embed[^>]*>.*?</embed>',  # Embeds
        r'<link[^>]*>',  # Link tags
        r'<meta[^>]*>',  # Meta tags
        r'data:text/html',  # Data URLs
        r'vbscript:',  # VBScript
    ]
    
    # SQL injection patterns
    SQL_INJECTION_PATTERNS = [
        r'(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|UNION)\b)',
        r'(\b(OR|AND)\s+\d+\s*=\s*\d+)',
        r'(\b(OR|AND)\s+[\'"]?\w+[\'"]?\s*=\s*[\'"]?\w+[\'"]?)',
        r'(--|#|/\*|\*/)',
        r'(\bxp_\w+)',
        r'(\bsp_\w+)',
    ]
    
    @classmethod
    def sanitize_input(cls, text: str) -> str:
        """Sanitize user input to prevent XSS and other attacks."""
        if not text:
            return ""
        
        # HTML escape
        text = html.escape(text)
        
        # Remove dangerous patterns
        for pattern in cls.DANGEROUS_PATTERNS:
            text = re.sub(pattern, '', text, flags=re.IGNORECASE)
        
        # Limit length
        if len(text) > 10000:  # 10KB limit
            text = text[:10000]
        
        return text.strip()
    
    @classmethod
    def validate_query(cls, query: str) -> bool:
        """Validate if query contains potential SQL injection."""
        if not query:
            return True
        
        query_lower = query.lower()
        
        for pattern in cls.SQL_INJECTION_PATTERNS:
            if re.search(pattern, query_lower, re.IGNORECASE):
                return False
        
        return True
    
    @classmethod
    def validate_file_upload(cls, filename: str, content: bytes) -> tuple[bool, str]:
        """Validate uploaded file for security."""
        # Check filename
        if not filename or '..' in filename or '/' in filename or '\\' in filename:
            return False, "Invalid filename"
        
        # Check file extension
        allowed_extensions = {'.txt', '.pdf', '.docx', '.epub', '.md'}
        file_ext = os.path.splitext(filename)[1].lower()
        if file_ext not in allowed_extensions:
            return False, f"File type {file_ext} not allowed"
        
        # Check file size (10MB limit)
        if len(content) > 10 * 1024 * 1024:
            return False, "File too large (max 10MB)"
        
        # Check for malicious content patterns
        content_str = content[:1024].decode('utf-8', errors='ignore').lower()
        malicious_patterns = ['<script', 'javascript:', 'vbscript:', 'data:text/html']
        
        for pattern in malicious_patterns:
            if pattern in content_str:
                return False, "Potentially malicious content detected"
        
        return True, "Valid file"
    
    @classmethod
    def generate_session_token(cls) -> str:
        """Generate secure session token."""
        return secrets.token_urlsafe(32)
    
    @classmethod
    def hash_sensitive_data(cls, data: str) -> str:
        """Hash sensitive data for storage."""
        salt = os.getenv('SECURITY_SALT', 'default_salt_change_in_production')
        return hashlib.sha256((data + salt).encode()).hexdigest()


class RateLimiter:
    """Simple in-memory rate limiter."""
    
    def __init__(self):
        self.requests = {}  # {client_id: [(timestamp, count), ...]}
        self.cleanup_interval = 3600  # 1 hour
        self.last_cleanup = datetime.now()
    
    def is_allowed(self, client_id: str, max_requests: int = 100, window_minutes: int = 60) -> bool:
        """Check if request is allowed based on rate limits."""
        now = datetime.now()
        
        # Cleanup old entries periodically
        if (now - self.last_cleanup).seconds > self.cleanup_interval:
            self._cleanup_old_entries()
            self.last_cleanup = now
        
        # Get client's request history
        if client_id not in self.requests:
            self.requests[client_id] = []
        
        client_requests = self.requests[client_id]
        
        # Remove requests outside the window
        window_start = now - timedelta(minutes=window_minutes)
        client_requests[:] = [req for req in client_requests if req[0] > window_start]
        
        # Count requests in current window
        current_count = sum(req[1] for req in client_requests)
        
        if current_count >= max_requests:
            return False
        
        # Add current request
        client_requests.append((now, 1))
        return True
    
    def _cleanup_old_entries(self):
        """Remove old entries to prevent memory leaks."""
        cutoff = datetime.now() - timedelta(hours=2)
        
        for client_id in list(self.requests.keys()):
            self.requests[client_id] = [
                req for req in self.requests[client_id] if req[0] > cutoff
            ]
            
            # Remove empty entries
            if not self.requests[client_id]:
                del self.requests[client_id]


class InputValidator:
    """Input validation utilities."""
    
    @staticmethod
    def validate_session_id(session_id: str) -> bool:
        """Validate session ID format."""
        if not session_id:
            return False
        
        # UUID format validation
        uuid_pattern = r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$'
        return bool(re.match(uuid_pattern, session_id, re.IGNORECASE))
    
    @staticmethod
    def validate_message_length(message: str, max_length: int = 5000) -> bool:
        """Validate message length."""
        return len(message) <= max_length if message else False
    
    @staticmethod
    def validate_audio_data(audio_data: str) -> bool:
        """Validate base64 audio data."""
        if not audio_data:
            return False
        
        try:
            import base64
            decoded = base64.b64decode(audio_data)
            # Basic validation - should be reasonable size
            return 100 < len(decoded) < 10 * 1024 * 1024  # 100 bytes to 10MB
        except Exception:
            return False


# Global rate limiter instance
rate_limiter = RateLimiter()