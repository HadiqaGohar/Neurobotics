"Comprehensive security middleware for the authentication system."

import logging
import time
import json
from typing import Callable, Dict, Any, Optional
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from src.security.security_utils import SecurityValidator, rate_limiter
import ipaddress
import re

logger = logging.getLogger(__name__)


class ComprehensiveSecurityMiddleware(BaseHTTPMiddleware):
    """Comprehensive security middleware with multiple protection layers."""
    
    def __init__(self, app, config: Optional[Dict[str, Any]] = None):
        super().__init__(app)
        self.config = config or {}
        
        # Security configuration
        self.enable_rate_limiting = self.config.get('enable_rate_limiting', True)
        self.enable_ip_filtering = self.config.get('enable_ip_filtering', True)
        self.enable_request_validation = self.config.get('enable_request_validation', True)
        self.enable_security_headers = self.config.get('enable_security_headers', True)
        self.enable_csrf_protection = self.config.get('enable_csrf_protection', True)
        
        # Rate limiting configuration
        self.rate_limit_requests = self.config.get('rate_limit_requests', 100)
        self.rate_limit_window = self.config.get('rate_limit_window', 60)
        
        # Blocked IPs and suspicious patterns
        self.blocked_ips = set(self.config.get('blocked_ips', []))
        self.suspicious_user_agents = [
            'sqlmap', 'nikto', 'nmap', 'masscan', 'zap', 'burp',
            'w3af', 'acunetix', 'nessus', 'openvas'
        ]
        
        # Sensitive endpoints that require extra protection
        self.sensitive_endpoints = {
            '/auth/login': {'max_requests': 10, 'window': 60},
            '/auth/signup': {'max_requests': 5, 'window': 60},
            '/auth/refresh': {'max_requests': 20, 'window': 60},
            '/auth/forgot-password': {'max_requests': 3, 'window': 300},
            '/auth/reset-password': {'max_requests': 5, 'window': 300},
        }
    
    async def dispatch(self, request: Request, call_next: Callable):
        # Skip security checks for OPTIONS requests (CORS preflight)
        if request.method == 'OPTIONS':
            return await call_next(request)

        start_time = time.time()
        client_ip = self._get_client_ip(request)
        
        try:
            
            # 1. IP filtering
            if self.enable_ip_filtering and not self._check_ip_allowed(client_ip):
                logger.warning(f"Blocked request from IP: {client_ip}")
                return self._create_error_response(
                    status.HTTP_403_FORBIDDEN,
                    "Access denied"
                )
            
            # 2. User agent validation
            if not self._check_user_agent(request):
                logger.warning(f"Suspicious user agent from IP {client_ip}: {request.headers.get('user-agent')}")
                return self._create_error_response(
                    status.HTTP_403_FORBIDDEN,
                    "Access denied"
                )
            
            # 3. Rate limiting
            if self.enable_rate_limiting and not self._check_rate_limit(request, client_ip):
                logger.warning(f"Rate limit exceeded for IP: {client_ip}")
                return self._create_error_response(
                    status.HTTP_429_TOO_MANY_REQUESTS,
                    "Rate limit exceeded"
                )
            
            # 4. Request validation
            if self.enable_request_validation:
                validation_result = await self._validate_request(request)
                if not validation_result['valid']:
                    logger.warning(f"Invalid request from IP {client_ip}: {validation_result['reason']}")
                    return self._create_error_response(
                        status.HTTP_400_BAD_REQUEST,
                        validation_result['reason']
                    )
            
            # 5. CSRF protection for state-changing operations
            if self.enable_csrf_protection and request.method in ['POST', 'PUT', 'DELETE', 'PATCH']:
                if not self._check_csrf_token(request):
                    logger.warning(f"CSRF token validation failed for IP: {client_ip}")
                    # For now, just log - in production, you might want to block
                    # return self._create_error_response(
                    #     status.HTTP_403_FORBIDDEN,
                    #     "CSRF token validation failed"
                    # )
            
            # Process the request
            response = await call_next(request)
            
            # 6. Add security headers
            if self.enable_security_headers:
                response = self._add_security_headers(response)
            
            # Log successful request
            process_time = time.time() - start_time
            logger.info(
                f"Security check passed - {request.method} {request.url.path} - "
                f"IP: {client_ip} - Time: {process_time:.3f}s"
            )
            
            return response
            
        except HTTPException as e:
            logger.warning(
                f"HTTP exception in security middleware - IP: {client_ip} - "
                f"Status: {e.status_code} - Detail: {e.detail}"
            )
            raise e
            
        except Exception as e:
            logger.error(
                f"Unexpected error in security middleware - IP: {client_ip} - "
                f"Error: {str(e)}"
            )
            return self._create_error_response(
                status.HTTP_500_INTERNAL_SERVER_ERROR,
                "Internal server error"
            )
    
    def _get_client_ip(self, request: Request) -> str:
        """Get the real client IP address."""
        # Check for forwarded headers (in order of preference)
        forwarded_headers = [
            'X-Forwarded-For',
            'X-Real-IP',
            'X-Client-IP',
            'CF-Connecting-IP',  # Cloudflare
            'True-Client-IP',    # Akamai
        ]
        
        for header in forwarded_headers:
            if header in request.headers:
                ip = request.headers[header].split(',')[0].strip()
                if self._is_valid_ip(ip):
                    return ip
        
        # Fallback to direct connection IP
        return request.client.host if request.client else 'unknown'
    
    def _is_valid_ip(self, ip: str) -> bool:
        """Validate IP address format."""
        try:
            ipaddress.ip_address(ip)
            return True
        except ValueError:
            return False
    
    def _check_ip_allowed(self, client_ip: str) -> bool:
        """Check if IP is allowed (not in blocked list)."""
        if client_ip in self.blocked_ips:
            return False
        
        # Check for private/local IPs in development
        try:
            ip_obj = ipaddress.ip_address(client_ip)
            if ip_obj.is_private or ip_obj.is_loopback:
                return True  # Allow private IPs in development
        except ValueError:
            pass
        
        return True
    
    def _check_user_agent(self, request: Request) -> bool:
        """Check if user agent is suspicious."""
        user_agent = request.headers.get('user-agent', '').lower()
        
        # Block empty user agents
        if not user_agent:
            return False
        
        # Check for suspicious patterns
        for suspicious in self.suspicious_user_agents:
            if suspicious in user_agent:
                return False
        
        return True
    
    def _check_rate_limit(self, request: Request, client_ip: str) -> bool:
        """Check rate limiting for the request."""
        path = request.url.path
        
        # Check if this is a sensitive endpoint
        if path in self.sensitive_endpoints:
            config = self.sensitive_endpoints[path]
            return rate_limiter.is_allowed(
                f"{client_ip}:{path}",
                config['max_requests'],
                config['window'] // 60  # Convert to minutes
            )
        
        # General rate limiting
        return rate_limiter.is_allowed(
            client_ip,
            self.rate_limit_requests,
            self.rate_limit_window // 60  # Convert to minutes
        )
    
    async def _validate_request(self, request: Request) -> Dict[str, Any]:
        """Validate request for security issues."""
        try:
            # Check request size
            content_length = request.headers.get('content-length')
            if content_length and int(content_length) > 10 * 1024 * 1024:  # 10MB limit
                return {'valid': False, 'reason': 'Request too large'}
            
            # Check content type for POST requests
            if request.method == 'POST':
                content_type = request.headers.get('content-type', '')
                allowed_types = [
                    'application/json',
                    'application/x-www-form-urlencoded',
                    'multipart/form-data'
                ]
                
                if not any(allowed_type in content_type for allowed_type in allowed_types):
                    return {'valid': False, 'reason': 'Invalid content type'}
            
            # Validate URL path
            path = str(request.url.path)
            if not SecurityValidator.sanitize_input(path) == path:
                return {'valid': False, 'reason': 'Invalid characters in path'}
            
            # Check for path traversal attempts
            if '../' in path or '..\' in path:
                return {'valid': False, 'reason': 'Path traversal attempt detected'}
            
            # Validate query parameters
            for key, value in request.query_params.items():
                if not SecurityValidator.validate_query(value):
                    return {'valid': False, 'reason': f'Invalid query parameter: {key}'}
            
            return {'valid': True, 'reason': 'Valid request'}
            
        except Exception as e:
            logger.error(f"Error validating request: {e}")
            return {'valid': False, 'reason': 'Request validation error'}
    
    def _check_csrf_token(self, request: Request) -> bool:
        """Check CSRF token for state-changing operations."""
        # For now, this is a placeholder implementation
        # In production, implement proper CSRF token validation
        
        # Check for CSRF token in headers
        csrf_token = request.headers.get('X-CSRF-Token')
        if not csrf_token:
            # Also check in form data for form submissions
            # This would require reading the request body
            pass
        
        # For now, return True to not break existing functionality
        # TODO: Implement proper CSRF token validation
        return True
    
    def _add_security_headers(self, response) -> Any:
        """Add comprehensive security headers to response."""
        security_headers = {
            # Prevent MIME type sniffing
            'X-Content-Type-Options': 'nosniff',
            
            # Prevent clickjacking
            'X-Frame-Options': 'DENY',
            
            # XSS protection
            'X-XSS-Protection': '1; mode=block',
            
            # Referrer policy
            'Referrer-Policy': 'strict-origin-when-cross-origin',
            
            # Content Security Policy
            'Content-Security-Policy': (
                "default-src 'self'; "
                "script-src 'self' 'unsafe-inline' 'unsafe-eval'; "
                "style-src 'self' 'unsafe-inline'; "
                "img-src 'self' data: https:; "
                "font-src 'self' https:; "
                "connect-src 'self' https:; "
                "frame-ancestors 'none';"
            ),
            
            # Strict Transport Security (HTTPS only)
            'Strict-Transport-Security': 'max-age=31536000; includeSubDomains',
            
            # Permissions Policy
            'Permissions-Policy': (
                "geolocation=(), microphone=(), camera=(), "
                "payment=(), usb=(), magnetometer=(), gyroscope=()"
            ),
        }
        
        for header, value in security_headers.items():
            response.headers[header] = value
        
        # Remove server information
        if 'server' in response.headers:
            del response.headers['server']
        
        return response
    
    def _create_error_response(self, status_code: int, message: str) -> JSONResponse:
        """Create a standardized error response."""
        return JSONResponse(
            status_code=status_code,
            content={
                "detail": message,
                "error_code": f"SEC_{status_code}",
                "timestamp": int(time.time())
            }
        )


class CSRFProtectionMiddleware(BaseHTTPMiddleware):
    """CSRF protection middleware for authentication endpoints."""
    
    def __init__(self, app, secret_key: str):
        super().__init__(app)
        self.secret_key = secret_key
        self.protected_methods = {'POST', 'PUT', 'DELETE', 'PATCH'}
        self.protected_paths = {
            '/auth/login',
            '/auth/signup',
            '/auth/logout',
            '/auth/refresh',
            '/auth/me',
            '/auth/forgot-password',
            '/auth/reset-password'
        }
    
    async def dispatch(self, request: Request, call_next: Callable):
        # Check if this request needs CSRF protection
        if (request.method in self.protected_methods and 
            request.url.path in self.protected_paths):
            
            # For now, just log CSRF checks
            # TODO: Implement proper CSRF token generation and validation
            logger.info(f"CSRF check for {request.method} {request.url.path}")
        
        response = await call_next(request)
        return response


class HTTPSRedirectMiddleware(BaseHTTPMiddleware):
    """Middleware to enforce HTTPS in production."""
    
    def __init__(self, app, enforce_https: bool = False):
        super().__init__(app)
        self.enforce_https = enforce_https
    
    async def dispatch(self, request: Request, call_next: Callable):
        # Enforce HTTPS in production
        if self.enforce_https and request.url.scheme != 'https':
            # Check if request came through a proxy with HTTPS
            forwarded_proto = request.headers.get('X-Forwarded-Proto')
            if forwarded_proto != 'https':
                https_url = request.url.replace(scheme='https')
                return JSONResponse(
                    status_code=status.HTTP_301_MOVED_PERMANENTLY,
                    headers={'Location': str(https_url)}
                )
        
        response = await call_next(request)
        return response
