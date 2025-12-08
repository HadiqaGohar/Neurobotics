"""
Global error handling middleware for the RAG chatbot API.
"""

import logging
import traceback
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Callable
import time

logger = logging.getLogger(__name__)


class ErrorHandlingMiddleware(BaseHTTPMiddleware):
    """Middleware to handle errors globally and provide consistent error responses."""
    
    async def dispatch(self, request: Request, call_next: Callable):
        start_time = time.time()
        
        try:
            response = await call_next(request)
            
            # Log successful requests
            process_time = time.time() - start_time
            logger.info(
                f"{request.method} {request.url.path} - "
                f"Status: {response.status_code} - "
                f"Time: {process_time:.3f}s"
            )
            
            return response
            
        except HTTPException as e:
            # Log HTTP exceptions
            logger.warning(
                f"{request.method} {request.url.path} - "
                f"HTTPException: {e.status_code} - {e.detail}"
            )
            raise e
            
        except Exception as e:
            # Log unexpected errors
            process_time = time.time() - start_time
            logger.error(
                f"{request.method} {request.url.path} - "
                f"Unexpected error: {str(e)} - "
                f"Time: {process_time:.3f}s\n"
                f"Traceback: {traceback.format_exc()}"
            )
            
            # Return generic error response
            return JSONResponse(
                status_code=500,
                content={
                    "detail": "Internal server error",
                    "error_id": f"err_{int(time.time())}"
                }
            )


class SecurityMiddleware(BaseHTTPMiddleware):
    """Security middleware to add security headers and basic protection."""
    
    async def dispatch(self, request: Request, call_next: Callable):
        # Check for basic security headers
        if request.method == "POST":
            content_type = request.headers.get("content-type", "")
            if not content_type.startswith("application/json") and not content_type.startswith("multipart/form-data"):
                return JSONResponse(
                    status_code=400,
                    content={"detail": "Invalid content type"}
                )
        
        response = await call_next(request)
        
        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        
        # Remove server header
        if "server" in response.headers:
            del response.headers["server"]
        
        return response


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('app.log')
        ]
    )
    
    # Set specific log levels
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("fastapi").setLevel(logging.INFO)
    logging.getLogger("sqlalchemy").setLevel(logging.WARNING)