"""
FastAPI Application for Book Backend API
"""

from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from app.core.config import get_settings
from app.api.v1.api import api_router
import logging
import traceback
from datetime import datetime

from app.qdrant.client import get_qdrant_manager
from app.qdrant.embedding_service import get_embedding_service


logger = logging.getLogger("app.main")

# Get settings
settings = get_settings()

# Create FastAPI app
app = FastAPI(
    title="Book Backend API",
    description="AI Book Creation and Management API with RAG Chatbot",
    version="1.0.0",
    docs_url="/api/v1/docs",
    redoc_url="/api/v1/redoc",
    openapi_url="/api/v1/openapi.json"
)

@app.on_event("startup")
async def startup_event():
    """
    Initialize Qdrant client and create collections on application startup.
    """
    qdrant_manager = get_qdrant_manager()
    embedding_service = get_embedding_service() # Get embedding service to access dimension
    
    # Use the embedding dimension from settings (or a default if not explicitly set by the model)
    embedding_dimension = settings.embedding_dimension
    
    logger.info(f"Initializing Qdrant collections with embedding dimension: {embedding_dimension}")
    try:
        qdrant_manager.create_collections(embedding_dimension=embedding_dimension)
        logger.info("Qdrant collections initialized successfully.")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant collections: {e}")
        # Depending on criticality, you might want to raise the exception or handle gracefully
        raise


# Error handling middleware
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for unhandled exceptions"""
    logger.error(
        f"Unhandled exception: {str(exc)}\n"
        f"Request: {request.method} {request.url}\n"
        f"Traceback: {traceback.format_exc()}"
    )
    
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal Server Error",
            "message": "An unexpected error occurred. Please try again later.",
            "timestamp": datetime.utcnow().isoformat(),
            "path": str(request.url.path)
        }
    )


@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions with consistent format"""
    logger.warning(
        f"HTTP Exception: {exc.status_code} - {exc.detail}\n"
        f"Request: {request.method} {request.url}"
    )
    
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": f"HTTP {exc.status_code}",
            "message": exc.detail,
            "timestamp": datetime.utcnow().isoformat(),
            "path": str(request.url.path)
        }
    )


@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Handle request validation errors"""
    logger.warning(
        f"Validation Error: {exc.errors()}\n"
        f"Request: {request.method} {request.url}"
    )
    
    return JSONResponse(
        status_code=422,
        content={
            "error": "Validation Error",
            "message": "Invalid request data",
            "details": exc.errors(),
            "timestamp": datetime.utcnow().isoformat(),
            "path": str(request.url.path)
        }
    )


@app.exception_handler(StarletteHTTPException)
async def starlette_exception_handler(request: Request, exc: StarletteHTTPException):
    """Handle Starlette HTTP exceptions"""
    logger.warning(
        f"Starlette HTTP Exception: {exc.status_code} - {exc.detail}\n"
        f"Request: {request.method} {request.url}"
    )
    
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": f"HTTP {exc.status_code}",
            "message": exc.detail or "An error occurred",
            "timestamp": datetime.utcnow().isoformat(),
            "path": str(request.url.path)
        }
    )


# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Book Backend API",
        "version": "1.0.0",
        "docs": "/api/v1/docs"
    }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "book-backend-api",
        "version": "1.0.0"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8001,  # Different port from main app
        reload=True
    )