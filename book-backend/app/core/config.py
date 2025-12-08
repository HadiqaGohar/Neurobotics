"""
Core configuration for Book Backend API
"""

import os
import logging
import logging.config
from typing import List, Dict, Any
from pydantic_settings import BaseSettings
from pydantic import validator
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings"""
    
    # Database
    database_url: str = ""
    
    # External APIs
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    
    # Authentication
    better_auth_api_key: str = os.getenv("BETTER_AUTH_API_KEY", "")
    jwt_secret_key: str = os.getenv("JWT_SECRET_KEY", "your-secret-key")
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 30
    
    # CORS
    allowed_origins: List[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
        "https://localhost:3000",
        "https://neurobotics.vercel.app"
    ]
    
    # Application
    app_name: str = "Book Backend API"
    debug: bool = os.getenv("DEBUG", "false").lower() == "true"
    environment: str = os.getenv("ENVIRONMENT", "development")
    
    @validator("database_url")
    def validate_database_url(cls, v):
        # Use NEON_DATABASE_URL if DATABASE_URL is not set
        if not v:
            neon_url = os.getenv("NEON_DATABASE_URL", "")
            if neon_url:
                return neon_url
            raise ValueError("DATABASE_URL or NEON_DATABASE_URL is required")
        return v
    
    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Allow extra fields in environment


def get_logging_config() -> Dict[str, Any]:
    """Get logging configuration"""
    return {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "detailed": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(module)s - %(funcName)s:%(lineno)d - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "json": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(module)s - %(funcName)s:%(lineno)d - %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            }
        },
        "handlers": {
            "console": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "default",
                "stream": "ext://sys.stdout"
            },
            "file": {
                "class": "logging.handlers.RotatingFileHandler",
                "level": "DEBUG",
                "formatter": "detailed",
                "filename": "logs/app.log",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
                "encoding": "utf8"
            },
            "error_file": {
                "class": "logging.handlers.RotatingFileHandler",
                "level": "ERROR",
                "formatter": "detailed",
                "filename": "logs/error.log",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
                "encoding": "utf8"
            }
        },
        "loggers": {
            "app": {
                "level": "DEBUG",
                "handlers": ["console", "file"],
                "propagate": False
            },
            "app.services": {
                "level": "DEBUG",
                "handlers": ["console", "file"],
                "propagate": False
            },
            "app.api": {
                "level": "INFO",
                "handlers": ["console", "file"],
                "propagate": False
            },
            "sqlalchemy.engine": {
                "level": "WARNING",
                "handlers": ["console"],
                "propagate": False
            },
            "uvicorn": {
                "level": "INFO",
                "handlers": ["console"],
                "propagate": False
            },
            "uvicorn.error": {
                "level": "INFO",
                "handlers": ["console", "error_file"],
                "propagate": False
            },
            "uvicorn.access": {
                "level": "INFO",
                "handlers": ["console"],
                "propagate": False
            }
        },
        "root": {
            "level": "INFO",
            "handlers": ["console", "file"]
        }
    }


def setup_logging(environment: str = "development"):
    """Setup logging configuration"""
    import os
    
    # Create logs directory if it doesn't exist
    os.makedirs("logs", exist_ok=True)
    
    # Get logging config
    config = get_logging_config()
    
    # Adjust logging level based on environment
    if environment == "production":
        config["handlers"]["console"]["level"] = "WARNING"
        config["loggers"]["app"]["level"] = "INFO"
        config["loggers"]["app.services"]["level"] = "INFO"
        config["root"]["level"] = "WARNING"
    elif environment == "testing":
        config["handlers"]["console"]["level"] = "ERROR"
        config["loggers"]["app"]["level"] = "WARNING"
        config["loggers"]["app.services"]["level"] = "WARNING"
        config["root"]["level"] = "ERROR"
    
    # Apply configuration
    logging.config.dictConfig(config)
    
    # Log startup message
    logger = logging.getLogger("app")
    logger.info(f"Logging configured for {environment} environment")


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance"""
    settings = Settings()
    
    # Setup logging when settings are first loaded
    setup_logging(settings.environment)
    
    return settings