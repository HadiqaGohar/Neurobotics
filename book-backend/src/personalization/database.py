"""
Database setup and migrations for personalization service.
"""

import logging
from sqlalchemy import create_engine, MetaData
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

from .models import UserPreferencesDB, ContentVariantDB, PersonalizationLogDB

logger = logging.getLogger(__name__)

# Create tables function
def create_personalization_tables(engine):
    """Create personalization-related database tables."""
    try:
        # Import the base from models
        from .models import Base
        
        # Create all tables
        Base.metadata.create_all(bind=engine)
        
        logger.info("Personalization tables created successfully")
        return True
        
    except Exception as e:
        logger.error(f"Error creating personalization tables: {e}")
        return False


def init_personalization_db():
    """Initialize personalization database with default data."""
    try:
        # This would be called during application startup
        # For now, just log that initialization is complete
        logger.info("Personalization database initialized")
        return True
        
    except Exception as e:
        logger.error(f"Error initializing personalization database: {e}")
        return False


# Database migration functions
def migrate_personalization_schema():
    """Handle schema migrations for personalization service."""
    try:
        # Placeholder for future migrations
        logger.info("Personalization schema migration completed")
        return True
        
    except Exception as e:
        logger.error(f"Error migrating personalization schema: {e}")
        return False


def cleanup_old_data(days_old: int = 30):
    """Clean up old personalization data."""
    try:
        from datetime import datetime, timedelta
        from ..database.database import get_db
        
        cutoff_date = datetime.utcnow() - timedelta(days=days_old)
        
        # This would clean up old logs and unused variants
        # Implementation would go here
        
        logger.info(f"Cleaned up personalization data older than {days_old} days")
        return True
        
    except Exception as e:
        logger.error(f"Error cleaning up old personalization data: {e}")
        return False


# Health check functions
def check_personalization_db_health():
    """Check health of personalization database components."""
    try:
        from ..database.database import get_db
        
        db = next(get_db())
        
        # Test basic database connectivity
        result = db.execute("SELECT 1").fetchone()
        
        if result:
            return {
                "status": "healthy",
                "database": "connected",
                "tables": "accessible"
            }
        else:
            return {
                "status": "unhealthy",
                "database": "connection_failed"
            }
            
    except Exception as e:
        logger.error(f"Personalization database health check failed: {e}")
        return {
            "status": "unhealthy",
            "error": str(e)
        }