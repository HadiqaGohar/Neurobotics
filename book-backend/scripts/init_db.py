#!/usr/bin/env python3
"""Initialize database tables"""

import sys
import os
from pathlib import Path

# Add the parent directory to the path so we can import from app
sys.path.append(str(Path(__file__).parent.parent))

from app.core.database import create_tables
from app.models.database import *  # Import all models to ensure they're registered
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Initialize database tables"""
    try:
        logger.info("Creating database tables...")
        create_tables()
        logger.info("Database tables created successfully!")
        
    except Exception as e:
        logger.error(f"Error creating database tables: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()