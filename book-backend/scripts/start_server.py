#!/usr/bin/env python3
"""Start the FastAPI server"""

import sys
import os
from pathlib import Path

# Add the parent directory to the path so we can import from app
sys.path.append(str(Path(__file__).parent.parent))

import uvicorn
from app.core.config import get_settings

settings = get_settings()


def main():
    """Start the FastAPI server"""
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8001,
        reload=settings.debug,
        log_level="info" if not settings.debug else "debug"
    )


if __name__ == "__main__":
    main()