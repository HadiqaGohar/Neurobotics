"""Main multilingual routes module."""

from fastapi import APIRouter
from .community_routes import router as community_router
from .search_routes import router as search_router
from .cultural_routes import router as cultural_router
from .workflow_routes import router as workflow_router

# Create main multilingual router
router = APIRouter(prefix="/multilingual", tags=["multilingual"])

# Include all multilingual sub-routers
router.include_router(community_router)
router.include_router(search_router)
router.include_router(cultural_router)
router.include_router(workflow_router)