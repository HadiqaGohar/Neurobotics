"""Main API router for v1 endpoints"""

from fastapi import APIRouter

from app.api.v1.endpoints import auth, content, chatbot, user
from src.personalization.routes import router as personalization_router
from src.multilingual.routes import router as multilingual_router

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(auth.router, prefix="/auth", tags=["authentication"])
api_router.include_router(content.router, prefix="/content", tags=["content"])
api_router.include_router(chatbot.router, prefix="/chatbot", tags=["chatbot"])
api_router.include_router(user.router, prefix="/user", tags=["user"])
api_router.include_router(personalization_router, tags=["personalization"])
api_router.include_router(multilingual_router, tags=["multilingual"])