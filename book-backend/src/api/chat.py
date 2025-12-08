from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from typing import List, Optional
import uuid
import logging
from pydantic import BaseModel, validator

from src.database.database import get_db
from src.database import crud
from src.services.rag_service import RAGService
from src.services.embedding_service import EmbeddingService
from src.security.security_utils import SecurityValidator, rate_limiter, InputValidator
from qdrant_client import QdrantClient
import os

logger = logging.getLogger(__name__)

router = APIRouter()

# Initialize Qdrant client and EmbeddingService
def get_qdrant_client() -> QdrantClient:
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variable not set")
    return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

def get_embedding_service() -> EmbeddingService:
    return EmbeddingService() # EmbeddingService loads model internally

def get_rag_service(
    qdrant_client: QdrantClient = Depends(get_qdrant_client),
    embedding_service: EmbeddingService = Depends(get_embedding_service)
) -> RAGService:
    return RAGService(qdrant_client, embedding_service)


class ChatRequest(BaseModel):
    message: str
    session_id: Optional[uuid.UUID] = None
    
    @validator('message')
    def validate_message(cls, v):
        if not v or not v.strip():
            raise ValueError('Message cannot be empty')
        if not InputValidator.validate_message_length(v):
            raise ValueError('Message too long')
        return v.strip()

class ChatResponse(BaseModel):
    response: str
    session_id: uuid.UUID
    context: Optional[List[str]] = None

@router.post("/chat/rag", response_model=ChatResponse)
async def chat_rag_endpoint(
    request: ChatRequest,
    http_request: Request,
    db: Session = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
):
    try:
        # Rate limiting
        client_ip = http_request.client.host
        if not rate_limiter.is_allowed(client_ip, max_requests=50, window_minutes=60):
            raise HTTPException(status_code=429, detail="Rate limit exceeded")
        
        # Validate session
        session_id = request.session_id
        if session_id is None:
            # Create a new session if not provided
            try:
                db_session = crud.create_session(db)
                session_id = db_session.id
            except Exception as e:
                logger.error(f"Failed to create session: {e}")
                raise HTTPException(status_code=500, detail="Failed to create session")
        else:
            # Validate session ID format
            if not InputValidator.validate_session_id(str(session_id)):
                raise HTTPException(status_code=400, detail="Invalid session ID format")
            
            db_session = crud.get_session(db, session_id)
            if not db_session:
                raise HTTPException(status_code=404, detail="Session not found")

        # Retrieve context from RAG service
        try:
            retrieved_context = await rag_service.retrieve_context(request.message)
        except Exception as e:
            logger.error(f"Context retrieval failed: {e}")
            retrieved_context = []

        # Generate RAG-enhanced response
        try:
            rag_response = await rag_service.generate_rag_response(request.message, retrieved_context)
        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            rag_response = "I'm sorry, but I encountered an error while processing your request."

        # Store user query and RAG response in messages table
        try:
            crud.create_message(db, session_id, request.message, "user")
            crud.create_message(db, session_id, rag_response, "ai", {"context": retrieved_context})
        except Exception as e:
            logger.error(f"Failed to store messages: {e}")
            # Continue anyway, don't fail the request

        return ChatResponse(
            response=rag_response, 
            session_id=session_id, 
            context=retrieved_context[:3]  # Limit context in response
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.post("/chat/history")
async def get_chat_history_endpoint(
    session_id: uuid.UUID,
    db: Session = Depends(get_db)
):
    messages = crud.get_messages_by_session_id(db, session_id)
    if not messages:
        raise HTTPException(status_code=404, detail="Session not found or no messages")
    return messages
