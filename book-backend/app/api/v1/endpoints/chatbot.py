"""Chatbot endpoints for RAG-powered conversations"""

from typing import List, Optional, Any
from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.services.chatbot_service import ChatbotService
from app.models.chat import (
    ChatRequest, ChatResponse, ChatSessionCreate, ChatSessionResponse,
    ChatMessageResponse, ChatSessionUpdate
)
from app.models.user import User
from app.api.dependencies import get_current_user, get_optional_current_user
import logging

logger = logging.getLogger(__name__)

router = APIRouter()


# Chat session endpoints
@router.post("/sessions", response_model=ChatSessionResponse, status_code=status.HTTP_201_CREATED)
def create_chat_session(
    session_data: ChatSessionCreate,
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """Create a new chat session"""
    try:
        chatbot_service = ChatbotService(db)
        
        session = chatbot_service.create_chat_session(
            user_id=current_user.id if current_user else None,
            title=session_data.title,
            session_type=session_data.session_type,
            language=session_data.language,
            context_book_id=session_data.context_book_id
        )
        
        return session
        
    except Exception as e:
        logger.error(f"Error creating chat session: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create chat session"
        )


@router.get("/sessions", response_model=List[ChatSessionResponse])
def get_user_chat_sessions(
    skip: int = Query(0, ge=0),
    limit: int = Query(20, ge=1, le=100),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Get user's chat sessions"""
    try:
        chatbot_service = ChatbotService(db)
        sessions = chatbot_service.get_user_chat_sessions(
            user_id=current_user.id,
            limit=limit
        )
        
        return sessions
        
    except Exception as e:
        logger.error(f"Error getting chat sessions: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get chat sessions"
        )


@router.get("/sessions/{session_id}", response_model=ChatSessionResponse)
def get_chat_session(
    session_id: int,
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """Get a specific chat session"""
    try:
        chatbot_service = ChatbotService(db)
        session = chatbot_service.get_chat_session(session_id)
        
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chat session not found"
            )
        
        # Check permissions for user sessions
        if session.user_id and (not current_user or session.user_id != current_user.id):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        return session
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting chat session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get chat session"
        )


@router.get("/sessions/{session_id}/messages", response_model=List[ChatMessageResponse])
def get_session_messages(
    session_id: int,
    skip: int = Query(0, ge=0),
    limit: int = Query(50, ge=1, le=100),
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """Get messages from a chat session"""
    try:
        chatbot_service = ChatbotService(db)
        
        # Check session exists and permissions
        session = chatbot_service.get_chat_session(session_id)
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chat session not found"
            )
        
        if session.user_id and (not current_user or session.user_id != current_user.id):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        messages = chatbot_service.get_session_messages(
            session_id=session_id,
            limit=limit
        )
        
        return messages
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting messages for session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get session messages"
        )


# Main chatbot endpoints
@router.post("/query", response_model=ChatResponse)
def chat_query(
    request: ChatRequest,
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """
    Main chatbot query endpoint with RAG capabilities
    
    This endpoint processes user questions about book content and returns
    intelligent responses using RAG (Retrieval-Augmented Generation).
    """
    try:
        chatbot_service = ChatbotService(db)
        
        # Process the chat message
        result = chatbot_service.chat_with_rag(
            session_id=request.session_id,
            user_message=request.message,
            user_id=current_user.id if current_user else None,
            use_rag=request.use_rag,
            context_book_ids=request.context_book_ids
        )
        
        return ChatResponse(
            message=result["response"],
            session_id=result["session_id"],
            message_id=result["assistant_message_id"],
            context_used=result["context_used"],
            confidence_score=result["confidence_score"],
            suggestions=chatbot_service.get_chat_suggestions(
                session_id=request.session_id,
                context_book_id=request.context_book_ids[0] if request.context_book_ids else None
            ),
            processing_time=result["processing_time"]
        )
        
    except ValueError as e:
        logger.error(f"Validation error in chat query: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error processing chat query: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process chat query"
        )


@router.post("/context", response_model=ChatResponse)
def chat_with_context(
    request: ChatRequest,
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """
    Chatbot endpoint with explicit context provision
    
    This endpoint allows users to provide specific text context
    to guide the chatbot's response more precisely.
    """
    try:
        chatbot_service = ChatbotService(db)
        
        # Enhanced message with context
        enhanced_message = request.message
        if request.context:
            enhanced_message = f"""
Context: {request.context}

Question: {request.message}

Please answer the question based on the provided context and any relevant book content you can find.
"""
        
        # Process the chat message with enhanced context
        result = chatbot_service.chat_with_rag(
            session_id=request.session_id,
            user_message=enhanced_message,
            user_id=current_user.id if current_user else None,
            use_rag=True,  # Always use RAG for context-based queries
            context_book_ids=request.context_book_ids
        )
        
        return ChatResponse(
            message=result["response"],
            session_id=result["session_id"],
            message_id=result["assistant_message_id"],
            context_used=result["context_used"],
            confidence_score=result["confidence_score"],
            suggestions=chatbot_service.get_chat_suggestions(
                session_id=request.session_id,
                context_book_id=request.context_book_ids[0] if request.context_book_ids else None
            ),
            processing_time=result["processing_time"]
        )
        
    except ValueError as e:
        logger.error(f"Validation error in context chat: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error processing context chat: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process context chat"
        )


@router.delete("/sessions/{session_id}")
def delete_chat_session(
    session_id: int,
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """Delete a chat session"""
    try:
        chatbot_service = ChatbotService(db)
        
        success = chatbot_service.delete_chat_session(
            session_id=session_id,
            user_id=current_user.id if current_user else None
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chat session not found or access denied"
            )
        
        return {"message": "Chat session deleted successfully"}
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting chat session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete chat session"
        )


@router.get("/suggestions/{session_id}", response_model=List[str])
def get_chat_suggestions(
    session_id: int,
    context_book_id: Optional[int] = Query(None),
    current_user: Optional[User] = Depends(get_optional_current_user),
    db: Session = Depends(get_db)
):
    """Get chat suggestions for a session"""
    try:
        chatbot_service = ChatbotService(db)
        
        # Check session exists and permissions
        session = chatbot_service.get_chat_session(session_id)
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chat session not found"
            )
        
        if session.user_id and (not current_user or session.user_id != current_user.id):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not enough permissions"
            )
        
        suggestions = chatbot_service.get_chat_suggestions(
            session_id=session_id,
            context_book_id=context_book_id
        )
        
        return suggestions
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting suggestions for session {session_id}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get chat suggestions"
        )