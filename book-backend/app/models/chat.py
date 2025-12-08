"""Chat and chatbot related Pydantic models"""

from typing import Optional, List, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field


# Pydantic models for API requests/responses
class ChatRequest(BaseModel):
    """Request model for chatbot queries"""
    session_id: int = Field(..., description="Chat session ID")
    message: str = Field(..., min_length=1, max_length=2000, description="User message")
    context: Optional[str] = Field(None, max_length=5000, description="Optional context text")
    use_rag: bool = Field(True, description="Whether to use RAG for enhanced responses")
    context_book_ids: Optional[List[int]] = Field(None, description="Specific book IDs for context")


class ChatResponse(BaseModel):
    """Response model for chatbot queries"""
    message: str = Field(..., description="Chatbot response")
    session_id: int = Field(..., description="Chat session ID")
    message_id: int = Field(..., description="Message ID in database")
    context_used: List[str] = Field(default_factory=list, description="Sources used for response")
    confidence_score: float = Field(0.0, ge=0.0, le=1.0, description="Response confidence score")
    suggestions: List[str] = Field(default_factory=list, description="Follow-up suggestions")
    processing_time: float = Field(0.0, description="Processing time in seconds")


class ChatSessionCreate(BaseModel):
    """Request model for creating chat sessions"""
    title: Optional[str] = Field(None, max_length=200, description="Session title")
    session_type: str = Field("general", description="Type of chat session")
    language: str = Field("en", description="Language preference")
    context_book_id: Optional[int] = Field(None, description="Book ID for context")


class ChatSessionUpdate(BaseModel):
    """Request model for updating chat sessions"""
    title: Optional[str] = Field(None, max_length=200, description="New session title")


class ChatSessionResponse(BaseModel):
    """Response model for chat sessions"""
    id: int
    user_id: Optional[int]
    title: str
    session_type: str
    language: str
    context_book_id: Optional[int]
    created_at: datetime
    updated_at: Optional[datetime]
    
    class Config:
        from_attributes = True


class ChatMessageResponse(BaseModel):
    """Response model for chat messages"""
    id: int
    session_id: int
    content: str
    sender: str
    message_type: str
    language: str
    context_used: Optional[List[str]]
    confidence_score: Optional[float]
    created_at: datetime
    
    class Config:
        from_attributes = True


# SQLAlchemy models are defined in app.models.database