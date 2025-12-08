"""Pydantic models for personalization features"""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field


class PersonalizationPreferences(BaseModel):
    """Model for user personalization preferences"""
    persona: Optional[str] = Field(None, regex=r'^(beginner|intermediate|advanced|researcher|developer|student)$')
    experience_level: Optional[str] = Field(None, regex=r'^(beginner|intermediate|advanced)$')
    domains_of_interest: Optional[List[str]] = Field(None, max_items=5)
    preferred_formats: Optional[List[str]] = Field(None, max_items=10)
    complexity_preference: Optional[int] = Field(None, ge=1, le=3)
    language_preference: Optional[str] = Field(None, regex=r'^[a-z]{2}$')
    learning_style: Optional[str] = Field(None, regex=r'^(visual|auditory|kinesthetic|reading)$')
    
    class Config:
        schema_extra = {
            "example": {
                "persona": "developer",
                "experience_level": "intermediate",
                "domains_of_interest": ["web development", "ai", "data science"],
                "preferred_formats": ["code examples", "diagrams", "step-by-step"],
                "complexity_preference": 2,
                "language_preference": "en",
                "learning_style": "visual"
            }
        }


class PersonalizedContent(BaseModel):
    """Model for personalized content response"""
    original_content: str
    personalized_content: str
    persona_applied: str
    experience_level: str
    suggestions: List[str]
    reading_time: Optional[int]
    word_count: Optional[int]
    personalization_score: Optional[float] = Field(None, ge=0, le=1)
    
    class Config:
        schema_extra = {
            "example": {
                "original_content": "This chapter covers basic algorithms...",
                "personalized_content": "This chapter covers basic algorithms... 💻 Developer Note: Consider the time complexity...",
                "persona_applied": "developer",
                "experience_level": "intermediate",
                "suggestions": [
                    "Try implementing the concepts in your preferred language",
                    "Look for open-source projects demonstrating these principles"
                ],
                "reading_time": 15,
                "word_count": 2500,
                "personalization_score": 0.85
            }
        }


class ContentRecommendation(BaseModel):
    """Model for content recommendations"""
    chapter_id: int
    title: str
    relevance_score: float = Field(..., ge=0, le=1)
    reason: str
    estimated_reading_time: Optional[int]
    difficulty_level: Optional[str]
    
    class Config:
        schema_extra = {
            "example": {
                "chapter_id": 123,
                "title": "Advanced Data Structures",
                "relevance_score": 0.92,
                "reason": "Matches your developer persona and intermediate experience level",
                "estimated_reading_time": 20,
                "difficulty_level": "intermediate"
            }
        }


class LearningPath(BaseModel):
    """Model for personalized learning path"""
    path_id: str
    title: str
    description: str
    chapters: List[ContentRecommendation]
    estimated_total_time: int
    difficulty_progression: str
    
    class Config:
        schema_extra = {
            "example": {
                "path_id": "dev-path-001",
                "title": "Web Developer's Algorithm Journey",
                "description": "A curated path for web developers to learn algorithms",
                "chapters": [],
                "estimated_total_time": 180,
                "difficulty_progression": "beginner -> intermediate -> advanced"
            }
        }


class PersonalizationAnalytics(BaseModel):
    """Model for personalization analytics"""
    user_id: int
    total_content_viewed: int
    personalized_content_viewed: int
    average_engagement_score: float
    preferred_content_types: List[str]
    learning_velocity: float  # chapters per week
    completion_rate: float
    
    class Config:
        schema_extra = {
            "example": {
                "user_id": 123,
                "total_content_viewed": 45,
                "personalized_content_viewed": 38,
                "average_engagement_score": 0.78,
                "preferred_content_types": ["code examples", "visual diagrams"],
                "learning_velocity": 2.5,
                "completion_rate": 0.84
            }
        }


class ContentFeedback(BaseModel):
    """Model for content feedback"""
    chapter_id: int
    user_id: int
    rating: int = Field(..., ge=1, le=5)
    feedback_type: str = Field(..., regex=r'^(helpful|confusing|too_easy|too_hard|just_right)$')
    comment: Optional[str] = Field(None, max_length=500)
    personalization_helpful: bool
    
    class Config:
        schema_extra = {
            "example": {
                "chapter_id": 123,
                "user_id": 456,
                "rating": 4,
                "feedback_type": "helpful",
                "comment": "The developer notes were very useful",
                "personalization_helpful": True
            }
        }


class AdaptiveLearning(BaseModel):
    """Model for adaptive learning adjustments"""
    user_id: int
    current_difficulty: str
    suggested_difficulty: str
    confidence_score: float
    areas_of_strength: List[str]
    areas_for_improvement: List[str]
    next_recommended_topics: List[str]
    
    class Config:
        schema_extra = {
            "example": {
                "user_id": 123,
                "current_difficulty": "intermediate",
                "suggested_difficulty": "intermediate",
                "confidence_score": 0.75,
                "areas_of_strength": ["algorithms", "data structures"],
                "areas_for_improvement": ["system design", "optimization"],
                "next_recommended_topics": ["advanced algorithms", "distributed systems"]
            }
        }