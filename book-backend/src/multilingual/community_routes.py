"""Community translation API routes."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .community_translation import (
    community_translation, 
    ContributionType, 
    ContributionStatus,
    ReputationAction
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/community", tags=["community-translation"])


# Pydantic models for request/response
class TranslationContributionRequest(BaseModel):
    content_type: str = Field(..., description="Type of content (e.g., 'chapter', 'article')")
    content_id: str = Field(..., description="ID of the content")
    language_code: str = Field(..., description="Target language code")
    title: Optional[str] = Field(None, description="Translated title")
    content: Optional[str] = Field(None, description="Translated content")
    notes: Optional[str] = Field(None, description="Notes for reviewers")


class ReviewSubmissionRequest(BaseModel):
    rating: int = Field(..., ge=1, le=5, description="Rating from 1 to 5 stars")
    feedback: str = Field(..., description="Review feedback")
    suggested_improvements: Optional[Dict[str, str]] = Field(None, description="Suggested improvements")


class CorrectionSubmissionRequest(BaseModel):
    corrections: Dict[str, str] = Field(..., description="Corrections to apply")
    explanation: str = Field(..., description="Explanation for corrections")


class TerminologySubmissionRequest(BaseModel):
    term: str = Field(..., description="Original term")
    translation: str = Field(..., description="Translated term")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(..., description="Target language code")
    definition: Optional[str] = Field(None, description="Term definition")
    context: Optional[str] = Field(None, description="Usage context")
    domain: str = Field("technical", description="Domain/category")


class VoteRequest(BaseModel):
    contribution_type: str = Field(..., description="Type of contribution")
    vote: str = Field(..., regex="^(upvote|downvote)$", description="Vote type")


class ContributionResponse(BaseModel):
    success: bool
    contribution_id: Optional[int] = None
    status: Optional[str] = None
    message: Optional[str] = None
    error: Optional[str] = None


@router.post("/translations", response_model=ContributionResponse)
async def submit_translation(
    request: TranslationContributionRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Submit a translation contribution."""
    try:
        result = await community_translation.submit_translation_contribution(
            content_type=request.content_type,
            content_id=request.content_id,
            language_code=request.language_code,
            title=request.title,
            content=request.content,
            contributor_id=current_user.id,
            notes=request.notes,
            db=db
        )
        
        return ContributionResponse(**result)
        
    except Exception as e:
        logger.error(f"Error in submit_translation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit translation"
        )


@router.post("/translations/{translation_id}/reviews", response_model=ContributionResponse)
async def submit_review(
    translation_id: int,
    request: ReviewSubmissionRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Submit a review for a translation."""
    try:
        result = await community_translation.submit_review(
            translation_id=translation_id,
            reviewer_id=current_user.id,
            rating=request.rating,
            feedback=request.feedback,
            suggested_improvements=request.suggested_improvements,
            db=db
        )
        
        return ContributionResponse(**result)
        
    except Exception as e:
        logger.error(f"Error in submit_review: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit review"
        )


@router.post("/translations/{translation_id}/corrections", response_model=ContributionResponse)
async def submit_correction(
    translation_id: int,
    request: CorrectionSubmissionRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Submit corrections for a translation."""
    try:
        result = await community_translation.submit_correction(
            translation_id=translation_id,
            corrector_id=current_user.id,
            corrections=request.corrections,
            explanation=request.explanation,
            db=db
        )
        
        return ContributionResponse(**result)
        
    except Exception as e:
        logger.error(f"Error in submit_correction: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit correction"
        )


@router.post("/terminology", response_model=ContributionResponse)
async def submit_terminology(
    request: TerminologySubmissionRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Submit terminology contribution."""
    try:
        result = await community_translation.submit_terminology(
            term=request.term,
            translation=request.translation,
            source_language=request.source_language,
            target_language=request.target_language,
            definition=request.definition,
            context=request.context,
            domain=request.domain,
            contributor_id=current_user.id,
            db=db
        )
        
        return ContributionResponse(**result)
        
    except Exception as e:
        logger.error(f"Error in submit_terminology: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit terminology"
        )


@router.post("/contributions/{contribution_id}/vote", response_model=ContributionResponse)
async def vote_on_contribution(
    contribution_id: int,
    request: VoteRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Vote on a community contribution."""
    try:
        contribution_type = ContributionType(request.contribution_type)
        
        result = await community_translation.vote_on_contribution(
            contribution_id=contribution_id,
            contribution_type=contribution_type,
            voter_id=current_user.id,
            vote=request.vote,
            db=db
        )
        
        return ContributionResponse(**result)
        
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid contribution type: {request.contribution_type}"
        )
    except Exception as e:
        logger.error(f"Error in vote_on_contribution: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to vote on contribution"
        )


@router.get("/users/{user_id}/contributions")
async def get_user_contributions(
    user_id: int,
    contribution_type: Optional[str] = Query(None, description="Filter by contribution type"),
    limit: int = Query(50, ge=1, le=100, description="Number of contributions to return"),
    offset: int = Query(0, ge=0, description="Offset for pagination"),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Get user's contributions."""
    try:
        # Check if user can view contributions (own contributions or admin)
        if user_id != current_user.id and not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to view these contributions"
            )
        
        contribution_type_enum = None
        if contribution_type:
            try:
                contribution_type_enum = ContributionType(contribution_type)
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Invalid contribution type: {contribution_type}"
                )
        
        result = await community_translation.get_user_contributions(
            user_id=user_id,
            contribution_type=contribution_type_enum,
            limit=limit,
            offset=offset,
            db=db
        )
        
        return result
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in get_user_contributions: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get user contributions"
        )


@router.get("/content/{content_type}/{content_id}/contributions")
async def get_content_contributions(
    content_type: str,
    content_id: str,
    language_code: str = Query(..., description="Language code to filter contributions"),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Get contributions for specific content."""
    try:
        # This is a simplified version - in a real implementation,
        # you'd query the ContentTranslation table for the specific content
        from app.models.multilingual import ContentTranslation
        from sqlalchemy import and_
        
        translations = db.query(ContentTranslation).filter(
            and_(
                ContentTranslation.content_type == content_type,
                ContentTranslation.content_id == content_id,
                ContentTranslation.language_code == language_code
            )
        ).all()
        
        contributions = []
        for translation in translations:
            metadata = translation.metadata or {}
            if metadata.get("community_contribution"):
                contributions.append({
                    "id": translation.id,
                    "type": "translation",
                    "title": translation.title,
                    "content": translation.content,
                    "status": translation.translation_status.value,
                    "created_at": translation.created_at.isoformat(),
                    "updated_at": translation.updated_at.isoformat(),
                    "votes": {
                        "upvotes": metadata.get("upvotes", 0),
                        "downvotes": metadata.get("downvotes", 0),
                        "vote_score": metadata.get("vote_score", 0)
                    },
                    "reviews": metadata.get("review_count", 0),
                    "average_rating": metadata.get("average_rating", 0)
                })
        
        return {
            "contributions": contributions,
            "total_count": len(contributions)
        }
        
    except Exception as e:
        logger.error(f"Error in get_content_contributions: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get content contributions"
        )


@router.get("/users/{user_id}/stats")
async def get_user_stats(
    user_id: int,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Get user statistics."""
    try:
        # Check if user can view stats (own stats or admin)
        if user_id != current_user.id and not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to view these statistics"
            )
        
        stats = await community_translation._get_user_stats(user_id, db)
        return stats
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in get_user_stats: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get user statistics"
        )


@router.get("/leaderboard")
async def get_leaderboard(
    period: str = Query("all_time", regex="^(week|month|year|all_time)$"),
    limit: int = Query(20, ge=1, le=100),
    db: Session = Depends(get_db)
):
    """Get community leaderboard."""
    try:
        result = await community_translation.get_leaderboard(
            period=period,
            limit=limit,
            db=db
        )
        
        return result
        
    except Exception as e:
        logger.error(f"Error in get_leaderboard: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get leaderboard"
        )


@router.get("/quality-metrics")
async def get_quality_metrics(
    content_type: Optional[str] = Query(None, description="Filter by content type"),
    language_code: Optional[str] = Query(None, description="Filter by language"),
    db: Session = Depends(get_db)
):
    """Get translation quality metrics."""
    try:
        from app.models.multilingual import ContentTranslation, TranslationStatus
        from sqlalchemy import func, and_
        
        # Base query
        query = db.query(ContentTranslation)
        
        # Apply filters
        filters = []
        if content_type:
            filters.append(ContentTranslation.content_type == content_type)
        if language_code:
            filters.append(ContentTranslation.language_code == language_code)
        
        if filters:
            query = query.filter(and_(*filters))
        
        # Calculate metrics
        total_translations = query.count()
        
        approved_translations = query.filter(
            ContentTranslation.translation_status == TranslationStatus.APPROVED
        ).count()
        
        quality_stats = query.filter(
            ContentTranslation.quality_score.isnot(None)
        ).with_entities(
            func.avg(ContentTranslation.quality_score),
            func.count(ContentTranslation.id)
        ).first()
        
        # Count active contributors
        active_contributors = query.filter(
            ContentTranslation.translator_id.isnot(None)
        ).with_entities(
            func.count(func.distinct(ContentTranslation.translator_id))
        ).scalar()
        
        return {
            "total_translations": total_translations,
            "approved_translations": approved_translations,
            "approval_rate": (approved_translations / max(total_translations, 1)) * 100,
            "average_quality": float(quality_stats[0]) if quality_stats[0] else None,
            "quality_sample_size": quality_stats[1] if quality_stats[1] else 0,
            "active_contributors": active_contributors or 0
        }
        
    except Exception as e:
        logger.error(f"Error in get_quality_metrics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get quality metrics"
        )


@router.get("/guidelines/{language_code}")
async def get_contribution_guidelines(
    language_code: str,
    db: Session = Depends(get_db)
):
    """Get contribution guidelines for a specific language."""
    try:
        # This would typically come from a database or configuration
        # For now, return static guidelines
        guidelines = {
            "language_code": language_code,
            "guidelines": {
                "translation": [
                    "Maintain the original meaning and context",
                    "Use appropriate technical terminology",
                    "Follow language-specific formatting rules",
                    "Ensure cultural appropriateness"
                ],
                "review": [
                    "Check for accuracy and completeness",
                    "Verify technical terminology usage",
                    "Assess readability and flow",
                    "Provide constructive feedback"
                ],
                "terminology": [
                    "Provide clear definitions",
                    "Include usage context",
                    "Specify the domain/field",
                    "Ensure consistency with existing terms"
                ]
            },
            "quality_standards": {
                "minimum_rating": 3.0,
                "required_reviews": 2,
                "approval_threshold": 4.0
            },
            "reputation_system": {
                "translation_submitted": 5,
                "translation_approved": 20,
                "review_submitted": 3,
                "terminology_added": 15
            }
        }
        
        return guidelines
        
    except Exception as e:
        logger.error(f"Error in get_contribution_guidelines: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get contribution guidelines"
        )


@router.post("/contributions/{contribution_id}/report")
async def report_content(
    contribution_id: int,
    contribution_type: str,
    reason: str,
    description: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Report inappropriate content."""
    try:
        # In a real implementation, this would create a report record
        # and potentially flag the content for moderation
        
        report_data = {
            "contribution_id": contribution_id,
            "contribution_type": contribution_type,
            "reason": reason,
            "description": description,
            "reporter_id": current_user.id,
            "reported_at": datetime.utcnow().isoformat(),
            "status": "pending"
        }
        
        # Log the report for now
        logger.info(f"Content report submitted: {report_data}")
        
        return {
            "success": True,
            "message": "Report submitted successfully",
            "report_id": f"RPT_{contribution_id}_{current_user.id}"
        }
        
    except Exception as e:
        logger.error(f"Error in report_content: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit report"
        )


@router.get("/users/{user_id}/reputation-history")
async def get_reputation_history(
    user_id: int,
    limit: int = Query(50, ge=1, le=100),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Get user's reputation history."""
    try:
        # Check if user can view reputation history
        if user_id != current_user.id and not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to view this reputation history"
            )
        
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
        
        metadata = getattr(user, 'metadata', None) or {}
        reputation_history = metadata.get("reputation_history", [])
        
        # Sort by timestamp (most recent first) and limit
        reputation_history.sort(key=lambda x: x.get("timestamp", ""), reverse=True)
        
        return {
            "history": reputation_history[:limit],
            "total_count": len(reputation_history)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in get_reputation_history: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get reputation history"
        )


@router.get("/tasks")
async def get_available_tasks(
    language_code: Optional[str] = Query(None, description="Filter by language"),
    priority: Optional[str] = Query(None, description="Filter by priority"),
    limit: int = Query(20, ge=1, le=100),
    db: Session = Depends(get_db)
):
    """Get available translation tasks."""
    try:
        from app.models.multilingual import ContentTranslation, TranslationStatus
        from sqlalchemy import and_
        
        # Query for pending translations that need work
        query = db.query(ContentTranslation).filter(
            ContentTranslation.translation_status.in_([
                TranslationStatus.PENDING,
                TranslationStatus.IN_PROGRESS
            ])
        )
        
        # Apply filters
        if language_code:
            query = query.filter(ContentTranslation.language_code == language_code)
        
        if priority:
            query = query.filter(
                ContentTranslation.metadata.op('->>')('priority') == priority
            )
        
        tasks = query.limit(limit).all()
        
        task_list = []
        for task in tasks:
            metadata = task.metadata or {}
            task_list.append({
                "id": task.id,
                "content_type": task.content_type,
                "content_id": task.content_id,
                "language_code": task.language_code,
                "title": task.title,
                "status": task.translation_status.value,
                "priority": metadata.get("priority", "medium"),
                "deadline": metadata.get("deadline"),
                "created_at": task.created_at.isoformat(),
                "estimated_effort": metadata.get("estimated_effort", "medium"),
                "domain": metadata.get("domain", "general")
            })
        
        return {
            "tasks": task_list,
            "total_count": len(task_list)
        }
        
    except Exception as e:
        logger.error(f"Error in get_available_tasks: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get available tasks"
        )