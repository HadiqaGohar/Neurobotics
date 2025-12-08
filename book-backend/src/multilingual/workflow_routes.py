"""API routes for translation workflow management."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.orm import Session
from pydantic import BaseModel

from app.core.database import get_db
from app.core.security import get_current_user
from app.models.database import User
from .workflow import translation_workflow, Priority, WorkflowStatus

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/workflow", tags=["translation-workflow"])


# Pydantic models for request/response
class TranslationJobCreate(BaseModel):
    content_type: str
    content_id: str
    source_language: str = "en"
    target_language: str
    title: Optional[str] = None
    content: Optional[str] = None
    priority: Priority = Priority.MEDIUM
    deadline: Optional[datetime] = None
    translator_id: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class TranslationJobAssign(BaseModel):
    translator_id: int


class TranslationProgressUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    quality_score: Optional[float] = None
    progress_percentage: Optional[int] = None
    notes: Optional[str] = None
    status: Optional[str] = None


class TranslationReview(BaseModel):
    approved: bool
    feedback: Optional[str] = None
    quality_score: Optional[float] = None


@router.post("/jobs", response_model=Dict[str, Any])
async def create_translation_job(
    job_data: TranslationJobCreate,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Create a new translation job."""
    try:
        job_id = await translation_workflow.create_translation_job(
            content_type=job_data.content_type,
            content_id=job_data.content_id,
            source_language=job_data.source_language,
            target_language=job_data.target_language,
            title=job_data.title,
            content=job_data.content,
            priority=job_data.priority,
            deadline=job_data.deadline,
            translator_id=job_data.translator_id,
            metadata=job_data.metadata,
            db=db
        )
        
        if job_id:
            return {
                "success": True,
                "job_id": job_id,
                "message": "Translation job created successfully"
            }
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to create translation job"
            )
            
    except Exception as e:
        logger.error(f"Error creating translation job: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/jobs/{job_id}/assign", response_model=Dict[str, str])
async def assign_translator(
    job_id: int,
    assignment_data: TranslationJobAssign,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Assign a translator to a job."""
    try:
        success = await translation_workflow.assign_translator(
            job_id=job_id,
            translator_id=assignment_data.translator_id,
            assigned_by=current_user.id,
            db=db
        )
        
        if success:
            return {"message": "Translator assigned successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to assign translator"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error assigning translator: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.put("/jobs/{job_id}/progress", response_model=Dict[str, str])
async def update_translation_progress(
    job_id: int,
    progress_data: TranslationProgressUpdate,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Update translation progress."""
    try:
        success = await translation_workflow.update_translation_progress(
            job_id=job_id,
            progress_data=progress_data.dict(exclude_unset=True),
            db=db
        )
        
        if success:
            return {"message": "Translation progress updated successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to update translation progress"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating translation progress: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/jobs/{job_id}/submit-review", response_model=Dict[str, str])
async def submit_for_review(
    job_id: int,
    reviewer_id: Optional[int] = None,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Submit translation for review."""
    try:
        success = await translation_workflow.submit_for_review(
            job_id=job_id,
            reviewer_id=reviewer_id,
            db=db
        )
        
        if success:
            return {"message": "Translation submitted for review successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to submit translation for review"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error submitting for review: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/jobs/{job_id}/review", response_model=Dict[str, str])
async def review_translation(
    job_id: int,
    review_data: TranslationReview,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Review and approve/reject translation."""
    try:
        success = await translation_workflow.review_translation(
            job_id=job_id,
            reviewer_id=current_user.id,
            approved=review_data.approved,
            feedback=review_data.feedback,
            quality_score=review_data.quality_score,
            db=db
        )
        
        if success:
            status_msg = "approved" if review_data.approved else "rejected"
            return {"message": f"Translation {status_msg} successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to review translation"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error reviewing translation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.post("/jobs/{job_id}/publish", response_model=Dict[str, str])
async def publish_translation(
    job_id: int,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Publish approved translation."""
    try:
        success = await translation_workflow.publish_translation(
            job_id=job_id,
            published_by=current_user.id,
            db=db
        )
        
        if success:
            return {"message": "Translation published successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to publish translation"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error publishing translation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/jobs/pending", response_model=List[Dict[str, Any]])
async def get_pending_jobs(
    translator_id: Optional[int] = Query(None),
    language_code: Optional[str] = Query(None),
    priority: Optional[Priority] = Query(None),
    limit: int = Query(50, le=100),
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Get pending translation jobs."""
    try:
        jobs = await translation_workflow.get_pending_jobs(
            translator_id=translator_id,
            language_code=language_code,
            priority=priority,
            limit=limit,
            db=db
        )
        
        return jobs
        
    except Exception as e:
        logger.error(f"Error getting pending jobs: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/statistics", response_model=Dict[str, Any])
async def get_workflow_statistics(
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Get workflow statistics."""
    try:
        stats = await translation_workflow.get_workflow_statistics(db)
        return stats
        
    except Exception as e:
        logger.error(f"Error getting workflow statistics: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )


@router.get("/jobs/{job_id}", response_model=Dict[str, Any])
async def get_translation_job(
    job_id: int,
    current_user: User = Depends(get_current_user()),
    db: Session = Depends(get_db)
):
    """Get translation job details."""
    try:
        from app.models.multilingual import ContentTranslation
        
        job = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
        
        if not job:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Translation job not found"
            )
        
        return {
            "id": job.id,
            "content_type": job.content_type,
            "content_id": job.content_id,
            "language_code": job.language_code,
            "title": job.title,
            "content": job.content,
            "status": job.translation_status.value,
            "method": job.translation_method.value,
            "translator_id": job.translator_id,
            "reviewer_id": job.reviewer_id,
            "quality_score": job.quality_score,
            "version": job.version,
            "metadata": job.metadata,
            "created_at": job.created_at.isoformat(),
            "updated_at": job.updated_at.isoformat()
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting translation job: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error"
        )