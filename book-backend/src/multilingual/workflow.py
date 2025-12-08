"""Translation workflow management system."""

import logging
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
import json

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_

from app.models.multilingual import (
    ContentTranslation, TranslationStatus, TranslationMethod,
    Language, TranslationMemory, TerminologyGlossary
)
from app.models.database import User

logger = logging.getLogger(__name__)


class WorkflowStatus(str, Enum):
    """Translation workflow status."""
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    REVIEW = "review"
    APPROVED = "approved"
    REJECTED = "rejected"
    PUBLISHED = "published"


class Priority(str, Enum):
    """Translation priority levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    URGENT = "urgent"


class TranslationWorkflow:
    """Translation workflow management."""
    
    def __init__(self):
        self.workflow_rules = {
            WorkflowStatus.PENDING: [WorkflowStatus.ASSIGNED, WorkflowStatus.IN_PROGRESS],
            WorkflowStatus.ASSIGNED: [WorkflowStatus.IN_PROGRESS, WorkflowStatus.PENDING],
            WorkflowStatus.IN_PROGRESS: [WorkflowStatus.REVIEW, WorkflowStatus.ASSIGNED],
            WorkflowStatus.REVIEW: [WorkflowStatus.APPROVED, WorkflowStatus.REJECTED, WorkflowStatus.IN_PROGRESS],
            WorkflowStatus.APPROVED: [WorkflowStatus.PUBLISHED, WorkflowStatus.REVIEW],
            WorkflowStatus.REJECTED: [WorkflowStatus.IN_PROGRESS, WorkflowStatus.ASSIGNED],
            WorkflowStatus.PUBLISHED: []  # Final state
        }
    
    async def create_translation_job(
        self,
        content_type: str,
        content_id: str,
        source_language: str,
        target_language: str,
        title: Optional[str] = None,
        content: Optional[str] = None,
        priority: Priority = Priority.MEDIUM,
        deadline: Optional[datetime] = None,
        translator_id: Optional[int] = None,
        metadata: Optional[Dict[str, Any]] = None,
        db: Session = None
    ) -> Optional[int]:
        """Create a new translation job."""
        try:
            # Check if translation already exists
            existing = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.content_type == content_type,
                    ContentTranslation.content_id == content_id,
                    ContentTranslation.language_code == target_language
                )
            ).first()
            
            if existing:
                logger.warning(f"Translation already exists for {content_type}:{content_id} in {target_language}")
                return existing.id
            
            # Create translation job
            job_metadata = metadata or {}
            job_metadata.update({
                "priority": priority.value,
                "deadline": deadline.isoformat() if deadline else None,
                "created_by": "system",
                "workflow_status": WorkflowStatus.PENDING.value,
                "source_language": source_language
            })
            
            translation = ContentTranslation(
                content_type=content_type,
                content_id=content_id,
                language_code=target_language,
                title=title,
                content=content,
                translation_method=TranslationMethod.MANUAL if translator_id else TranslationMethod.AI,
                translator_id=translator_id,
                metadata=job_metadata,
                translation_status=TranslationStatus.PENDING
            )
            
            db.add(translation)
            db.commit()
            db.refresh(translation)
            
            logger.info(f"Created translation job {translation.id} for {content_type}:{content_id}")
            return translation.id
            
        except Exception as e:
            logger.error(f"Error creating translation job: {e}")
            db.rollback()
            return None
    
    async def assign_translator(
        self,
        job_id: int,
        translator_id: int,
        assigned_by: int,
        db: Session
    ) -> bool:
        """Assign a translator to a job."""
        try:
            translation = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
            
            if not translation:
                logger.error(f"Translation job {job_id} not found")
                return False
            
            # Check if translator exists and is active
            translator = db.query(User).filter(
                and_(User.id == translator_id, User.is_active == True)
            ).first()
            
            if not translator:
                logger.error(f"Translator {translator_id} not found or inactive")
                return False
            
            # Update translation assignment
            translation.translator_id = translator_id
            translation.translation_status = TranslationStatus.IN_PROGRESS
            
            # Update metadata
            metadata = translation.metadata or {}
            metadata.update({
                "assigned_by": assigned_by,
                "assigned_at": datetime.utcnow().isoformat(),
                "workflow_status": WorkflowStatus.ASSIGNED.value
            })
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            db.commit()
            
            logger.info(f"Assigned translator {translator_id} to job {job_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error assigning translator: {e}")
            db.rollback()
            return False
    
    async def update_translation_progress(
        self,
        job_id: int,
        progress_data: Dict[str, Any],
        db: Session
    ) -> bool:
        """Update translation progress."""
        try:
            translation = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
            
            if not translation:
                logger.error(f"Translation job {job_id} not found")
                return False
            
            # Update translation content
            if "title" in progress_data:
                translation.title = progress_data["title"]
            
            if "content" in progress_data:
                translation.content = progress_data["content"]
            
            if "quality_score" in progress_data:
                translation.quality_score = progress_data["quality_score"]
            
            # Update metadata
            metadata = translation.metadata or {}
            metadata.update({
                "last_updated": datetime.utcnow().isoformat(),
                "progress_percentage": progress_data.get("progress_percentage", 0),
                "notes": progress_data.get("notes", "")
            })
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            # Update status if provided
            if "status" in progress_data:
                new_status = progress_data["status"]
                if self._can_transition_to_status(translation.translation_status, new_status):
                    translation.translation_status = new_status
                    metadata["workflow_status"] = new_status
                    translation.metadata = metadata
            
            db.commit()
            
            logger.info(f"Updated progress for translation job {job_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error updating translation progress: {e}")
            db.rollback()
            return False
    
    async def submit_for_review(
        self,
        job_id: int,
        reviewer_id: Optional[int] = None,
        db: Session = None
    ) -> bool:
        """Submit translation for review."""
        try:
            translation = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
            
            if not translation:
                logger.error(f"Translation job {job_id} not found")
                return False
            
            if translation.translation_status != TranslationStatus.IN_PROGRESS:
                logger.error(f"Translation job {job_id} is not in progress")
                return False
            
            # Update status to review
            translation.translation_status = TranslationStatus.REVIEW
            translation.reviewer_id = reviewer_id
            
            # Update metadata
            metadata = translation.metadata or {}
            metadata.update({
                "submitted_for_review_at": datetime.utcnow().isoformat(),
                "workflow_status": WorkflowStatus.REVIEW.value,
                "reviewer_id": reviewer_id
            })
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            db.commit()
            
            logger.info(f"Submitted translation job {job_id} for review")
            return True
            
        except Exception as e:
            logger.error(f"Error submitting for review: {e}")
            db.rollback()
            return False
    
    async def review_translation(
        self,
        job_id: int,
        reviewer_id: int,
        approved: bool,
        feedback: Optional[str] = None,
        quality_score: Optional[float] = None,
        db: Session = None
    ) -> bool:
        """Review and approve/reject translation."""
        try:
            translation = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
            
            if not translation:
                logger.error(f"Translation job {job_id} not found")
                return False
            
            if translation.translation_status != TranslationStatus.REVIEW:
                logger.error(f"Translation job {job_id} is not in review status")
                return False
            
            # Update review results
            new_status = TranslationStatus.APPROVED if approved else TranslationStatus.PENDING
            translation.translation_status = new_status
            translation.reviewer_id = reviewer_id
            
            if quality_score is not None:
                translation.quality_score = quality_score
            
            # Update metadata
            metadata = translation.metadata or {}
            metadata.update({
                "reviewed_at": datetime.utcnow().isoformat(),
                "reviewed_by": reviewer_id,
                "approved": approved,
                "review_feedback": feedback,
                "review_quality_score": quality_score,
                "workflow_status": WorkflowStatus.APPROVED.value if approved else WorkflowStatus.REJECTED.value
            })
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            db.commit()
            
            logger.info(f"Reviewed translation job {job_id}: {'approved' if approved else 'rejected'}")
            return True
            
        except Exception as e:
            logger.error(f"Error reviewing translation: {e}")
            db.rollback()
            return False
    
    async def publish_translation(
        self,
        job_id: int,
        published_by: int,
        db: Session
    ) -> bool:
        """Publish approved translation."""
        try:
            translation = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
            
            if not translation:
                logger.error(f"Translation job {job_id} not found")
                return False
            
            if translation.translation_status != TranslationStatus.APPROVED:
                logger.error(f"Translation job {job_id} is not approved")
                return False
            
            # Update status to published
            translation.translation_status = TranslationStatus.PUBLISHED
            
            # Update metadata
            metadata = translation.metadata or {}
            metadata.update({
                "published_at": datetime.utcnow().isoformat(),
                "published_by": published_by,
                "workflow_status": WorkflowStatus.PUBLISHED.value
            })
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            # Add to translation memory if quality is good
            if translation.quality_score and translation.quality_score >= 0.8:
                await self._add_to_translation_memory(translation, db)
            
            db.commit()
            
            logger.info(f"Published translation job {job_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error publishing translation: {e}")
            db.rollback()
            return False
    
    async def get_workflow_statistics(self, db: Session) -> Dict[str, Any]:
        """Get workflow statistics."""
        try:
            stats = {
                "total_jobs": 0,
                "by_status": {},
                "by_language": {},
                "by_priority": {},
                "average_completion_time": 0,
                "quality_metrics": {}
            }
            
            # Total jobs
            stats["total_jobs"] = db.query(ContentTranslation).count()
            
            # By status
            for status in TranslationStatus:
                count = db.query(ContentTranslation).filter(
                    ContentTranslation.translation_status == status
                ).count()
                stats["by_status"][status.value] = count
            
            # By language
            language_stats = db.query(
                ContentTranslation.language_code,
                db.func.count(ContentTranslation.id)
            ).group_by(ContentTranslation.language_code).all()
            
            for lang_code, count in language_stats:
                stats["by_language"][lang_code] = count
            
            # Quality metrics
            quality_stats = db.query(
                db.func.avg(ContentTranslation.quality_score),
                db.func.min(ContentTranslation.quality_score),
                db.func.max(ContentTranslation.quality_score)
            ).filter(ContentTranslation.quality_score.isnot(None)).first()
            
            if quality_stats[0]:
                stats["quality_metrics"] = {
                    "average_quality": float(quality_stats[0]),
                    "min_quality": float(quality_stats[1]),
                    "max_quality": float(quality_stats[2])
                }
            
            return stats
            
        except Exception as e:
            logger.error(f"Error getting workflow statistics: {e}")
            return {}
    
    async def get_pending_jobs(
        self,
        translator_id: Optional[int] = None,
        language_code: Optional[str] = None,
        priority: Optional[Priority] = None,
        limit: int = 50,
        db: Session = None
    ) -> List[Dict[str, Any]]:
        """Get pending translation jobs."""
        try:
            query = db.query(ContentTranslation).filter(
                ContentTranslation.translation_status.in_([
                    TranslationStatus.PENDING,
                    TranslationStatus.IN_PROGRESS
                ])
            )
            
            if translator_id:
                query = query.filter(ContentTranslation.translator_id == translator_id)
            
            if language_code:
                query = query.filter(ContentTranslation.language_code == language_code)
            
            if priority:
                query = query.filter(
                    ContentTranslation.metadata.op('->>')('priority') == priority.value
                )
            
            jobs = query.limit(limit).all()
            
            result = []
            for job in jobs:
                job_data = {
                    "id": job.id,
                    "content_type": job.content_type,
                    "content_id": job.content_id,
                    "language_code": job.language_code,
                    "status": job.translation_status.value,
                    "translator_id": job.translator_id,
                    "reviewer_id": job.reviewer_id,
                    "quality_score": job.quality_score,
                    "created_at": job.created_at.isoformat(),
                    "updated_at": job.updated_at.isoformat(),
                    "metadata": job.metadata or {}
                }
                result.append(job_data)
            
            return result
            
        except Exception as e:
            logger.error(f"Error getting pending jobs: {e}")
            return []
    
    def _can_transition_to_status(
        self,
        current_status: TranslationStatus,
        new_status: str
    ) -> bool:
        """Check if status transition is allowed."""
        try:
            current_workflow_status = WorkflowStatus(current_status.value)
            new_workflow_status = WorkflowStatus(new_status)
            
            allowed_transitions = self.workflow_rules.get(current_workflow_status, [])
            return new_workflow_status in allowed_transitions
            
        except (ValueError, KeyError):
            return False
    
    async def _add_to_translation_memory(
        self,
        translation: ContentTranslation,
        db: Session
    ) -> None:
        """Add high-quality translation to memory."""
        try:
            if not translation.content or not translation.title:
                return
            
            # Get source language from metadata
            source_language = translation.metadata.get("source_language", "en")
            
            # Add title to memory
            if translation.title:
                memory_entry = TranslationMemory(
                    source_text=f"title_{translation.content_id}",
                    target_text=translation.title,
                    source_language=source_language,
                    target_language=translation.language_code,
                    context="title",
                    domain="content",
                    quality_score=translation.quality_score
                )
                db.add(memory_entry)
            
            # Add content to memory (in chunks if too large)
            if translation.content:
                content_chunks = self._chunk_content(translation.content)
                for i, chunk in enumerate(content_chunks):
                    memory_entry = TranslationMemory(
                        source_text=f"content_{translation.content_id}_chunk_{i}",
                        target_text=chunk,
                        source_language=source_language,
                        target_language=translation.language_code,
                        context="content",
                        domain="content",
                        quality_score=translation.quality_score
                    )
                    db.add(memory_entry)
            
            db.commit()
            logger.info(f"Added translation {translation.id} to memory")
            
        except Exception as e:
            logger.error(f"Error adding to translation memory: {e}")
    
    def _chunk_content(self, content: str, max_length: int = 1000) -> List[str]:
        """Split content into manageable chunks."""
        if len(content) <= max_length:
            return [content]
        
        chunks = []
        sentences = content.split('. ')
        current_chunk = ""
        
        for sentence in sentences:
            if len(current_chunk + sentence) <= max_length:
                current_chunk += sentence + ". "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + ". "
        
        if current_chunk:
            chunks.append(current_chunk.strip())
        
        return chunks


# Global workflow instance
translation_workflow = TranslationWorkflow()