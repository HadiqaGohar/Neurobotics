"""Translation Workflow Engine with automated processing and queue management."""

import logging
import asyncio
from typing import Dict, Any, List, Optional, Callable
from datetime import datetime, timedelta
from enum import Enum
import json
from dataclasses import dataclass, asdict

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_

from app.models.multilingual import (
    ContentTranslation, TranslationStatus, TranslationMethod,
    Language, TranslationMemory
)
from app.models.database import User
from .ai_translation import ai_translation_service, TranslationProvider
from .workflow import translation_workflow, Priority, WorkflowStatus

logger = logging.getLogger(__name__)


class QueuePriority(int, Enum):
    """Queue priority levels."""
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    URGENT = 4


@dataclass
class TranslationTask:
    """Translation task data structure."""
    id: str
    job_id: int
    content_type: str
    content_id: str
    source_language: str
    target_language: str
    title: Optional[str]
    content: Optional[str]
    priority: QueuePriority
    deadline: Optional[datetime]
    metadata: Dict[str, Any]
    created_at: datetime
    retry_count: int = 0
    max_retries: int = 3


class WorkflowEngine:
    """Translation workflow engine with automated processing."""
    
    def __init__(self):
        self.task_queue = asyncio.Queue()
        self.processing_tasks = {}
        self.completed_tasks = {}
        self.failed_tasks = {}
        self.workers = []
        self.is_running = False
        self.worker_count = 3
        self.batch_size = 5
        self.processing_interval = 30  # seconds
        
        # Workflow rules and handlers
        self.status_handlers = {
            TranslationStatus.PENDING: self._handle_pending_task,
            TranslationStatus.IN_PROGRESS: self._handle_in_progress_task,
            TranslationStatus.REVIEW: self._handle_review_task,
            TranslationStatus.APPROVED: self._handle_approved_task,
        }
        
        # Quality thresholds
        self.quality_thresholds = {
            "auto_approve": 0.9,
            "needs_review": 0.7,
            "needs_revision": 0.5
        }
    
    async def start_engine(self, db_session_factory: Callable):
        """Start the workflow engine."""
        if self.is_running:
            logger.warning("Workflow engine is already running")
            return
        
        self.is_running = True
        self.db_session_factory = db_session_factory
        
        logger.info(f"Starting workflow engine with {self.worker_count} workers")
        
        # Start worker tasks
        for i in range(self.worker_count):
            worker = asyncio.create_task(self._worker(f"worker-{i}"))
            self.workers.append(worker)
        
        # Start queue processor
        queue_processor = asyncio.create_task(self._queue_processor())
        self.workers.append(queue_processor)
        
        # Start periodic tasks
        periodic_processor = asyncio.create_task(self._periodic_processor())
        self.workers.append(periodic_processor)
        
        logger.info("Workflow engine started successfully")
    
    async def stop_engine(self):
        """Stop the workflow engine."""
        if not self.is_running:
            return
        
        logger.info("Stopping workflow engine...")
        self.is_running = False
        
        # Cancel all workers
        for worker in self.workers:
            worker.cancel()
        
        # Wait for workers to finish
        await asyncio.gather(*self.workers, return_exceptions=True)
        
        self.workers.clear()
        logger.info("Workflow engine stopped")
    
    async def submit_translation_job(
        self,
        content_type: str,
        content_id: str,
        source_language: str,
        target_language: str,
        title: Optional[str] = None,
        content: Optional[str] = None,
        priority: Priority = Priority.MEDIUM,
        deadline: Optional[datetime] = None,
        auto_translate: bool = True,
        metadata: Optional[Dict[str, Any]] = None,
        db: Session = None
    ) -> Optional[str]:
        """Submit a translation job to the workflow engine."""
        try:
            # Create translation job in database
            job_id = await translation_workflow.create_translation_job(
                content_type=content_type,
                content_id=content_id,
                source_language=source_language,
                target_language=target_language,
                title=title,
                content=content,
                priority=priority,
                deadline=deadline,
                metadata=metadata,
                db=db
            )
            
            if not job_id:
                logger.error("Failed to create translation job")
                return None
            
            # Create workflow task
            task_id = f"task_{job_id}_{datetime.utcnow().timestamp()}"
            
            task = TranslationTask(
                id=task_id,
                job_id=job_id,
                content_type=content_type,
                content_id=content_id,
                source_language=source_language,
                target_language=target_language,
                title=title,
                content=content,
                priority=QueuePriority(priority.value if hasattr(priority, 'value') else 2),
                deadline=deadline,
                metadata=metadata or {},
                created_at=datetime.utcnow()
            )
            
            # Add to queue if auto-translate is enabled
            if auto_translate:
                await self.task_queue.put(task)
                logger.info(f"Submitted translation task {task_id} to queue")
            
            return task_id
            
        except Exception as e:
            logger.error(f"Error submitting translation job: {e}")
            return None
    
    async def get_queue_status(self) -> Dict[str, Any]:
        """Get current queue status."""
        return {
            "is_running": self.is_running,
            "queue_size": self.task_queue.qsize(),
            "processing_tasks": len(self.processing_tasks),
            "completed_tasks": len(self.completed_tasks),
            "failed_tasks": len(self.failed_tasks),
            "worker_count": len(self.workers),
            "uptime": datetime.utcnow().isoformat() if self.is_running else None
        }
    
    async def get_task_status(self, task_id: str) -> Optional[Dict[str, Any]]:
        """Get status of a specific task."""
        # Check processing tasks
        if task_id in self.processing_tasks:
            task = self.processing_tasks[task_id]
            return {
                "status": "processing",
                "task": asdict(task),
                "started_at": task.metadata.get("started_at")
            }
        
        # Check completed tasks
        if task_id in self.completed_tasks:
            return {
                "status": "completed",
                "result": self.completed_tasks[task_id]
            }
        
        # Check failed tasks
        if task_id in self.failed_tasks:
            return {
                "status": "failed",
                "error": self.failed_tasks[task_id]
            }
        
        return None
    
    async def _worker(self, worker_name: str):
        """Worker process for handling translation tasks."""
        logger.info(f"Worker {worker_name} started")
        
        while self.is_running:
            try:
                # Get task from queue with timeout
                task = await asyncio.wait_for(
                    self.task_queue.get(),
                    timeout=5.0
                )
                
                logger.info(f"Worker {worker_name} processing task {task.id}")
                
                # Add to processing tasks
                task.metadata["started_at"] = datetime.utcnow().isoformat()
                task.metadata["worker"] = worker_name
                self.processing_tasks[task.id] = task
                
                # Process the task
                result = await self._process_translation_task(task)
                
                # Handle result
                if result.get("success", False):
                    self.completed_tasks[task.id] = result
                    logger.info(f"Task {task.id} completed successfully")
                else:
                    # Retry logic
                    if task.retry_count < task.max_retries:
                        task.retry_count += 1
                        task.metadata["retry_at"] = datetime.utcnow().isoformat()
                        await self.task_queue.put(task)
                        logger.warning(f"Task {task.id} failed, retrying ({task.retry_count}/{task.max_retries})")
                    else:
                        self.failed_tasks[task.id] = result
                        logger.error(f"Task {task.id} failed permanently after {task.max_retries} retries")
                
                # Remove from processing
                self.processing_tasks.pop(task.id, None)
                
                # Mark task as done in queue
                self.task_queue.task_done()
                
            except asyncio.TimeoutError:
                # No tasks in queue, continue
                continue
            except asyncio.CancelledError:
                logger.info(f"Worker {worker_name} cancelled")
                break
            except Exception as e:
                logger.error(f"Worker {worker_name} error: {e}")
                await asyncio.sleep(1)
        
        logger.info(f"Worker {worker_name} stopped")
    
    async def _process_translation_task(self, task: TranslationTask) -> Dict[str, Any]:
        """Process a single translation task."""
        try:
            db = self.db_session_factory()
            
            # Get current job status
            job = db.query(ContentTranslation).filter(
                ContentTranslation.id == task.job_id
            ).first()
            
            if not job:
                return {"success": False, "error": "Job not found"}
            
            # Route to appropriate handler based on status
            handler = self.status_handlers.get(job.translation_status)
            if handler:
                result = await handler(task, job, db)
            else:
                result = {"success": False, "error": f"No handler for status {job.translation_status}"}
            
            db.close()
            return result
            
        except Exception as e:
            logger.error(f"Error processing task {task.id}: {e}")
            return {"success": False, "error": str(e)}
    
    async def _handle_pending_task(
        self,
        task: TranslationTask,
        job: ContentTranslation,
        db: Session
    ) -> Dict[str, Any]:
        """Handle pending translation task."""
        try:
            # Check if we have content to translate
            if not task.content and not task.title:
                return {"success": False, "error": "No content to translate"}
            
            # Perform AI translation
            translation_results = []
            
            # Translate title if present
            if task.title:
                title_result = await ai_translation_service.translate_text(
                    text=task.title,
                    source_language=task.source_language,
                    target_language=task.target_language,
                    context="title",
                    domain=task.content_type,
                    db=db
                )
                translation_results.append(("title", title_result))
            
            # Translate content if present
            if task.content:
                # Split content into chunks if too large
                content_chunks = self._split_content(task.content)
                
                for i, chunk in enumerate(content_chunks):
                    chunk_result = await ai_translation_service.translate_text(
                        text=chunk,
                        source_language=task.source_language,
                        target_language=task.target_language,
                        context=f"content_chunk_{i}",
                        domain=task.content_type,
                        db=db
                    )
                    translation_results.append((f"content_chunk_{i}", chunk_result))
            
            # Combine results
            translated_title = None
            translated_content_chunks = []
            overall_quality = 0.0
            
            for result_type, result in translation_results:
                if result_type == "title":
                    translated_title = result.get("translated_text")
                    overall_quality += result.get("quality_score", 0.0)
                elif result_type.startswith("content_chunk"):
                    translated_content_chunks.append(result.get("translated_text", ""))
                    overall_quality += result.get("quality_score", 0.0)
            
            # Calculate average quality
            if translation_results:
                overall_quality /= len(translation_results)
            
            # Combine content chunks
            translated_content = " ".join(translated_content_chunks) if translated_content_chunks else None
            
            # Update job with translation
            if translated_title:
                job.title = translated_title
            if translated_content:
                job.content = translated_content
            
            job.quality_score = overall_quality
            job.translation_method = TranslationMethod.AI
            
            # Determine next status based on quality
            if overall_quality >= self.quality_thresholds["auto_approve"]:
                job.translation_status = TranslationStatus.APPROVED
                next_action = "auto_approved"
            elif overall_quality >= self.quality_thresholds["needs_review"]:
                job.translation_status = TranslationStatus.REVIEW
                next_action = "needs_review"
            else:
                job.translation_status = TranslationStatus.PENDING
                next_action = "needs_revision"
            
            # Update metadata
            metadata = job.metadata or {}
            metadata.update({
                "ai_translation_completed": datetime.utcnow().isoformat(),
                "translation_quality": overall_quality,
                "next_action": next_action,
                "translation_provider": "ai_service"
            })
            job.metadata = metadata
            job.updated_at = datetime.utcnow()
            
            db.commit()
            
            return {
                "success": True,
                "translated_title": translated_title,
                "translated_content": translated_content,
                "quality_score": overall_quality,
                "next_status": job.translation_status.value,
                "next_action": next_action
            }
            
        except Exception as e:
            logger.error(f"Error handling pending task: {e}")
            return {"success": False, "error": str(e)}
    
    async def _handle_in_progress_task(
        self,
        task: TranslationTask,
        job: ContentTranslation,
        db: Session
    ) -> Dict[str, Any]:
        """Handle in-progress translation task."""
        try:
            # Check if task has been idle too long
            if job.updated_at:
                idle_time = datetime.utcnow() - job.updated_at
                if idle_time > timedelta(hours=24):  # 24 hours idle
                    # Send reminder or reassign
                    metadata = job.metadata or {}
                    metadata["idle_reminder_sent"] = datetime.utcnow().isoformat()
                    job.metadata = metadata
                    db.commit()
                    
                    return {
                        "success": True,
                        "action": "idle_reminder_sent",
                        "idle_time_hours": idle_time.total_seconds() / 3600
                    }
            
            return {"success": True, "action": "no_action_needed"}
            
        except Exception as e:
            logger.error(f"Error handling in-progress task: {e}")
            return {"success": False, "error": str(e)}
    
    async def _handle_review_task(
        self,
        task: TranslationTask,
        job: ContentTranslation,
        db: Session
    ) -> Dict[str, Any]:
        """Handle review task."""
        try:
            # Auto-assign reviewer if not assigned
            if not job.reviewer_id:
                # Find available reviewer
                reviewer = await self._find_available_reviewer(
                    task.target_language, db
                )
                
                if reviewer:
                    job.reviewer_id = reviewer.id
                    metadata = job.metadata or {}
                    metadata["auto_assigned_reviewer"] = datetime.utcnow().isoformat()
                    job.metadata = metadata
                    db.commit()
                    
                    return {
                        "success": True,
                        "action": "reviewer_assigned",
                        "reviewer_id": reviewer.id
                    }
            
            return {"success": True, "action": "awaiting_review"}
            
        except Exception as e:
            logger.error(f"Error handling review task: {e}")
            return {"success": False, "error": str(e)}
    
    async def _handle_approved_task(
        self,
        task: TranslationTask,
        job: ContentTranslation,
        db: Session
    ) -> Dict[str, Any]:
        """Handle approved task."""
        try:
            # Auto-publish if quality is high enough
            if job.quality_score and job.quality_score >= self.quality_thresholds["auto_approve"]:
                success = await translation_workflow.publish_translation(
                    job_id=job.id,
                    published_by=1,  # System user
                    db=db
                )
                
                if success:
                    return {
                        "success": True,
                        "action": "auto_published",
                        "quality_score": job.quality_score
                    }
            
            return {"success": True, "action": "awaiting_manual_publish"}
            
        except Exception as e:
            logger.error(f"Error handling approved task: {e}")
            return {"success": False, "error": str(e)}
    
    async def _queue_processor(self):
        """Process queue and prioritize tasks."""
        logger.info("Queue processor started")
        
        while self.is_running:
            try:
                await asyncio.sleep(self.processing_interval)
                
                # Get pending jobs from database
                db = self.db_session_factory()
                
                pending_jobs = db.query(ContentTranslation).filter(
                    ContentTranslation.translation_status.in_([
                        TranslationStatus.PENDING,
                        TranslationStatus.IN_PROGRESS,
                        TranslationStatus.REVIEW
                    ])
                ).limit(50).all()
                
                # Create tasks for jobs not in queue
                for job in pending_jobs:
                    task_id = f"task_{job.id}_{datetime.utcnow().timestamp()}"
                    
                    # Skip if already processing
                    if any(task.job_id == job.id for task in self.processing_tasks.values()):
                        continue
                    
                    # Create task
                    priority = QueuePriority.MEDIUM
                    if job.metadata:
                        priority_str = job.metadata.get("priority", "medium")
                        priority = QueuePriority[priority_str.upper()] if priority_str.upper() in QueuePriority.__members__ else QueuePriority.MEDIUM
                    
                    task = TranslationTask(
                        id=task_id,
                        job_id=job.id,
                        content_type=job.content_type,
                        content_id=job.content_id,
                        source_language=job.metadata.get("source_language", "en") if job.metadata else "en",
                        target_language=job.language_code,
                        title=job.title,
                        content=job.content,
                        priority=priority,
                        deadline=None,  # Would need to parse from metadata
                        metadata=job.metadata or {},
                        created_at=job.created_at
                    )
                    
                    await self.task_queue.put(task)
                
                db.close()
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Queue processor error: {e}")
                await asyncio.sleep(5)
        
        logger.info("Queue processor stopped")
    
    async def _periodic_processor(self):
        """Handle periodic maintenance tasks."""
        logger.info("Periodic processor started")
        
        while self.is_running:
            try:
                await asyncio.sleep(300)  # Run every 5 minutes
                
                # Clean up old completed/failed tasks
                current_time = datetime.utcnow()
                
                # Remove completed tasks older than 1 hour
                completed_to_remove = [
                    task_id for task_id, result in self.completed_tasks.items()
                    if "completed_at" in result and 
                    (current_time - datetime.fromisoformat(result["completed_at"])).total_seconds() > 3600
                ]
                
                for task_id in completed_to_remove:
                    self.completed_tasks.pop(task_id, None)
                
                # Remove failed tasks older than 24 hours
                failed_to_remove = [
                    task_id for task_id, result in self.failed_tasks.items()
                    if "failed_at" in result and 
                    (current_time - datetime.fromisoformat(result["failed_at"])).total_seconds() > 86400
                ]
                
                for task_id in failed_to_remove:
                    self.failed_tasks.pop(task_id, None)
                
                logger.info(f"Cleaned up {len(completed_to_remove)} completed and {len(failed_to_remove)} failed tasks")
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Periodic processor error: {e}")
                await asyncio.sleep(30)
        
        logger.info("Periodic processor stopped")
    
    def _split_content(self, content: str, max_chunk_size: int = 2000) -> List[str]:
        """Split content into manageable chunks."""
        if len(content) <= max_chunk_size:
            return [content]
        
        chunks = []
        sentences = content.split('. ')
        current_chunk = ""
        
        for sentence in sentences:
            if len(current_chunk + sentence) <= max_chunk_size:
                current_chunk += sentence + ". "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + ". "
        
        if current_chunk:
            chunks.append(current_chunk.strip())
        
        return chunks
    
    async def _find_available_reviewer(
        self,
        language_code: str,
        db: Session
    ) -> Optional[User]:
        """Find available reviewer for language."""
        try:
            # This would implement logic to find reviewers
            # For now, return None (manual assignment required)
            return None
            
        except Exception as e:
            logger.error(f"Error finding reviewer: {e}")
            return None


# Global workflow engine instance
workflow_engine = WorkflowEngine()