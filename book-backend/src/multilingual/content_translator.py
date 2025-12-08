"""Priority Content Translation System."""

import logging
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
import json

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_

from app.models.multilingual import (
    ContentTranslation, TranslationStatus, TranslationMethod
)
from .ai_translation import ai_translation_service
from .workflow_engine import workflow_engine
from .quality_assurance import quality_assurance

logger = logging.getLogger(__name__)


class ContentPriority:
    """Content priority levels."""
    CRITICAL = 1    # Core concepts, introduction
    HIGH = 2        # Main chapters, key topics
    MEDIUM = 3      # Supporting content, examples
    LOW = 4         # Appendices, references


class PriorityContentTranslator:
    """System for translating priority book content."""
    
    def __init__(self):
        self.priority_chapters = [
            # Critical Priority - Core Programming Concepts
            {
                "id": "ch001",
                "title": "Introduction to Programming",
                "content_type": "chapter",
                "priority": ContentPriority.CRITICAL,
                "estimated_words": 2500,
                "topics": ["programming basics", "computer science", "algorithms"]
            },
            {
                "id": "ch002", 
                "title": "Variables and Data Types",
                "content_type": "chapter",
                "priority": ContentPriority.CRITICAL,
                "estimated_words": 3000,
                "topics": ["variables", "data types", "memory"]
            },
            {
                "id": "ch003",
                "title": "Control Structures",
                "content_type": "chapter", 
                "priority": ContentPriority.CRITICAL,
                "estimated_words": 3500,
                "topics": ["loops", "conditions", "control flow"]
            },
            {
                "id": "ch004",
                "title": "Functions and Methods",
                "content_type": "chapter",
                "priority": ContentPriority.CRITICAL,
                "estimated_words": 4000,
                "topics": ["functions", "methods", "parameters"]
            },
            {
                "id": "ch005",
                "title": "Object-Oriented Programming",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 4500,
                "topics": ["classes", "objects", "inheritance"]
            },
            
            # High Priority - Advanced Concepts
            {
                "id": "ch006",
                "title": "Data Structures",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 4000,
                "topics": ["arrays", "lists", "stacks", "queues"]
            },
            {
                "id": "ch007",
                "title": "Algorithms and Problem Solving",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 3800,
                "topics": ["algorithms", "problem solving", "complexity"]
            },
            {
                "id": "ch008",
                "title": "File Handling and I/O",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 3200,
                "topics": ["files", "input output", "streams"]
            },
            {
                "id": "ch009",
                "title": "Error Handling and Debugging",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 3000,
                "topics": ["exceptions", "debugging", "testing"]
            },
            {
                "id": "ch010",
                "title": "Database Fundamentals",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 3500,
                "topics": ["databases", "SQL", "data modeling"]
            },
            
            # Medium Priority - Practical Applications
            {
                "id": "ch011",
                "title": "Web Development Basics",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 4200,
                "topics": ["web development", "HTML", "CSS", "JavaScript"]
            },
            {
                "id": "ch012",
                "title": "API Development",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3800,
                "topics": ["APIs", "REST", "HTTP"]
            },
            {
                "id": "ch013",
                "title": "Version Control with Git",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 2800,
                "topics": ["git", "version control", "collaboration"]
            },
            {
                "id": "ch014",
                "title": "Software Testing",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3200,
                "topics": ["testing", "unit tests", "integration tests"]
            },
            {
                "id": "ch015",
                "title": "Design Patterns",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3600,
                "topics": ["design patterns", "architecture", "best practices"]
            },
            
            # Additional High-Value Chapters
            {
                "id": "ch016",
                "title": "Mobile App Development",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 4000,
                "topics": ["mobile development", "apps", "UI/UX"]
            },
            {
                "id": "ch017",
                "title": "Cloud Computing Basics",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3400,
                "topics": ["cloud computing", "AWS", "deployment"]
            },
            {
                "id": "ch018",
                "title": "Security Fundamentals",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 3600,
                "topics": ["security", "encryption", "authentication"]
            },
            {
                "id": "ch019",
                "title": "Performance Optimization",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3200,
                "topics": ["performance", "optimization", "profiling"]
            },
            {
                "id": "ch020",
                "title": "Career in Programming",
                "content_type": "chapter",
                "priority": ContentPriority.HIGH,
                "estimated_words": 2800,
                "topics": ["career", "job search", "professional development"]
            },
            
            # Bonus Chapters
            {
                "id": "ch021",
                "title": "Machine Learning Basics",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 4200,
                "topics": ["machine learning", "AI", "data science"]
            },
            {
                "id": "ch022",
                "title": "DevOps and Deployment",
                "content_type": "chapter",
                "priority": ContentPriority.MEDIUM,
                "estimated_words": 3800,
                "topics": ["DevOps", "CI/CD", "deployment"]
            }
        ]
        
        # Sample content for demonstration
        self.sample_content = {
            "ch001": {
                "title": "Introduction to Programming",
                "content": """
Programming is the process of creating a set of instructions that tell a computer how to perform a task. 
Programming can be done using a variety of computer programming languages, such as JavaScript, Python, and C++.

## What is Programming?

Programming is essentially problem-solving using code. When you program, you break down complex problems 
into smaller, manageable pieces and then write instructions that a computer can follow to solve those problems.

### Key Concepts:

1. **Algorithm**: A step-by-step procedure for solving a problem
2. **Syntax**: The rules that define valid constructs in a programming language  
3. **Variables**: Storage locations with associated names that contain data
4. **Functions**: Reusable blocks of code that perform specific tasks

### Why Learn Programming?

- **Problem Solving**: Programming teaches logical thinking and problem-solving skills
- **Career Opportunities**: High demand for programmers across industries
- **Creativity**: Build applications, websites, games, and more
- **Automation**: Automate repetitive tasks and processes

Programming languages are tools that allow humans to communicate with computers. Just as we use 
different languages to communicate with people from different countries, we use different programming 
languages to communicate with computers for different purposes.
"""
            },
            "ch002": {
                "title": "Variables and Data Types", 
                "content": """
Variables are fundamental building blocks in programming. They are like containers that store data 
values that can be used and manipulated throughout your program.

## Understanding Variables

A variable is a storage location with an associated name that contains data. Think of it as a labeled 
box where you can store different types of information.

### Variable Declaration

In most programming languages, you need to declare a variable before using it:

```python
# Python example
name = "John"
age = 25
height = 5.9
is_student = True
```

### Data Types

Different types of data require different types of storage:

#### 1. **Numbers**
- **Integers**: Whole numbers (1, 2, 3, -5, 100)
- **Floating-point**: Decimal numbers (3.14, -2.5, 0.001)

#### 2. **Text** 
- **Strings**: Sequences of characters ("Hello", "Programming", "123")

#### 3. **Boolean**
- **True/False**: Logical values (True, False)

#### 4. **Collections**
- **Arrays/Lists**: Ordered collections of items
- **Objects**: Complex data structures with properties

### Variable Naming Rules

1. Use descriptive names: `student_name` instead of `sn`
2. Start with letter or underscore: `name`, `_count`
3. No spaces: use `first_name` not `first name`
4. Case sensitive: `Name` and `name` are different

Variables make programs flexible and reusable by allowing the same code to work with different data.
"""
            }
        }
    
    async def translate_priority_content(
        self,
        target_language: str = "ur",
        batch_size: int = 5,
        db: Session = None
    ) -> Dict[str, Any]:
        """Translate priority content in batches."""
        try:
            results = {
                "total_chapters": len(self.priority_chapters),
                "translated": 0,
                "failed": 0,
                "skipped": 0,
                "translation_jobs": [],
                "errors": []
            }
            
            # Sort chapters by priority
            sorted_chapters = sorted(
                self.priority_chapters, 
                key=lambda x: x["priority"]
            )
            
            # Process in batches
            for i in range(0, len(sorted_chapters), batch_size):
                batch = sorted_chapters[i:i + batch_size]
                
                logger.info(f"Processing batch {i//batch_size + 1}: chapters {i+1}-{min(i+batch_size, len(sorted_chapters))}")
                
                batch_results = await self._process_chapter_batch(
                    batch, target_language, db
                )
                
                # Update results
                results["translated"] += batch_results["translated"]
                results["failed"] += batch_results["failed"] 
                results["skipped"] += batch_results["skipped"]
                results["translation_jobs"].extend(batch_results["jobs"])
                results["errors"].extend(batch_results["errors"])
                
                # Small delay between batches to avoid overwhelming the system
                await asyncio.sleep(2)
            
            # Generate summary report
            summary = await self._generate_translation_summary(results, target_language, db)
            results["summary"] = summary
            
            return results
            
        except Exception as e:
            logger.error(f"Error in priority content translation: {e}")
            return {"error": str(e)}
    
    async def _process_chapter_batch(
        self,
        chapters: List[Dict[str, Any]],
        target_language: str,
        db: Session
    ) -> Dict[str, Any]:
        """Process a batch of chapters for translation."""
        batch_results = {
            "translated": 0,
            "failed": 0,
            "skipped": 0,
            "jobs": [],
            "errors": []
        }
        
        try:
            # Create translation tasks for each chapter
            tasks = []
            for chapter in chapters:
                task = self._translate_single_chapter(
                    chapter, target_language, db
                )
                tasks.append(task)
            
            # Execute batch with concurrency control
            semaphore = asyncio.Semaphore(3)  # Limit concurrent translations
            
            async def limited_task(task):
                async with semaphore:
                    return await task
            
            # Wait for all tasks to complete
            results = await asyncio.gather(
                *[limited_task(task) for task in tasks],
                return_exceptions=True
            )
            
            # Process results
            for i, result in enumerate(results):
                chapter = chapters[i]
                
                if isinstance(result, Exception):
                    batch_results["failed"] += 1
                    batch_results["errors"].append({
                        "chapter_id": chapter["id"],
                        "error": str(result)
                    })
                    logger.error(f"Failed to translate chapter {chapter['id']}: {result}")
                    
                elif result.get("success"):
                    batch_results["translated"] += 1
                    batch_results["jobs"].append(result["job_info"])
                    logger.info(f"Successfully queued translation for chapter {chapter['id']}")
                    
                elif result.get("skipped"):
                    batch_results["skipped"] += 1
                    logger.info(f"Skipped chapter {chapter['id']}: {result.get('reason', 'Unknown')}")
                    
                else:
                    batch_results["failed"] += 1
                    batch_results["errors"].append({
                        "chapter_id": chapter["id"],
                        "error": result.get("error", "Unknown error")
                    })
            
            return batch_results
            
        except Exception as e:
            logger.error(f"Error processing chapter batch: {e}")
            batch_results["errors"].append({"batch_error": str(e)})
            return batch_results
    
    async def _translate_single_chapter(
        self,
        chapter: Dict[str, Any],
        target_language: str,
        db: Session
    ) -> Dict[str, Any]:
        """Translate a single chapter."""
        try:
            chapter_id = chapter["id"]
            
            # Check if translation already exists
            existing = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.content_type == "chapter",
                    ContentTranslation.content_id == chapter_id,
                    ContentTranslation.language_code == target_language
                )
            ).first()
            
            if existing and existing.translation_status in [
                TranslationStatus.PUBLISHED,
                TranslationStatus.APPROVED
            ]:
                return {
                    "success": False,
                    "skipped": True,
                    "reason": "Translation already exists and is published/approved"
                }
            
            # Get chapter content (from sample or database)
            content_data = self.sample_content.get(chapter_id)
            if not content_data:
                # In production, this would fetch from the actual content database
                content_data = {
                    "title": chapter["title"],
                    "content": f"Sample content for {chapter['title']}. This would contain the actual chapter content in a real implementation."
                }
            
            # Create translation job through workflow engine
            task_id = await workflow_engine.submit_translation_job(
                content_type="chapter",
                content_id=chapter_id,
                source_language="en",
                target_language=target_language,
                title=content_data["title"],
                content=content_data["content"],
                priority=self._map_priority(chapter["priority"]),
                auto_translate=True,
                metadata={
                    "chapter_info": chapter,
                    "estimated_words": chapter.get("estimated_words", 0),
                    "topics": chapter.get("topics", []),
                    "batch_translation": True,
                    "priority_content": True
                },
                db=db
            )
            
            if task_id:
                return {
                    "success": True,
                    "job_info": {
                        "task_id": task_id,
                        "chapter_id": chapter_id,
                        "title": chapter["title"],
                        "priority": chapter["priority"],
                        "estimated_words": chapter.get("estimated_words", 0)
                    }
                }
            else:
                return {
                    "success": False,
                    "error": "Failed to create translation job"
                }
                
        except Exception as e:
            logger.error(f"Error translating chapter {chapter.get('id', 'unknown')}: {e}")
            return {
                "success": False,
                "error": str(e)
            }
    
    def _map_priority(self, content_priority: int) -> str:
        """Map content priority to workflow priority."""
        priority_map = {
            ContentPriority.CRITICAL: "urgent",
            ContentPriority.HIGH: "high", 
            ContentPriority.MEDIUM: "medium",
            ContentPriority.LOW: "low"
        }
        return priority_map.get(content_priority, "medium")
    
    async def _generate_translation_summary(
        self,
        results: Dict[str, Any],
        target_language: str,
        db: Session
    ) -> Dict[str, Any]:
        """Generate translation summary report."""
        try:
            summary = {
                "translation_overview": {
                    "target_language": target_language,
                    "total_chapters": results["total_chapters"],
                    "successfully_queued": results["translated"],
                    "failed": results["failed"],
                    "skipped": results["skipped"],
                    "success_rate": results["translated"] / max(results["total_chapters"], 1) * 100
                },
                "priority_breakdown": {},
                "estimated_metrics": {},
                "next_steps": []
            }
            
            # Priority breakdown
            for priority in [ContentPriority.CRITICAL, ContentPriority.HIGH, ContentPriority.MEDIUM, ContentPriority.LOW]:
                priority_chapters = [ch for ch in self.priority_chapters if ch["priority"] == priority]
                priority_jobs = [job for job in results["translation_jobs"] if job.get("priority") == priority]
                
                summary["priority_breakdown"][f"priority_{priority}"] = {
                    "total_chapters": len(priority_chapters),
                    "queued_for_translation": len(priority_jobs),
                    "estimated_words": sum(ch.get("estimated_words", 0) for ch in priority_chapters)
                }
            
            # Estimated metrics
            total_words = sum(ch.get("estimated_words", 0) for ch in self.priority_chapters)
            queued_words = sum(job.get("estimated_words", 0) for job in results["translation_jobs"])
            
            summary["estimated_metrics"] = {
                "total_estimated_words": total_words,
                "queued_words": queued_words,
                "estimated_translation_time_hours": queued_words / 500,  # Assuming 500 words/hour
                "estimated_review_time_hours": queued_words / 1000,     # Assuming 1000 words/hour for review
                "estimated_completion_days": (queued_words / 500) / 8   # 8 hours per day
            }
            
            # Next steps
            if results["translated"] > 0:
                summary["next_steps"].append("Monitor translation progress in workflow dashboard")
                summary["next_steps"].append("Review completed translations for quality")
                summary["next_steps"].append("Assign reviewers for high-priority content")
            
            if results["failed"] > 0:
                summary["next_steps"].append(f"Investigate and retry {results['failed']} failed translations")
            
            if results["skipped"] > 0:
                summary["next_steps"].append(f"Review {results['skipped']} skipped items for updates")
            
            return summary
            
        except Exception as e:
            logger.error(f"Error generating translation summary: {e}")
            return {"error": str(e)}
    
    async def get_translation_progress(
        self,
        target_language: str = "ur",
        db: Session = None
    ) -> Dict[str, Any]:
        """Get current translation progress for priority content."""
        try:
            progress = {
                "overall_progress": {},
                "by_priority": {},
                "by_status": {},
                "quality_metrics": {},
                "recent_completions": []
            }
            
            # Get all translations for priority chapters
            chapter_ids = [ch["id"] for ch in self.priority_chapters]
            
            translations = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.content_type == "chapter",
                    ContentTranslation.content_id.in_(chapter_ids),
                    ContentTranslation.language_code == target_language
                )
            ).all()
            
            # Overall progress
            total_chapters = len(self.priority_chapters)
            translated_count = len(translations)
            published_count = len([t for t in translations if t.translation_status == TranslationStatus.PUBLISHED])
            
            progress["overall_progress"] = {
                "total_chapters": total_chapters,
                "translation_started": translated_count,
                "published": published_count,
                "completion_percentage": (published_count / total_chapters) * 100,
                "translation_percentage": (translated_count / total_chapters) * 100
            }
            
            # Progress by priority
            for priority in [ContentPriority.CRITICAL, ContentPriority.HIGH, ContentPriority.MEDIUM, ContentPriority.LOW]:
                priority_chapters = [ch for ch in self.priority_chapters if ch["priority"] == priority]
                priority_chapter_ids = [ch["id"] for ch in priority_chapters]
                priority_translations = [t for t in translations if t.content_id in priority_chapter_ids]
                priority_published = [t for t in priority_translations if t.translation_status == TranslationStatus.PUBLISHED]
                
                progress["by_priority"][f"priority_{priority}"] = {
                    "total": len(priority_chapters),
                    "translated": len(priority_translations),
                    "published": len(priority_published),
                    "completion_rate": (len(priority_published) / max(len(priority_chapters), 1)) * 100
                }
            
            # Progress by status
            status_counts = {}
            for status in TranslationStatus:
                count = len([t for t in translations if t.translation_status == status])
                status_counts[status.value] = count
            
            progress["by_status"] = status_counts
            
            # Quality metrics
            quality_scores = [t.quality_score for t in translations if t.quality_score is not None]
            if quality_scores:
                progress["quality_metrics"] = {
                    "average_quality": sum(quality_scores) / len(quality_scores),
                    "min_quality": min(quality_scores),
                    "max_quality": max(quality_scores),
                    "high_quality_count": len([s for s in quality_scores if s >= 0.8])
                }
            
            # Recent completions
            recent_translations = sorted(
                [t for t in translations if t.translation_status == TranslationStatus.PUBLISHED],
                key=lambda x: x.updated_at,
                reverse=True
            )[:5]
            
            progress["recent_completions"] = [
                {
                    "chapter_id": t.content_id,
                    "title": t.title,
                    "quality_score": t.quality_score,
                    "completed_at": t.updated_at.isoformat()
                }
                for t in recent_translations
            ]
            
            return progress
            
        except Exception as e:
            logger.error(f"Error getting translation progress: {e}")
            return {"error": str(e)}
    
    def get_priority_chapters_info(self) -> List[Dict[str, Any]]:
        """Get information about priority chapters."""
        return self.priority_chapters.copy()
    
    async def retry_failed_translations(
        self,
        target_language: str = "ur",
        db: Session = None
    ) -> Dict[str, Any]:
        """Retry failed translations."""
        try:
            # Find failed translations
            chapter_ids = [ch["id"] for ch in self.priority_chapters]
            
            failed_translations = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.content_type == "chapter",
                    ContentTranslation.content_id.in_(chapter_ids),
                    ContentTranslation.language_code == target_language,
                    ContentTranslation.translation_status == TranslationStatus.PENDING,
                    ContentTranslation.quality_score < 0.5
                )
            ).all()
            
            retry_results = {
                "total_retries": len(failed_translations),
                "successful_retries": 0,
                "failed_retries": 0,
                "retry_jobs": []
            }
            
            for translation in failed_translations:
                try:
                    # Find chapter info
                    chapter_info = next(
                        (ch for ch in self.priority_chapters if ch["id"] == translation.content_id),
                        None
                    )
                    
                    if not chapter_info:
                        continue
                    
                    # Resubmit for translation
                    task_id = await workflow_engine.submit_translation_job(
                        content_type="chapter",
                        content_id=translation.content_id,
                        source_language="en",
                        target_language=target_language,
                        title=translation.title,
                        content=translation.content,
                        priority=self._map_priority(chapter_info["priority"]),
                        auto_translate=True,
                        metadata={
                            "retry_attempt": True,
                            "original_translation_id": translation.id,
                            "chapter_info": chapter_info
                        },
                        db=db
                    )
                    
                    if task_id:
                        retry_results["successful_retries"] += 1
                        retry_results["retry_jobs"].append({
                            "task_id": task_id,
                            "chapter_id": translation.content_id,
                            "original_quality": translation.quality_score
                        })
                    else:
                        retry_results["failed_retries"] += 1
                        
                except Exception as e:
                    logger.error(f"Error retrying translation for {translation.content_id}: {e}")
                    retry_results["failed_retries"] += 1
            
            return retry_results
            
        except Exception as e:
            logger.error(f"Error in retry failed translations: {e}")
            return {"error": str(e)}


# Global content translator instance
content_translator = PriorityContentTranslator()