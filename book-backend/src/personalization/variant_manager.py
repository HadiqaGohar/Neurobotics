"""
Content variant management system for personalization.
"""

import logging
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, desc

from .models import ExperienceLevel, ContentComplexity
from .content_tagger import content_tagger, ContentTag
from app.models.database import ContentVariant as ContentVariantDB
from app.models.database import UserPreferences as UserPreferencesDB

logger = logging.getLogger(__name__)


class VariantStatus(str, Enum):
    """Content variant status."""
    DRAFT = "draft"
    ACTIVE = "active"
    ARCHIVED = "archived"
    DEPRECATED = "deprecated"


class VariantType(str, Enum):
    """Content variant types."""
    MANUAL = "manual"
    AI_GENERATED = "ai_generated"
    TEMPLATE = "template"
    USER_CUSTOMIZED = "user_customized"


class ContentVariantManager:
    """Manager for content variants with CRUD operations and version control."""
    
    def __init__(self):
        self.cache = {}
        self.cache_ttl = 1800  # 30 minutes
    
    async def create_variant(
        self,
        db: Session,
        chapter_id: str,
        section_id: Optional[str],
        variant_type: str,
        target_audience: Dict[str, Any],
        content: Dict[str, Any],
        metadata: Optional[Dict[str, Any]] = None,
        is_ai_generated: bool = False
    ) -> Optional[ContentVariantDB]:
        """Create a new content variant."""
        try:
            # Generate unique content ID
            content_id = self._generate_content_id(chapter_id, section_id, variant_type)
            
            # Tag the content
            content_tags = content_tagger.tag_content(content)
            
            # Prepare metadata
            if not metadata:
                metadata = {}
            
            metadata.update({
                "created_at": datetime.utcnow().isoformat(),
                "content_tags": [tag.value for tag in content_tags],
                "variant_status": VariantStatus.ACTIVE.value,
                "version": "1.0"
            })
            
            # Create variant
            variant = ContentVariantDB(
                content_id=content_id,
                chapter_id=chapter_id,
                section_id=section_id,
                variant_type=variant_type,
                target_audience=target_audience,
                content=content,
                content_metadata=metadata,
                is_ai_generated=is_ai_generated,
                quality_score=self._calculate_initial_quality_score(content, content_tags),
                usage_count=0
            )
            
            db.add(variant)
            db.commit()
            db.refresh(variant)
            
            logger.info(f"Created content variant {content_id}")
            return variant
            
        except Exception as e:
            logger.error(f"Error creating content variant: {e}")
            db.rollback()
            return None
    
    async def get_variant(
        self,
        db: Session,
        content_id: str
    ) -> Optional[ContentVariantDB]:
        """Get a specific content variant by ID."""
        try:
            variant = db.query(ContentVariantDB).filter(
                ContentVariantDB.content_id == content_id
            ).first()
            
            if variant:
                # Update usage count
                variant.usage_count += 1
                db.commit()
            
            return variant
            
        except Exception as e:
            logger.error(f"Error getting content variant {content_id}: {e}")
            return None
    
    async def get_variants_for_chapter(
        self,
        db: Session,
        chapter_id: str,
        section_id: Optional[str] = None,
        variant_type: Optional[str] = None,
        active_only: bool = True
    ) -> List[ContentVariantDB]:
        """Get all variants for a chapter/section."""
        try:
            query = db.query(ContentVariantDB).filter(
                ContentVariantDB.chapter_id == chapter_id
            )
            
            if section_id:
                query = query.filter(ContentVariantDB.section_id == section_id)
            
            if variant_type:
                query = query.filter(ContentVariantDB.variant_type == variant_type)
            
            if active_only:
                query = query.filter(
                    ContentVariantDB.content_metadata['variant_status'].astext == VariantStatus.ACTIVE.value
                )
            
            variants = query.order_by(desc(ContentVariantDB.created_at)).all()
            return variants
            
        except Exception as e:
            logger.error(f"Error getting variants for chapter {chapter_id}: {e}")
            return []
    
    async def find_best_variant(
        self,
        db: Session,
        chapter_id: str,
        section_id: Optional[str],
        user_preferences: Dict[str, Any]
    ) -> Optional[ContentVariantDB]:
        """Find the best matching variant for user preferences."""
        try:
            # Get all active variants for the chapter/section
            variants = await self.get_variants_for_chapter(
                db, chapter_id, section_id, active_only=True
            )
            
            if not variants:
                return None
            
            best_variant = None
            best_score = 0.0
            
            for variant in variants:
                # Calculate match score
                score = self._calculate_variant_match_score(variant, user_preferences)
                
                if score > best_score:
                    best_score = score
                    best_variant = variant
            
            return best_variant
            
        except Exception as e:
            logger.error(f"Error finding best variant: {e}")
            return None
    
    async def update_variant(
        self,
        db: Session,
        content_id: str,
        updates: Dict[str, Any]
    ) -> Optional[ContentVariantDB]:
        """Update an existing content variant."""
        try:
            variant = db.query(ContentVariantDB).filter(
                ContentVariantDB.content_id == content_id
            ).first()
            
            if not variant:
                return None
            
            # Create new version if content is updated
            if "content" in updates:
                await self._create_version_backup(db, variant)
                
                # Update content and retag
                variant.content = updates["content"]
                content_tags = content_tagger.tag_content(variant.content)
                
                # Update metadata
                if not variant.content_metadata:
                    variant.content_metadata = {}
                
                variant.content_metadata.update({
                    "updated_at": datetime.utcnow().isoformat(),
                    "content_tags": [tag.value for tag in content_tags],
                    "version": self._increment_version(variant.content_metadata.get("version", "1.0"))
                })
                
                # Recalculate quality score
                variant.quality_score = self._calculate_initial_quality_score(
                    variant.content, content_tags
                )
            
            # Update other fields
            for key, value in updates.items():
                if key != "content" and hasattr(variant, key):
                    setattr(variant, key, value)
            
            variant.updated_at = datetime.utcnow()
            db.commit()
            
            logger.info(f"Updated content variant {content_id}")
            return variant
            
        except Exception as e:
            logger.error(f"Error updating content variant {content_id}: {e}")
            db.rollback()
            return None
    
    async def delete_variant(
        self,
        db: Session,
        content_id: str,
        soft_delete: bool = True
    ) -> bool:
        """Delete a content variant (soft delete by default)."""
        try:
            variant = db.query(ContentVariantDB).filter(
                ContentVariantDB.content_id == content_id
            ).first()
            
            if not variant:
                return False
            
            if soft_delete:
                # Soft delete by marking as archived
                if not variant.content_metadata:
                    variant.content_metadata = {}
                
                variant.content_metadata["variant_status"] = VariantStatus.ARCHIVED.value
                variant.content_metadata["archived_at"] = datetime.utcnow().isoformat()
                variant.updated_at = datetime.utcnow()
                db.commit()
            else:
                # Hard delete
                db.delete(variant)
                db.commit()
            
            logger.info(f"Deleted content variant {content_id} (soft={soft_delete})")
            return True
            
        except Exception as e:
            logger.error(f"Error deleting content variant {content_id}: {e}")
            db.rollback()
            return False
    
    async def get_variant_analytics(
        self,
        db: Session,
        chapter_id: Optional[str] = None,
        days: int = 30
    ) -> Dict[str, Any]:
        """Get analytics for content variants."""
        try:
            since_date = datetime.utcnow() - timedelta(days=days)
            
            query = db.query(ContentVariantDB).filter(
                ContentVariantDB.created_at >= since_date
            )
            
            if chapter_id:
                query = query.filter(ContentVariantDB.chapter_id == chapter_id)
            
            variants = query.all()
            
            analytics = {
                "total_variants": len(variants),
                "ai_generated_count": sum(1 for v in variants if v.is_ai_generated),
                "manual_count": sum(1 for v in variants if not v.is_ai_generated),
                "average_quality_score": sum(v.quality_score or 0 for v in variants) / len(variants) if variants else 0,
                "total_usage": sum(v.usage_count for v in variants),
                "variants_by_type": {},
                "variants_by_experience_level": {},
                "most_used_variants": []
            }
            
            # Group by variant type
            for variant in variants:
                variant_type = variant.variant_type
                analytics["variants_by_type"][variant_type] = analytics["variants_by_type"].get(variant_type, 0) + 1
            
            # Group by experience level
            for variant in variants:
                exp_level = variant.target_audience.get("experience_level", "unknown")
                analytics["variants_by_experience_level"][exp_level] = analytics["variants_by_experience_level"].get(exp_level, 0) + 1
            
            # Most used variants
            sorted_variants = sorted(variants, key=lambda v: v.usage_count, reverse=True)
            analytics["most_used_variants"] = [
                {
                    "content_id": v.content_id,
                    "chapter_id": v.chapter_id,
                    "usage_count": v.usage_count,
                    "quality_score": v.quality_score
                }
                for v in sorted_variants[:10]
            ]
            
            return analytics
            
        except Exception as e:
            logger.error(f"Error getting variant analytics: {e}")
            return {}
    
    async def cleanup_old_variants(
        self,
        db: Session,
        days_old: int = 90,
        keep_minimum: int = 5
    ) -> int:
        """Clean up old, unused variants."""
        try:
            cutoff_date = datetime.utcnow() - timedelta(days=days_old)
            
            # Find old variants with low usage
            old_variants = db.query(ContentVariantDB).filter(
                and_(
                    ContentVariantDB.created_at < cutoff_date,
                    ContentVariantDB.usage_count < 5,
                    ContentVariantDB.content_metadata['variant_status'].astext == VariantStatus.ACTIVE.value
                )
            ).all()
            
            # Group by chapter to ensure we keep minimum variants per chapter
            variants_by_chapter = {}
            for variant in old_variants:
                key = f"{variant.chapter_id}_{variant.section_id or 'main'}"
                if key not in variants_by_chapter:
                    variants_by_chapter[key] = []
                variants_by_chapter[key].append(variant)
            
            cleaned_count = 0
            
            for chapter_variants in variants_by_chapter.values():
                # Keep the most recent and highest quality variants
                chapter_variants.sort(key=lambda v: (v.quality_score or 0, v.created_at), reverse=True)
                
                # Archive old variants beyond the minimum
                for variant in chapter_variants[keep_minimum:]:
                    await self.delete_variant(db, variant.content_id, soft_delete=True)
                    cleaned_count += 1
            
            logger.info(f"Cleaned up {cleaned_count} old content variants")
            return cleaned_count
            
        except Exception as e:
            logger.error(f"Error cleaning up old variants: {e}")
            return 0
    
    def _generate_content_id(self, chapter_id: str, section_id: Optional[str], variant_type: str) -> str:
        """Generate unique content ID."""
        timestamp = int(datetime.utcnow().timestamp())
        section_part = f"_{section_id}" if section_id else ""
        return f"{chapter_id}{section_part}_{variant_type}_{timestamp}"
    
    def _calculate_initial_quality_score(
        self,
        content: Dict[str, Any],
        content_tags: set
    ) -> int:
        """Calculate initial quality score for content."""
        score = 50  # Base score
        
        # Content completeness
        if "text" in content and len(content["text"]) > 100:
            score += 10
        
        if "code_examples" in content and content["code_examples"]:
            score += 15
        
        if "explanations" in content and content["explanations"]:
            score += 10
        
        if "learning_objectives" in content and content["learning_objectives"]:
            score += 10
        
        # Content tags quality
        if ContentTag.PRACTICAL in content_tags:
            score += 5
        
        if ContentTag.CODE_HEAVY in content_tags:
            score += 5
        
        if ContentTag.STEP_BY_STEP in content_tags:
            score += 5
        
        return min(score, 100)  # Cap at 100
    
    def _calculate_variant_match_score(
        self,
        variant: ContentVariantDB,
        user_preferences: Dict[str, Any]
    ) -> float:
        """Calculate how well a variant matches user preferences."""
        try:
            score = 0.0
            
            # Experience level match
            target_exp = variant.target_audience.get("experience_level")
            user_exp = user_preferences.get("experience_level")
            
            if target_exp and user_exp:
                if target_exp == user_exp:
                    score += 0.3
                elif abs(self._experience_level_to_int(target_exp) - self._experience_level_to_int(user_exp)) <= 1:
                    score += 0.2
            
            # Software categories match
            target_software = variant.target_audience.get("software_categories", [])
            user_software = user_preferences.get("software_categories", [])
            
            if target_software and user_software:
                common_software = set(target_software) & set(user_software)
                if common_software:
                    score += 0.25 * (len(common_software) / max(len(target_software), len(user_software)))
            
            # Hardware categories match
            target_hardware = variant.target_audience.get("hardware_categories", [])
            user_hardware = user_preferences.get("hardware_categories", [])
            
            if target_hardware and user_hardware:
                common_hardware = set(target_hardware) & set(user_hardware)
                if common_hardware:
                    score += 0.25 * (len(common_hardware) / max(len(target_hardware), len(user_hardware)))
            
            # Programming languages match
            target_langs = variant.target_audience.get("preferred_languages", [])
            user_langs = user_preferences.get("preferred_languages", [])
            
            if target_langs and user_langs:
                common_langs = set(target_langs) & set(user_langs)
                if common_langs:
                    score += 0.2 * (len(common_langs) / max(len(target_langs), len(user_langs)))
            
            # Quality score bonus
            quality_bonus = (variant.quality_score or 50) / 500  # Max 0.2 bonus
            score += quality_bonus
            
            # Usage popularity bonus
            usage_bonus = min(variant.usage_count / 100, 0.1)  # Max 0.1 bonus
            score += usage_bonus
            
            return min(score, 1.0)
            
        except Exception as e:
            logger.error(f"Error calculating variant match score: {e}")
            return 0.0
    
    def _experience_level_to_int(self, level: str) -> int:
        """Convert experience level to integer for comparison."""
        mapping = {
            "beginner": 1,
            "intermediate": 2,
            "advanced": 3,
            "expert": 4
        }
        return mapping.get(level.lower(), 2)
    
    async def _create_version_backup(self, db: Session, variant: ContentVariantDB) -> None:
        """Create a backup version of the variant before updating."""
        try:
            backup_id = f"{variant.content_id}_backup_{int(datetime.utcnow().timestamp())}"
            
            backup_metadata = variant.content_metadata.copy() if variant.content_metadata else {}
            backup_metadata.update({
                "is_backup": True,
                "original_id": variant.content_id,
                "backup_created_at": datetime.utcnow().isoformat()
            })
            
            backup_variant = ContentVariantDB(
                content_id=backup_id,
                chapter_id=variant.chapter_id,
                section_id=variant.section_id,
                variant_type=f"{variant.variant_type}_backup",
                target_audience=variant.target_audience,
                content=variant.content,
                content_metadata=backup_metadata,
                is_ai_generated=variant.is_ai_generated,
                quality_score=variant.quality_score,
                usage_count=0
            )
            
            db.add(backup_variant)
            db.commit()
            
        except Exception as e:
            logger.error(f"Error creating version backup: {e}")
    
    def _increment_version(self, current_version: str) -> str:
        """Increment version number."""
        try:
            parts = current_version.split(".")
            major, minor = int(parts[0]), int(parts[1]) if len(parts) > 1 else 0
            return f"{major}.{minor + 1}"
        except:
            return "1.1"


# Global variant manager instance
variant_manager = ContentVariantManager()