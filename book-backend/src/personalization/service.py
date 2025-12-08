"""
Core personalization service for content adaptation.
"""

import logging
import time
import hashlib
from typing import Dict, Any, Optional, List
from datetime import datetime, timedelta

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_

from .models import (
    UserPreferences, ContentVariant, PersonalizationRequest, PersonalizationResponse,
    ExperienceLevel, ContentComplexity
)
from .content_adapter import content_adaptation_engine, ContentType
from .rules_engine import rules_engine
from .cache_manager import cache_manager, PersonalizationCacheKeys, performance_optimizer
from app.models.database import UserPreferences as UserPreferencesDB
from app.models.database import ContentVariant as ContentVariantDB  
from app.models.database import PersonalizationLog as PersonalizationLogDB
from app.core.database import get_db

from app.qdrant.client import get_qdrant_manager
from app.qdrant.embedding_service import get_embedding_service

logger = logging.getLogger(__name__)


class PersonalizationService:
    """Core service for content personalization."""
    
    def __init__(self):
        # Use the advanced cache manager instead of simple dict
        self.cache_manager = cache_manager
        self.cache_ttl = 3600  # 1 hour cache TTL
        self.qdrant_manager = get_qdrant_manager()
        self.embedding_service = get_embedding_service()
        
    async def get_user_preferences(self, user_id: str, db: Session) -> Optional[UserPreferences]:
        """Get user preferences from database with caching."""
        try:
            # Check cache first
            cache_key = PersonalizationCacheKeys.user_preferences(user_id)
            cached_prefs = self.cache_manager.get(cache_key)
            
            if cached_prefs:
                return UserPreferences(**cached_prefs)
            
            # Get from database
            db_preferences = db.query(UserPreferencesDB).filter(
                UserPreferencesDB.user_id == user_id
            ).first()
            
            if not db_preferences:
                return None
            
            preferences = UserPreferences(
                user_id=db_preferences.user_id,
                software_background=db_preferences.software_background,
                hardware_background=db_preferences.hardware_background,
                content_complexity=db_preferences.content_complexity,
                explanation_depth=db_preferences.explanation_depth,
                example_style=db_preferences.example_style,
                created_at=db_preferences.created_at,
                updated_at=db_preferences.updated_at
            )
            
            # Cache the preferences
            self.cache_manager.set(
                cache_key,
                preferences.dict(),
                ttl=self.cache_ttl,
                tags=["user_preferences", f"user:{user_id}"]
            )
            
            return preferences
            
        except Exception as e:
            logger.error(f"Error getting user preferences for {user_id}: {e}")
            return None
    
    async def save_user_preferences(self, preferences: UserPreferences, db: Session) -> bool:
        """Save or update user preferences in database and invalidate cache, then upsert to Qdrant."""
        try:
            db_preferences = db.query(UserPreferencesDB).filter(
                UserPreferencesDB.user_id == preferences.user_id
            ).first()
            
            if db_preferences:
                # Update existing preferences
                db_preferences.software_background = preferences.software_background.dict()
                db_preferences.hardware_background = preferences.hardware_background.dict()
                db_preferences.content_complexity = preferences.content_complexity
                db_preferences.explanation_depth = preferences.explanation_depth
                db_preferences.example_style = preferences.example_style
                db_preferences.updated_at = datetime.utcnow()
            else:
                # Create new preferences
                db_preferences = UserPreferencesDB(
                    user_id=preferences.user_id,
                    software_background=preferences.software_background.dict(),
                    hardware_background=preferences.hardware_background.dict(),
                    content_complexity=preferences.content_complexity,
                    explanation_depth=preferences.explanation_depth,
                    example_style=preferences.example_style
                )
                db.add(db_preferences)
            
            db.commit()
            db.refresh(db_preferences) # Refresh to get the generated ID

            # Generate embedding and upsert to Qdrant
            user_embedding = await self.embedding_service.get_user_profile_embedding(preferences.dict())
            self.qdrant_manager.upsert_vectors(
                collection_name="user_profiles",
                ids=[db_preferences.id], # Use the DB ID as Qdrant ID
                vectors=[user_embedding],
                payloads=[{"user_id": preferences.user_id, "db_id": db_preferences.id}]
            )
            
            # Invalidate related cache entries
            self._invalidate_user_cache(preferences.user_id)
            
            return True
            
        except Exception as e:
            logger.error(f"Error saving user preferences for {preferences.user_id}: {e}")
            db.rollback()
            return False
    
    async def personalize_content(
        self, 
        request: PersonalizationRequest, 
        db: Session
    ) -> PersonalizationResponse:
        """Personalize content based on user preferences."""
        start_time = time.time()
        
        try:
            # Get user preferences
            preferences = await self.get_user_preferences(request.user_id, db)
            if not preferences:
                # Return default content if no preferences found
                return await self._get_default_content(request, start_time)
            
            # Check cache first
            cache_key = PersonalizationCacheKeys.personalized_content(
                request.user_id, request.chapter_id, request.section_id
            )
            
            if not request.force_regenerate:
                cached_content = self.cache_manager.get(cache_key)
                if cached_content:
                    return PersonalizationResponse(
                        content_id=cached_content["content_id"],
                        personalized_content=cached_content["content"],
                        personalization_applied=cached_content["personalization"],
                        cache_hit=True,
                        generation_time_ms=int((time.time() - start_time) * 1000)
                    )
            
            # Look for existing content variant
            content_variant = await self._find_content_variant(request, preferences, db)
            
            if content_variant:
                # Use existing variant
                personalized_content = content_variant.content
                personalization_applied = {
                    "variant_type": content_variant.variant_type,
                    "target_audience": content_variant.target_audience
                }
            else:
                # Generate new personalized content
                personalized_content, personalization_applied = await self._generate_personalized_content(
                    request, preferences, db
                )
            
            # Cache the result
            cache_data = {
                "content_id": f"{request.chapter_id}_{request.section_id or 'full'}",
                "content": personalized_content,
                "personalization": personalization_applied
            }
            
            # Cache with appropriate tags for invalidation
            cache_tags = [
                "personalized_content",
                f"user:{request.user_id}",
                f"chapter:{request.chapter_id}"
            ]
            if request.section_id:
                cache_tags.append(f"section:{request.section_id}")
            
            self.cache_manager.set(cache_key, cache_data, ttl=self.cache_ttl, tags=cache_tags)
            
            # Log the personalization request
            await self._log_personalization_request(request, personalization_applied, False, start_time, db)
            
            return PersonalizationResponse(
                content_id=cache_data["content_id"],
                personalized_content=personalized_content,
                personalization_applied=personalization_applied,
                cache_hit=False,
                generation_time_ms=int((time.time() - start_time) * 1000)
            )
            
        except Exception as e:
            logger.error(f"Error personalizing content: {e}")
            return await self._get_default_content(request, start_time)
    
    async def _find_content_variant_qdrant(
        self,
        request: PersonalizationRequest,
        preferences: UserPreferences,
        db: Session
    ) -> Optional[ContentVariantDB]:
        """Find existing content variant using Qdrant similarity search."""
        try:
            # 1. Generate query vector from user preferences
            user_query_vector = await self.embedding_service.get_user_profile_embedding(preferences.dict())

            # 2. Construct Qdrant filter
            # Filter by chapter_id and section_id (if present)
            qdrant_filter = {
                "must": [
                    {"key": "chapter_id", "match": {"value": request.chapter_id}},
                ]
            }
            if request.section_id:
                qdrant_filter["must"].append({"key": "section_id", "match": {"value": request.section_id}})

            # 3. Perform Qdrant search
            search_results = self.qdrant_manager.search_vectors(
                collection_name="content_variants",
                query_vector=user_query_vector,
                limit=1,  # We only need the top matching variant
                query_filter=qdrant_filter
            )

            if search_results:
                top_match = search_results[0]
                db_id = top_match.payload.get("db_id")
                if db_id:
                    # Retrieve the actual content variant from PostgreSQL using the db_id
                    db_variant = db.query(ContentVariantDB).filter(ContentVariantDB.id == db_id).first()
                    if db_variant:
                        logger.info(f"Found content variant via Qdrant: {db_variant.content_id}, score: {top_match.score}")
                        return db_variant
            
            logger.info("No suitable content variant found in Qdrant.")
            return None

        except Exception as e:
            logger.error(f"Error finding content variant via Qdrant: {e}")
            return None


    async def _find_content_variant(
        self, 
        request: PersonalizationRequest, 
        preferences: UserPreferences, 
        db: Session
    ) -> Optional[ContentVariantDB]:
        """Find existing content variant that matches user preferences, prioritizing Qdrant search."""
        
        # 1. Try Qdrant search first
        qdrant_variant = await self._find_content_variant_qdrant(request, preferences, db)
        if qdrant_variant:
            return qdrant_variant

        # 2. Fallback to SQL query if Qdrant doesn't find a suitable variant
        try:
            # Determine target experience level (for SQL fallback)
            target_level = self._determine_target_level(preferences)
            
            # Query for matching content variant in SQL
            variant = db.query(ContentVariantDB).filter(
                and_(
                    ContentVariantDB.chapter_id == request.chapter_id,
                    ContentVariantDB.section_id == request.section_id,
                    ContentVariantDB.variant_type == target_level
                )
            ).first()
            
            if variant:
                logger.info(f"Found content variant via SQL fallback: {variant.content_id}")
                return variant
            
            logger.info("No content variant found via SQL fallback.")
            return None
            
        except Exception as e:
            logger.error(f"Error finding content variant via SQL fallback: {e}")
            return None
    
    async def _generate_personalized_content(
        self, 
        request: PersonalizationRequest, 
        preferences: UserPreferences, 
        db: Session
    ) -> tuple[Dict[str, Any], Dict[str, Any]]:
        """Generate personalized content using the content adaptation engine."""
        try:
            # Get base content (this would normally come from the chapter/section)
            base_content = await self._get_base_content(request, db)
            
            # Use the content adaptation engine to personalize the content
            personalized_content = content_adaptation_engine.adapt_content(
                original_content=base_content,
                preferences=preferences,
                content_type=ContentType.TEXT
            )
            
            # Apply personalization rules for additional customization
            personalized_content, rule_results = rules_engine.execute_rules(
                content=personalized_content,
                user_preferences=preferences,
                context={"chapter_id": request.chapter_id, "section_id": request.section_id}
            )
            
            # Extract personalization metadata
            personalization_applied = personalized_content.get("adaptation_metadata", {})
            personalization_applied.update({
                "software_categories": [cat.value for cat in preferences.software_background.categories],
                "hardware_categories": [cat.value for cat in preferences.hardware_background.categories],
                "preferred_languages": preferences.software_background.preferred_languages,
                "content_complexity": preferences.content_complexity.value,
                "explanation_depth": preferences.explanation_depth,
                "example_style": preferences.example_style
            })
            
            # Save as new content variant for future use
            await self._save_content_variant(request, personalized_content, personalization_applied, db)
            
            return personalized_content, personalization_applied
            
        except Exception as e:
            logger.error(f"Error generating personalized content: {e}")
            return self._get_fallback_content(), {}
    
    async def _get_base_content(self, request: PersonalizationRequest, db: Session) -> Dict[str, Any]:
        """Get base content for the requested chapter/section."""
        try:
            # This is a placeholder - in a real implementation, this would fetch
            # the actual chapter content from the database
            base_content = {
                "title": f"Chapter {request.chapter_id}",
                "text": "This is the base content that will be personalized based on user preferences. It covers fundamental concepts and provides examples that can be adapted to different experience levels and technology preferences.",
                "learning_objectives": [
                    "Understand core concepts",
                    "Apply knowledge in practical scenarios",
                    "Integrate with existing knowledge"
                ],
                "key_concepts": [
                    "Fundamental principles",
                    "Best practices",
                    "Common patterns"
                ],
                "original_examples": [
                    {
                        "title": "Basic Example",
                        "description": "A simple demonstration of the concept"
                    }
                ]
            }
            
            if request.section_id:
                base_content["section"] = request.section_id
                base_content["text"] = f"Section {request.section_id}: " + base_content["text"]
            
            return base_content
            
        except Exception as e:
            logger.error(f"Error getting base content: {e}")
            return {
                "title": "Default Content",
                "text": "Default content when base content cannot be retrieved.",
                "learning_objectives": [],
                "key_concepts": [],
                "original_examples": []
            }
    
    def _determine_target_level(self, preferences: UserPreferences) -> str:
        """Determine target experience level based on user preferences."""
        # Use the higher of software or hardware experience levels
        software_level = preferences.software_background.experience_level
        hardware_level = preferences.hardware_background.experience_level
        
        level_order = [ExperienceLevel.BEGINNER, ExperienceLevel.INTERMEDIATE, 
                      ExperienceLevel.ADVANCED, ExperienceLevel.EXPERT]
        
        software_index = level_order.index(software_level)
        hardware_index = level_order.index(hardware_level)
        
        return level_order[max(software_index, hardware_index)]
    
    def _adapt_code_examples(self, preferences: UserPreferences) -> List[Dict[str, Any]]:
        """Adapt code examples based on user's preferred languages."""
        preferred_langs = preferences.software_background.preferred_languages
        
        if not preferred_langs:
            preferred_langs = ["python"]  # Default to Python
        
        examples = []
        for lang in preferred_langs[:3]:  # Limit to top 3 languages
            examples.append({
                "language": lang,
                "code": f"// Example code in {lang}\\n// Adapted for {preferences.software_background.experience_level} level",
                "explanation": f"This example demonstrates the concept using {lang}"
            })
        
        return examples
    
    def _adapt_explanations(self, preferences: UserPreferences) -> Dict[str, str]:
        """Adapt explanations based on user's experience level and preferences."""
        level = preferences.software_background.experience_level
        depth = preferences.explanation_depth
        
        explanations = {
            "overview": "High-level explanation suitable for quick understanding",
            "detailed": "In-depth explanation with technical details and background",
            "practical": "Hands-on explanation with practical examples and use cases"
        }
        
        if level == ExperienceLevel.BEGINNER:
            explanations["main"] = "Simplified explanation with basic concepts and terminology"
        elif level == ExperienceLevel.EXPERT:
            explanations["main"] = "Advanced explanation assuming deep technical knowledge"
        else:
            explanations["main"] = "Balanced explanation with moderate technical depth"
        
        return explanations
    
    def _adapt_hardware_references(self, preferences: UserPreferences) -> List[Dict[str, Any]]:
        """Adapt hardware references based on user's hardware background."""
        platforms = preferences.hardware_background.platforms
        categories = preferences.hardware_background.categories
        
        if not platforms:
            platforms = ["arduino", "raspberry_pi"]  # Default platforms
        
        references = []
        for platform in platforms[:3]:  # Limit to top 3 platforms
            references.append({
                "platform": platform,
                "description": f"Implementation details for {platform}",
                "complexity": preferences.hardware_background.experience_level
            })
        
        return references
    
    async def _save_content_variant(
        self, 
        request: PersonalizationRequest, 
        content: Dict[str, Any], 
        personalization: Dict[str, Any], 
        db: Session
    ) -> bool:
        """Save generated content as a new variant and upsert to Qdrant."""
        try:
            content_id = f"{request.chapter_id}_{request.section_id or 'full'}_{int(time.time())}"
            
            variant = ContentVariantDB(
                content_id=content_id,
                chapter_id=request.chapter_id,
                section_id=request.section_id,
                variant_type=personalization.get("experience_level", "intermediate"),
                target_audience=personalization,
                content=content,
                metadata={"generated_at": datetime.utcnow().isoformat()},
                is_ai_generated=True
            )
            
            db.add(variant)
            db.commit()
            db.refresh(variant) # Refresh to get the generated ID

            # Generate embedding and upsert to Qdrant
            content_variant_embedding = await self.embedding_service.get_content_variant_embedding(variant.dict())
            self.qdrant_manager.upsert_vectors(
                collection_name="content_variants",
                ids=[variant.id], # Use the DB ID as Qdrant ID
                vectors=[content_variant_embedding],
                payloads=[{"content_id": variant.content_id, "db_id": variant.id, "chapter_id": variant.chapter_id, "section_id": variant.section_id}]
            )

            return True
            
        except Exception as e:
            logger.error(f"Error saving content variant: {e}")
            db.rollback()
            return False
    
    async def _get_default_content(self, request: PersonalizationRequest, start_time: float) -> PersonalizationResponse:
        """Get default content when personalization fails."""
        return PersonalizationResponse(
            content_id=f"{request.chapter_id}_{request.section_id or 'full'}_default",
            personalized_content=self._get_fallback_content(),
            personalization_applied={"type": "default"},
            cache_hit=False,
            generation_time_ms=int((time.time() - start_time) * 1000)
        )
    
    def _get_fallback_content(self) -> Dict[str, Any]:
        """Get fallback content when personalization fails."""
        return {
            "text": "Default content - personalization not available",
            "complexity_level": "intermediate",
            "code_examples": [{"language": "python", "code": "# Default example"}],
            "explanations": {"main": "Standard explanation"},
            "hardware_references": []
        }
    
    def _invalidate_user_cache(self, user_id: str) -> None:
        """Invalidate all cache entries for a user."""
        try:
            # Invalidate user preferences
            cache_key = PersonalizationCacheKeys.user_preferences(user_id)
            self.cache_manager.delete(cache_key)
            
            # Invalidate user's personalized content
            self.cache_manager.invalidate_by_tags([f"user:{user_id}"])
            
            logger.info(f"Invalidated cache for user {user_id}")
            
        except Exception as e:
            logger.error(f"Error invalidating cache for user {user_id}: {e}")
    
    async def get_cache_stats(self) -> Dict[str, Any]:
        """Get cache performance statistics."""
        try:
            return self.cache_manager.get_stats()
        except Exception as e:
            logger.error(f"Error getting cache stats: {e}")
            return {}
    
    async def warm_cache_for_popular_content(
        self,
        popular_chapters: List[str],
        common_preferences: List[Dict[str, Any]]
    ) -> int:
        """Warm cache with popular content combinations."""
        try:
            return performance_optimizer.pre_generate_common_content(
                common_preferences, popular_chapters
            )
        except Exception as e:
            logger.error(f"Error warming cache: {e}")
            return 0
    
    async def cleanup_expired_cache(self) -> int:
        """Clean up expired cache entries."""
        try:
            return self.cache_manager.cleanup_expired()
        except Exception as e:
            logger.error(f"Error cleaning up cache: {e}")
            return 0
    
    async def _log_personalization_request(
        self, 
        request: PersonalizationRequest, 
        personalization: Dict[str, Any], 
        cache_hit: bool, 
        start_time: float, 
        db: Session
    ) -> None:
        """Log personalization request for analytics."""
        try:
            log_entry = PersonalizationLogDB(
                user_id=request.user_id,
                chapter_id=request.chapter_id,
                section_id=request.section_id,
                requested_complexity=request.requested_complexity,
                applied_personalization=personalization,
                cache_hit=cache_hit,
                generation_time_ms=int((time.time() - start_time) * 1000)
            )
            
            db.add(log_entry)
            db.commit()
            
        except Exception as e:
            logger.error(f"Error logging personalization request: {e}")
            db.rollback()
    
    async def get_personalization_stats(self, db: Session) -> Dict[str, Any]:
        """Get personalization usage statistics."""
        try:
            # Get basic stats from logs
            total_requests = db.query(PersonalizationLogDB).count()
            
            if total_requests == 0:
                return {
                    "total_requests": 0,
                    "cache_hit_rate": 0.0,
                    "average_generation_time_ms": 0.0,
                    "popular_preferences": {},
                    "content_variants_count": 0
                }
            
            # Calculate cache hit rate
            cache_hits = db.query(PersonalizationLogDB).filter(
                PersonalizationLogDB.cache_hit == True
            ).count()
            cache_hit_rate = cache_hits / total_requests
            
            # Calculate average generation time
            avg_time_result = db.query(
                db.func.avg(PersonalizationLogDB.generation_time_ms)
            ).scalar()
            avg_generation_time = float(avg_time_result) if avg_time_result else 0.0
            
            # Get content variants count
            variants_count = db.query(ContentVariantDB).count()
            
            return {
                "total_requests": total_requests,
                "cache_hit_rate": cache_hit_rate,
                "average_generation_time_ms": avg_generation_time,
                "popular_preferences": {},  # TODO: Implement preference analysis
                "content_variants_count": variants_count
            }
            
        except Exception as e:
            logger.error(f"Error getting personalization stats: {e}")
            return {
                "total_requests": 0,
                "cache_hit_rate": 0.0,
                "average_generation_time_ms": 0.0,
                "popular_preferences": {},
                "content_variants_count": 0
            }
    
    async def validate_user_preferences(self, preferences: UserPreferences) -> Dict[str, Any]:
        """Validate user preferences and return validation results."""
        try:
            validation_result = {
                "valid": True,
                "errors": [],
                "warnings": [],
                "suggestions": []
            }
            
            # Validate software background
            if preferences.software_background:
                if len(preferences.software_background.categories) > 10:
                    validation_result["warnings"].append("Too many software categories selected - consider focusing on your main areas")
                
                if len(preferences.software_background.preferred_languages) > 8:
                    validation_result["warnings"].append("Too many programming languages selected - consider your top 3-5 languages")
                
                # Check for conflicting experience levels
                if (preferences.software_background.experience_level == ExperienceLevel.BEGINNER and 
                    len(preferences.software_background.preferred_languages) > 3):
                    validation_result["suggestions"].append("As a beginner, consider starting with 1-2 programming languages")
            
            # Validate hardware background
            if preferences.hardware_background:
                if len(preferences.hardware_background.categories) > 8:
                    validation_result["warnings"].append("Too many hardware categories selected - consider your main focus areas")
                
                if len(preferences.hardware_background.platforms) > 6:
                    validation_result["warnings"].append("Too many hardware platforms selected - consider your primary platforms")
                
                # Check for experience level consistency
                if (preferences.hardware_background.experience_level == ExperienceLevel.BEGINNER and 
                    len(preferences.hardware_background.platforms) > 2):
                    validation_result["suggestions"].append("As a hardware beginner, consider starting with 1-2 platforms like Arduino or Raspberry Pi")
            
            # Validate content preferences consistency
            if (preferences.content_complexity == ContentComplexity.SIMPLE and 
                preferences.explanation_depth == "overview"):
                validation_result["suggestions"].append("Consider 'detailed' explanations with simple content for better learning")
            
            if (preferences.content_complexity == ContentComplexity.COMPREHENSIVE and 
                preferences.explanation_depth == "overview"):
                validation_result["warnings"].append("Comprehensive content with overview explanations may be inconsistent")
            
            return validation_result
            
        except Exception as e:
            logger.error(f"Error validating user preferences: {e}")
            return {
                "valid": False,
                "errors": [f"Validation error: {str(e)}"],
                "warnings": [],
                "suggestions": []
            }
    
    async def sanitize_preferences(self, preferences: UserPreferences) -> UserPreferences:
        """Sanitize and normalize user preferences."""
        try:
            # Limit the number of categories and languages to reasonable amounts
            if len(preferences.software_background.categories) > 10:
                preferences.software_background.categories = preferences.software_background.categories[:10]
            
            if len(preferences.software_background.preferred_languages) > 8:
                preferences.software_background.preferred_languages = preferences.software_background.preferred_languages[:8]
            
            if len(preferences.hardware_background.categories) > 8:
                preferences.hardware_background.categories = preferences.hardware_background.categories[:8]
            
            if len(preferences.hardware_background.platforms) > 6:
                preferences.hardware_background.platforms = preferences.hardware_background.platforms[:6]
            
            # Normalize string values to lowercase
            preferences.software_background.preferred_languages = [
                lang.lower().strip() for lang in preferences.software_background.preferred_languages
            ]
            
            preferences.hardware_background.platforms = [
                platform.lower().strip().replace(" ", "_") for platform in preferences.hardware_background.platforms
            ]
            
            return preferences
            
        except Exception as e:
            logger.error(f"Error sanitizing preferences: {e}")
            return preferences
    
    async def get_preference_conflicts(self, preferences: UserPreferences) -> List[Dict[str, str]]:
        """Identify potential conflicts in user preferences."""
        try:
            conflicts = []
            
            # Check for experience level conflicts
            if (preferences.software_background.experience_level != preferences.hardware_background.experience_level):
                level_diff = abs(
                    [ExperienceLevel.BEGINNER, ExperienceLevel.INTERMEDIATE, ExperienceLevel.ADVANCED, ExperienceLevel.EXPERT].index(preferences.software_background.experience_level) -
                    [ExperienceLevel.BEGINNER, ExperienceLevel.INTERMEDIATE, ExperienceLevel.ADVANCED, ExperienceLevel.EXPERT].index(preferences.hardware_background.experience_level)
                )
                
                if level_diff > 1:
                    conflicts.append({
                        "type": "experience_mismatch",
                        "description": f"Large gap between software ({preferences.software_background.experience_level}) and hardware ({preferences.hardware_background.experience_level}) experience levels",
                        "suggestion": "Consider adjusting experience levels to be more consistent"
                    })
            
            # Check for category overlaps
            software_embedded = "embedded_systems" in [cat.value for cat in preferences.software_background.categories]
            hardware_embedded = "embedded_systems" in [cat.value for cat in preferences.hardware_background.categories]
            
            if software_embedded and not hardware_embedded:
                conflicts.append({
                    "type": "category_mismatch",
                    "description": "Software embedded systems selected but no hardware embedded systems category",
                    "suggestion": "Consider adding embedded systems to hardware categories"
                })
            
            return conflicts
            
        except Exception as e:
            logger.error(f"Error checking preference conflicts: {e}")
            return []


# Global service instance
personalization_service = PersonalizationService()