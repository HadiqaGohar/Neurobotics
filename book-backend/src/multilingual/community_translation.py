"""Community Translation Contribution System."""

import logging
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
import json

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, desc

from app.models.multilingual import (
    ContentTranslation, TranslationMemory, TerminologyGlossary,
    TranslationStatus, TranslationMethod, Language
)
from app.models.database import User

logger = logging.getLogger(__name__)


class ContributionType(str, Enum):
    """Types of community contributions."""
    TRANSLATION = "translation"
    REVIEW = "review"
    CORRECTION = "correction"
    TERMINOLOGY = "terminology"
    SUGGESTION = "suggestion"


class ContributionStatus(str, Enum):
    """Status of community contributions."""
    PENDING = "pending"
    UNDER_REVIEW = "under_review"
    APPROVED = "approved"
    REJECTED = "rejected"
    IMPLEMENTED = "implemented"


class ReputationAction(str, Enum):
    """Actions that affect user reputation."""
    TRANSLATION_SUBMITTED = "translation_submitted"
    TRANSLATION_APPROVED = "translation_approved"
    TRANSLATION_REJECTED = "translation_rejected"
    REVIEW_SUBMITTED = "review_submitted"
    REVIEW_HELPFUL = "review_helpful"
    TERMINOLOGY_ADDED = "terminology_added"
    CORRECTION_ACCEPTED = "correction_accepted"


class CommunityTranslationSystem:
    """System for managing community translation contributions."""
    
    def __init__(self):
        # Reputation scoring system
        self.reputation_scores = {
            ReputationAction.TRANSLATION_SUBMITTED: 5,
            ReputationAction.TRANSLATION_APPROVED: 20,
            ReputationAction.TRANSLATION_REJECTED: -5,
            ReputationAction.REVIEW_SUBMITTED: 3,
            ReputationAction.REVIEW_HELPFUL: 10,
            ReputationAction.TERMINOLOGY_ADDED: 15,
            ReputationAction.CORRECTION_ACCEPTED: 8
        }
        
        # User levels based on reputation
        self.user_levels = {
            "Beginner": (0, 49),
            "Contributor": (50, 199),
            "Translator": (200, 499),
            "Expert": (500, 999),
            "Master": (1000, float('inf'))
        }
        
        # Privileges based on user level
        self.level_privileges = {
            "Beginner": ["submit_suggestions", "vote_on_translations"],
            "Contributor": ["submit_translations", "review_translations"],
            "Translator": ["approve_suggestions", "edit_terminology"],
            "Expert": ["moderate_content", "mentor_users"],
            "Master": ["admin_privileges", "system_configuration"]
        }
    
    async def submit_translation_contribution(
        self,
        content_type: str,
        content_id: str,
        language_code: str,
        title: Optional[str],
        content: Optional[str],
        contributor_id: int,
        notes: Optional[str] = None,
        db: Session = None
    ) -> Dict[str, Any]:
        """Submit a community translation contribution."""
        try:
            # Check if user has permission to contribute
            user = db.query(User).filter(User.id == contributor_id).first()
            if not user:
                return {"success": False, "error": "User not found"}
            
            user_level = await self._get_user_level(contributor_id, db)
            if "submit_translations" not in self.level_privileges.get(user_level, []):
                return {"success": False, "error": "Insufficient privileges to submit translations"}
            
            # Check if translation already exists
            existing = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.content_type == content_type,
                    ContentTranslation.content_id == content_id,
                    ContentTranslation.language_code == language_code
                )
            ).first()
            
            if existing and existing.translation_status in [
                TranslationStatus.PUBLISHED, TranslationStatus.APPROVED
            ]:
                # Create a suggestion for improvement instead
                return await self._create_translation_suggestion(
                    existing.id, title, content, contributor_id, notes, db
                )
            
            # Create new translation contribution
            contribution_metadata = {
                "contribution_type": ContributionType.TRANSLATION.value,
                "contributor_id": contributor_id,
                "contribution_status": ContributionStatus.PENDING.value,
                "submitted_at": datetime.utcnow().isoformat(),
                "notes": notes,
                "community_contribution": True
            }
            
            if existing:
                # Update existing translation
                existing.title = title
                existing.content = content
                existing.translator_id = contributor_id
                existing.translation_method = TranslationMethod.MANUAL
                existing.translation_status = TranslationStatus.PENDING
                existing.metadata = {**(existing.metadata or {}), **contribution_metadata}
                existing.updated_at = datetime.utcnow()
                
                translation_id = existing.id
            else:
                # Create new translation
                translation = ContentTranslation(
                    content_type=content_type,
                    content_id=content_id,
                    language_code=language_code,
                    title=title,
                    content=content,
                    translator_id=contributor_id,
                    translation_method=TranslationMethod.MANUAL,
                    translation_status=TranslationStatus.PENDING,
                    metadata=contribution_metadata
                )
                
                db.add(translation)
                db.flush()
                translation_id = translation.id
            
            # Award reputation points
            await self._award_reputation(
                contributor_id, ReputationAction.TRANSLATION_SUBMITTED, db
            )
            
            # Create contribution record
            contribution_record = {
                "id": translation_id,
                "type": ContributionType.TRANSLATION.value,
                "status": ContributionStatus.PENDING.value,
                "contributor_id": contributor_id,
                "content_reference": f"{content_type}:{content_id}:{language_code}",
                "submitted_at": datetime.utcnow().isoformat()
            }
            
            db.commit()
            
            return {
                "success": True,
                "contribution_id": translation_id,
                "status": ContributionStatus.PENDING.value,
                "message": "Translation contribution submitted successfully",
                "contribution_record": contribution_record
            }
            
        except Exception as e:
            logger.error(f"Error submitting translation contribution: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def submit_review(
        self,
        translation_id: int,
        reviewer_id: int,
        rating: int,  # 1-5 stars
        feedback: str,
        suggested_improvements: Optional[Dict[str, str]] = None,
        db: Session = None
    ) -> Dict[str, Any]:
        """Submit a review for a translation."""
        try:
            # Validate rating
            if not 1 <= rating <= 5:
                return {"success": False, "error": "Rating must be between 1 and 5"}
            
            # Check if user has permission to review
            user_level = await self._get_user_level(reviewer_id, db)
            if "review_translations" not in self.level_privileges.get(user_level, []):
                return {"success": False, "error": "Insufficient privileges to review translations"}
            
            # Get translation
            translation = db.query(ContentTranslation).filter(
                ContentTranslation.id == translation_id
            ).first()
            
            if not translation:
                return {"success": False, "error": "Translation not found"}
            
            # Check if user is not reviewing their own translation
            if translation.translator_id == reviewer_id:
                return {"success": False, "error": "Cannot review your own translation"}
            
            # Create review record
            review_data = {
                "reviewer_id": reviewer_id,
                "rating": rating,
                "feedback": feedback,
                "suggested_improvements": suggested_improvements or {},
                "submitted_at": datetime.utcnow().isoformat()
            }
            
            # Update translation metadata with review
            metadata = translation.metadata or {}
            reviews = metadata.get("reviews", [])
            reviews.append(review_data)
            metadata["reviews"] = reviews
            
            # Calculate average rating
            total_rating = sum(review["rating"] for review in reviews)
            average_rating = total_rating / len(reviews)
            metadata["average_rating"] = average_rating
            metadata["review_count"] = len(reviews)
            
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            # Award reputation points
            await self._award_reputation(
                reviewer_id, ReputationAction.REVIEW_SUBMITTED, db
            )
            
            # If rating is high, consider for auto-approval
            if average_rating >= 4.0 and len(reviews) >= 2:
                translation.translation_status = TranslationStatus.APPROVED
                
                # Award bonus points to translator
                if translation.translator_id:
                    await self._award_reputation(
                        translation.translator_id, ReputationAction.TRANSLATION_APPROVED, db
                    )
            
            db.commit()
            
            return {
                "success": True,
                "review_id": len(reviews),
                "average_rating": average_rating,
                "review_count": len(reviews),
                "message": "Review submitted successfully"
            }
            
        except Exception as e:
            logger.error(f"Error submitting review: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def submit_correction(
        self,
        translation_id: int,
        corrector_id: int,
        corrections: Dict[str, str],  # {"field": "corrected_value"}
        explanation: str,
        db: Session = None
    ) -> Dict[str, Any]:
        """Submit corrections for a translation."""
        try:
            # Get translation
            translation = db.query(ContentTranslation).filter(
                ContentTranslation.id == translation_id
            ).first()
            
            if not translation:
                return {"success": False, "error": "Translation not found"}
            
            # Create correction record
            correction_data = {
                "corrector_id": corrector_id,
                "corrections": corrections,
                "explanation": explanation,
                "status": ContributionStatus.PENDING.value,
                "submitted_at": datetime.utcnow().isoformat()
            }
            
            # Update translation metadata
            metadata = translation.metadata or {}
            corrections_list = metadata.get("corrections", [])
            corrections_list.append(correction_data)
            metadata["corrections"] = corrections_list
            
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            db.commit()
            
            return {
                "success": True,
                "correction_id": len(corrections_list),
                "status": ContributionStatus.PENDING.value,
                "message": "Correction submitted successfully"
            }
            
        except Exception as e:
            logger.error(f"Error submitting correction: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def submit_terminology(
        self,
        term: str,
        translation: str,
        source_language: str,
        target_language: str,
        definition: Optional[str],
        context: Optional[str],
        domain: str,
        contributor_id: int,
        db: Session = None
    ) -> Dict[str, Any]:
        """Submit terminology contribution."""
        try:
            # Check if terminology already exists
            existing = db.query(TerminologyGlossary).filter(
                and_(
                    TerminologyGlossary.term == term,
                    TerminologyGlossary.source_language == source_language,
                    TerminologyGlossary.target_language == target_language
                )
            ).first()
            
            if existing:
                return {"success": False, "error": "Terminology already exists"}
            
            # Create terminology entry
            terminology = TerminologyGlossary(
                term=term,
                translation=translation,
                source_language=source_language,
                target_language=target_language,
                definition=definition,
                context=context,
                domain=domain,
                approved=False,  # Requires approval
                created_by=contributor_id
            )
            
            db.add(terminology)
            db.commit()
            db.refresh(terminology)
            
            # Award reputation points
            await self._award_reputation(
                contributor_id, ReputationAction.TERMINOLOGY_ADDED, db
            )
            
            return {
                "success": True,
                "terminology_id": terminology.id,
                "status": "pending_approval",
                "message": "Terminology submitted successfully"
            }
            
        except Exception as e:
            logger.error(f"Error submitting terminology: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def vote_on_contribution(
        self,
        contribution_id: int,
        contribution_type: ContributionType,
        voter_id: int,
        vote: str,  # "upvote" or "downvote"
        db: Session = None
    ) -> Dict[str, Any]:
        """Vote on a community contribution."""
        try:
            if vote not in ["upvote", "downvote"]:
                return {"success": False, "error": "Invalid vote type"}
            
            # Get the contribution based on type
            if contribution_type == ContributionType.TRANSLATION:
                contribution = db.query(ContentTranslation).filter(
                    ContentTranslation.id == contribution_id
                ).first()
            else:
                return {"success": False, "error": "Unsupported contribution type for voting"}
            
            if not contribution:
                return {"success": False, "error": "Contribution not found"}
            
            # Check if user already voted
            metadata = contribution.metadata or {}
            votes = metadata.get("votes", {})
            
            if str(voter_id) in votes:
                return {"success": False, "error": "You have already voted on this contribution"}
            
            # Record vote
            votes[str(voter_id)] = {
                "vote": vote,
                "timestamp": datetime.utcnow().isoformat()
            }
            
            # Calculate vote counts
            upvotes = sum(1 for v in votes.values() if v["vote"] == "upvote")
            downvotes = sum(1 for v in votes.values() if v["vote"] == "downvote")
            
            metadata["votes"] = votes
            metadata["upvotes"] = upvotes
            metadata["downvotes"] = downvotes
            metadata["vote_score"] = upvotes - downvotes
            
            contribution.metadata = metadata
            contribution.updated_at = datetime.utcnow()
            
            db.commit()
            
            return {
                "success": True,
                "upvotes": upvotes,
                "downvotes": downvotes,
                "vote_score": upvotes - downvotes,
                "message": f"Vote recorded successfully"
            }
            
        except Exception as e:
            logger.error(f"Error voting on contribution: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def get_user_contributions(
        self,
        user_id: int,
        contribution_type: Optional[ContributionType] = None,
        limit: int = 50,
        offset: int = 0,
        db: Session = None
    ) -> Dict[str, Any]:
        """Get user's contributions."""
        try:
            contributions = []
            
            # Get translations
            if not contribution_type or contribution_type == ContributionType.TRANSLATION:
                translations = db.query(ContentTranslation).filter(
                    ContentTranslation.translator_id == user_id
                ).offset(offset).limit(limit).all()
                
                for translation in translations:
                    metadata = translation.metadata or {}
                    contributions.append({
                        "id": translation.id,
                        "type": ContributionType.TRANSLATION.value,
                        "content_reference": f"{translation.content_type}:{translation.content_id}",
                        "language": translation.language_code,
                        "title": translation.title,
                        "status": translation.translation_status.value,
                        "created_at": translation.created_at.isoformat(),
                        "updated_at": translation.updated_at.isoformat(),
                        "votes": metadata.get("vote_score", 0),
                        "reviews": metadata.get("review_count", 0),
                        "average_rating": metadata.get("average_rating", 0)
                    })
            
            # Get terminology contributions
            if not contribution_type or contribution_type == ContributionType.TERMINOLOGY:
                terminology_entries = db.query(TerminologyGlossary).filter(
                    TerminologyGlossary.created_by == user_id
                ).offset(offset).limit(limit).all()
                
                for entry in terminology_entries:
                    contributions.append({
                        "id": entry.id,
                        "type": ContributionType.TERMINOLOGY.value,
                        "term": entry.term,
                        "translation": entry.translation,
                        "domain": entry.domain,
                        "status": "approved" if entry.approved else "pending",
                        "created_at": entry.created_at.isoformat(),
                        "updated_at": entry.updated_at.isoformat()
                    })
            
            # Sort by creation date
            contributions.sort(key=lambda x: x["created_at"], reverse=True)
            
            return {
                "contributions": contributions[:limit],
                "total_count": len(contributions),
                "user_stats": await self._get_user_stats(user_id, db)
            }
            
        except Exception as e:
            logger.error(f"Error getting user contributions: {e}")
            return {"contributions": [], "total_count": 0, "error": str(e)}
    
    async def get_leaderboard(
        self,
        period: str = "all_time",  # "week", "month", "year", "all_time"
        limit: int = 20,
        db: Session = None
    ) -> Dict[str, Any]:
        """Get community leaderboard."""
        try:
            # Calculate date range
            end_date = datetime.utcnow()
            if period == "week":
                start_date = end_date - timedelta(weeks=1)
            elif period == "month":
                start_date = end_date - timedelta(days=30)
            elif period == "year":
                start_date = end_date - timedelta(days=365)
            else:
                start_date = datetime.min
            
            # Get top contributors
            leaderboard = []
            
            # Query users with their contribution counts
            user_stats = db.query(
                User.id,
                User.username,
                User.full_name,
                func.count(ContentTranslation.id).label('translation_count')
            ).outerjoin(
                ContentTranslation,
                and_(
                    ContentTranslation.translator_id == User.id,
                    ContentTranslation.created_at >= start_date
                )
            ).group_by(User.id, User.username, User.full_name).all()
            
            for user_stat in user_stats:
                user_reputation = await self._get_user_reputation(user_stat.id, db)
                user_level = await self._get_user_level(user_stat.id, db)
                
                leaderboard.append({
                    "user_id": user_stat.id,
                    "username": user_stat.username,
                    "full_name": user_stat.full_name,
                    "reputation": user_reputation,
                    "level": user_level,
                    "translation_count": user_stat.translation_count,
                    "period_contributions": user_stat.translation_count
                })
            
            # Sort by reputation
            leaderboard.sort(key=lambda x: x["reputation"], reverse=True)
            
            return {
                "leaderboard": leaderboard[:limit],
                "period": period,
                "total_contributors": len(leaderboard)
            }
            
        except Exception as e:
            logger.error(f"Error getting leaderboard: {e}")
            return {"leaderboard": [], "error": str(e)}
    
    async def _create_translation_suggestion(
        self,
        translation_id: int,
        suggested_title: Optional[str],
        suggested_content: Optional[str],
        contributor_id: int,
        notes: Optional[str],
        db: Session
    ) -> Dict[str, Any]:
        """Create a suggestion for improving existing translation."""
        try:
            translation = db.query(ContentTranslation).filter(
                ContentTranslation.id == translation_id
            ).first()
            
            if not translation:
                return {"success": False, "error": "Translation not found"}
            
            suggestion_data = {
                "contributor_id": contributor_id,
                "suggested_title": suggested_title,
                "suggested_content": suggested_content,
                "notes": notes,
                "status": ContributionStatus.PENDING.value,
                "submitted_at": datetime.utcnow().isoformat()
            }
            
            # Update translation metadata
            metadata = translation.metadata or {}
            suggestions = metadata.get("suggestions", [])
            suggestions.append(suggestion_data)
            metadata["suggestions"] = suggestions
            
            translation.metadata = metadata
            translation.updated_at = datetime.utcnow()
            
            db.commit()
            
            return {
                "success": True,
                "suggestion_id": len(suggestions),
                "status": ContributionStatus.PENDING.value,
                "message": "Translation suggestion submitted successfully"
            }
            
        except Exception as e:
            logger.error(f"Error creating translation suggestion: {e}")
            db.rollback()
            return {"success": False, "error": str(e)}
    
    async def _award_reputation(
        self,
        user_id: int,
        action: ReputationAction,
        db: Session
    ) -> None:
        """Award reputation points to user."""
        try:
            points = self.reputation_scores.get(action, 0)
            
            # Get or create user reputation record
            user = db.query(User).filter(User.id == user_id).first()
            if not user:
                return
            
            # Update user metadata with reputation
            metadata = getattr(user, 'metadata', None) or {}
            current_reputation = metadata.get("reputation", 0)
            metadata["reputation"] = current_reputation + points
            
            # Record reputation history
            reputation_history = metadata.get("reputation_history", [])
            reputation_history.append({
                "action": action.value,
                "points": points,
                "timestamp": datetime.utcnow().isoformat(),
                "total_reputation": metadata["reputation"]
            })
            metadata["reputation_history"] = reputation_history
            
            # Update user metadata (assuming User model has metadata field)
            if hasattr(user, 'metadata'):
                user.metadata = metadata
            
            db.commit()
            
        except Exception as e:
            logger.error(f"Error awarding reputation: {e}")
    
    async def _get_user_reputation(self, user_id: int, db: Session) -> int:
        """Get user's current reputation."""
        try:
            user = db.query(User).filter(User.id == user_id).first()
            if not user:
                return 0
            
            metadata = getattr(user, 'metadata', None) or {}
            return metadata.get("reputation", 0)
            
        except Exception as e:
            logger.error(f"Error getting user reputation: {e}")
            return 0
    
    async def _get_user_level(self, user_id: int, db: Session) -> str:
        """Get user's level based on reputation."""
        try:
            reputation = await self._get_user_reputation(user_id, db)
            
            for level, (min_rep, max_rep) in self.user_levels.items():
                if min_rep <= reputation <= max_rep:
                    return level
            
            return "Beginner"
            
        except Exception as e:
            logger.error(f"Error getting user level: {e}")
            return "Beginner"
    
    async def _get_user_stats(self, user_id: int, db: Session) -> Dict[str, Any]:
        """Get comprehensive user statistics."""
        try:
            # Get basic stats
            translation_count = db.query(ContentTranslation).filter(
                ContentTranslation.translator_id == user_id
            ).count()
            
            approved_count = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.translator_id == user_id,
                    ContentTranslation.translation_status == TranslationStatus.APPROVED
                )
            ).count()
            
            terminology_count = db.query(TerminologyGlossary).filter(
                TerminologyGlossary.created_by == user_id
            ).count()
            
            reputation = await self._get_user_reputation(user_id, db)
            level = await self._get_user_level(user_id, db)
            
            return {
                "reputation": reputation,
                "level": level,
                "translation_count": translation_count,
                "approved_translations": approved_count,
                "terminology_contributions": terminology_count,
                "approval_rate": (approved_count / max(translation_count, 1)) * 100,
                "privileges": self.level_privileges.get(level, [])
            }
            
        except Exception as e:
            logger.error(f"Error getting user stats: {e}")
            return {}


# Global community translation system instance
community_translation = CommunityTranslationSystem()