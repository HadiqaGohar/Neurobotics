"""User service for database operations"""

from typing import Optional, List, Dict, Any
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from sqlalchemy import func

from app.models.database import User
from app.models.user import UserCreate, UserUpdate
from app.core.security import get_password_hash, verify_password
import logging

logger = logging.getLogger(__name__)


class UserService:
    """Service for user operations"""
    
    def __init__(self, db: Session):
        self.db = db
    
    def get(self, user_id: int) -> Optional[User]:
        """Get user by ID"""
        return self.db.query(User).filter(User.id == user_id).first()
    
    def get_by_email(self, email: str) -> Optional[User]:
        """Get user by email"""
        return self.db.query(User).filter(User.email == email).first()
    
    def get_multi(
        self, 
        skip: int = 0, 
        limit: int = 100,
        is_active: Optional[bool] = None
    ) -> List[User]:
        """Get multiple users"""
        query = self.db.query(User)
        
        if is_active is not None:
            query = query.filter(User.is_active == is_active)
        
        return query.offset(skip).limit(limit).all()
    
    def create(self, user_in: UserCreate) -> User:
        """Create new user"""
        try:
            hashed_password = get_password_hash(user_in.password)
            
            db_user = User(
                email=user_in.email,
                hashed_password=hashed_password,
                full_name=user_in.full_name,
                is_active=user_in.is_active,
                persona=user_in.persona,
                experience_level=user_in.experience_level,
                preferred_language=user_in.preferred_language,
                created_at=datetime.utcnow()
            )
            
            self.db.add(db_user)
            self.db.commit()
            self.db.refresh(db_user)
            
            logger.info(f"Created user: {db_user.email} (ID: {db_user.id})")
            return db_user
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error creating user: {e}")
            raise
    
    def update(self, user_id: int, user_in: UserUpdate) -> Optional[User]:
        """Update user"""
        try:
            db_user = self.get(user_id)
            if not db_user:
                return None
            
            update_data = user_in.dict(exclude_unset=True)
            
            for field, value in update_data.items():
                setattr(db_user, field, value)
            
            db_user.updated_at = datetime.utcnow()
            
            self.db.commit()
            self.db.refresh(db_user)
            
            logger.info(f"Updated user: {db_user.email} (ID: {db_user.id})")
            return db_user
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating user {user_id}: {e}")
            raise
    
    def delete(self, user_id: int) -> bool:
        """Delete user (soft delete by setting is_active=False)"""
        try:
            db_user = self.get(user_id)
            if not db_user:
                return False
            
            db_user.is_active = False
            db_user.updated_at = datetime.utcnow()
            
            self.db.commit()
            logger.info(f"Deactivated user: {db_user.email} (ID: {user_id})")
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error deactivating user {user_id}: {e}")
            return False
    
    def authenticate(self, email: str, password: str) -> Optional[User]:
        """Authenticate user"""
        user = self.get_by_email(email)
        if not user:
            return None
        
        if not verify_password(password, user.hashed_password):
            return None
        
        return user
    
    def update_last_login(self, user_id: int) -> bool:
        """Update user's last login timestamp"""
        try:
            db_user = self.get(user_id)
            if not db_user:
                return False
            
            db_user.last_login = datetime.utcnow()
            self.db.commit()
            
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating last login for user {user_id}: {e}")
            return False
    
    def update_preferences(self, user_id: int, preferences: Dict[str, Any]) -> Optional[User]:
        """Update user preferences"""
        try:
            db_user = self.get(user_id)
            if not db_user:
                return None
            
            db_user.preferences = preferences
            db_user.updated_at = datetime.utcnow()
            
            self.db.commit()
            self.db.refresh(db_user)
            
            logger.info(f"Updated preferences for user: {db_user.email}")
            return db_user
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating preferences for user {user_id}: {e}")
            raise
    
    def get_stats(self) -> Dict[str, Any]:
        """Get user statistics"""
        try:
            total_users = self.db.query(User).count()
            active_users = self.db.query(User).filter(User.is_active == True).count()
            
            # Users created today
            today = datetime.utcnow().date()
            new_users_today = self.db.query(User).filter(
                func.date(User.created_at) == today
            ).count()
            
            # Users created this week
            week_ago = datetime.utcnow() - timedelta(days=7)
            new_users_this_week = self.db.query(User).filter(
                User.created_at >= week_ago
            ).count()
            
            # Persona distribution
            persona_stats = self.db.query(
                User.persona, func.count(User.id)
            ).group_by(User.persona).all()
            
            # Experience level distribution
            experience_stats = self.db.query(
                User.experience_level, func.count(User.id)
            ).group_by(User.experience_level).all()
            
            # Language distribution
            language_stats = self.db.query(
                User.preferred_language, func.count(User.id)
            ).group_by(User.preferred_language).all()
            
            return {
                "total_users": total_users,
                "active_users": active_users,
                "new_users_today": new_users_today,
                "new_users_this_week": new_users_this_week,
                "personas": dict(persona_stats),
                "experience_levels": dict(experience_stats),
                "languages": dict(language_stats)
            }
            
        except Exception as e:
            logger.error(f"Error getting user stats: {e}")
            return {}
    
    def search(self, query: str, limit: int = 20) -> List[User]:
        """Search users by name or email"""
        try:
            return self.db.query(User).filter(
                (User.full_name.ilike(f"%{query}%")) |
                (User.email.ilike(f"%{query}%"))
            ).limit(limit).all()
            
        except Exception as e:
            logger.error(f"Error searching users: {e}")
            return []