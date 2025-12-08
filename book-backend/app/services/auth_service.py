"""Authentication service for user registration, login, and token management"""

from typing import Optional, Dict, Any
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
import logging

from app.models.database import User
from app.models.user import UserCreate, UserLogin, Token
from app.core.security import (
    get_password_hash, 
    verify_password, 
    create_access_token, 
    create_refresh_token,
    verify_token
)
from app.services.user_service import UserService
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class AuthService:
    """Service for authentication operations"""
    
    def __init__(self, db: Session):
        self.db = db
        self.user_service = UserService(db)
    
    def register_user(self, user_data: UserCreate) -> Dict[str, Any]:
        """
        Register a new user
        
        Returns:
            Dict with user info and tokens
        """
        try:
            # Check if user already exists
            existing_user = self.user_service.get_by_email(user_data.email)
            if existing_user:
                raise ValueError("User with this email already exists")
            
            # Create new user
            user = self.user_service.create(user_data)
            
            # Generate tokens
            access_token = create_access_token(subject=str(user.id))
            refresh_token = create_refresh_token(subject=str(user.id))
            
            # Update last login
            self.user_service.update_last_login(user.id)
            
            logger.info(f"User registered successfully: {user.email}")
            
            return {
                "user": user,
                "access_token": access_token,
                "refresh_token": refresh_token,
                "token_type": "bearer"
            }
            
        except ValueError:
            raise
        except Exception as e:
            logger.error(f"Error registering user: {e}")
            raise Exception("Registration failed")
    
    def login_user(self, login_data: UserLogin) -> Dict[str, Any]:
        """
        Authenticate user and return tokens
        
        Returns:
            Dict with user info and tokens
        """
        try:
            # Authenticate user
            user = self.user_service.authenticate(login_data.email, login_data.password)
            
            if not user:
                raise ValueError("Invalid email or password")
            
            if not user.is_active:
                raise ValueError("Account is deactivated")
            
            # Generate tokens
            access_token = create_access_token(subject=str(user.id))
            refresh_token = create_refresh_token(subject=str(user.id))
            
            # Update last login
            self.user_service.update_last_login(user.id)
            
            logger.info(f"User logged in successfully: {user.email}")
            
            return {
                "user": user,
                "access_token": access_token,
                "refresh_token": refresh_token,
                "token_type": "bearer"
            }
            
        except ValueError:
            raise
        except Exception as e:
            logger.error(f"Error logging in user: {e}")
            raise Exception("Login failed")
    
    def refresh_access_token(self, refresh_token: str) -> Dict[str, Any]:
        """
        Refresh access token using refresh token
        
        Returns:
            Dict with new tokens
        """
        try:
            # Verify refresh token
            user_id = verify_token(refresh_token, token_type="refresh")
            if not user_id:
                raise ValueError("Invalid refresh token")
            
            # Get user
            user = self.user_service.get(int(user_id))
            if not user or not user.is_active:
                raise ValueError("User not found or inactive")
            
            # Generate new tokens
            new_access_token = create_access_token(subject=str(user.id))
            new_refresh_token = create_refresh_token(subject=str(user.id))
            
            logger.info(f"Tokens refreshed for user: {user.email}")
            
            return {
                "access_token": new_access_token,
                "refresh_token": new_refresh_token,
                "token_type": "bearer"
            }
            
        except ValueError:
            raise
        except Exception as e:
            logger.error(f"Error refreshing token: {e}")
            raise Exception("Token refresh failed")
    
    def get_current_user_from_token(self, token: str) -> Optional[User]:
        """
        Get current user from access token
        
        Returns:
            User object or None
        """
        try:
            # Verify token
            user_id = verify_token(token, token_type="access")
            if not user_id:
                return None
            
            # Get user
            user = self.user_service.get(int(user_id))
            if not user or not user.is_active:
                return None
            
            return user
            
        except Exception as e:
            logger.error(f"Error getting user from token: {e}")
            return None
    
    def change_password(
        self, 
        user_id: int, 
        current_password: str, 
        new_password: str
    ) -> bool:
        """
        Change user password
        
        Returns:
            True if successful, False otherwise
        """
        try:
            user = self.user_service.get(user_id)
            if not user:
                raise ValueError("User not found")
            
            # Verify current password
            if not verify_password(current_password, user.hashed_password):
                raise ValueError("Current password is incorrect")
            
            # Update password
            user.hashed_password = get_password_hash(new_password)
            user.updated_at = datetime.utcnow()
            
            self.db.commit()
            
            logger.info(f"Password changed for user: {user.email}")
            return True
            
        except ValueError:
            raise
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error changing password for user {user_id}: {e}")
            raise Exception("Password change failed")
    
    def deactivate_user(self, user_id: int) -> bool:
        """
        Deactivate user account
        
        Returns:
            True if successful, False otherwise
        """
        try:
            return self.user_service.delete(user_id)
            
        except Exception as e:
            logger.error(f"Error deactivating user {user_id}: {e}")
            return False
    
    def update_user_preferences(
        self, 
        user_id: int, 
        preferences: Dict[str, Any]
    ) -> Optional[User]:
        """
        Update user preferences
        
        Returns:
            Updated user object or None
        """
        try:
            return self.user_service.update_preferences(user_id, preferences)
            
        except Exception as e:
            logger.error(f"Error updating preferences for user {user_id}: {e}")
            raise Exception("Failed to update preferences")
    
    def validate_token(self, token: str, token_type: str = "access") -> bool:
        """
        Validate token
        
        Returns:
            True if valid, False otherwise
        """
        try:
            user_id = verify_token(token, token_type)
            if not user_id:
                return False
            
            # Check if user exists and is active
            user = self.user_service.get(int(user_id))
            return user is not None and user.is_active
            
        except Exception as e:
            logger.error(f"Error validating token: {e}")
            return False
    
    def get_user_profile(self, user_id: int) -> Optional[Dict[str, Any]]:
        """
        Get user profile with additional info
        
        Returns:
            Dict with user profile data
        """
        try:
            user = self.user_service.get(user_id)
            if not user:
                return None
            
            # Get additional stats (could be extended)
            profile = {
                "user": user,
                "account_age_days": (datetime.utcnow() - user.created_at).days,
                "last_login": user.last_login,
                "preferences": user.preferences or {}
            }
            
            return profile
            
        except Exception as e:
            logger.error(f"Error getting user profile {user_id}: {e}")
            return None
    
    def logout_user(self, user_id: int) -> bool:
        """
        Logout user (placeholder for token blacklisting)
        
        In a production system, you would add the token to a blacklist
        For now, this is just a placeholder that returns True
        
        Returns:
            True if successful
        """
        try:
            # In a real implementation, you would:
            # 1. Add the current token to a blacklist/cache
            # 2. Optionally update user's last_logout timestamp
            
            logger.info(f"User {user_id} logged out")
            return True
            
        except Exception as e:
            logger.error(f"Error logging out user {user_id}: {e}")
            return False