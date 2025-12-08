"""Authentication API endpoints"""

from typing import Dict, Any
from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session

from app.core.database import get_db
from app.models.user import (
    UserCreate, UserLogin, UserResponse, AuthResponse, 
    Token, PasswordChange, UserPreferences, UserProfile
)
from app.services.auth_service import AuthService
from app.core.security import get_user_id_from_token
from app.core.rate_limiter import rate_limiter, get_client_ip
import logging

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/auth", tags=["authentication"])
security = HTTPBearer()


def get_current_user_id(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> int:
    """Get current user ID from JWT token"""
    try:
        token = credentials.credentials
        user_id = get_user_id_from_token(token)
        
        # Verify user exists and is active
        auth_service = AuthService(db)
        user = auth_service.user_service.get(user_id)
        
        if not user or not user.is_active:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found or inactive",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return user_id
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting current user: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )


@router.post("/register", response_model=AuthResponse)
async def register(
    user_data: UserCreate,
    request: Request,
    db: Session = Depends(get_db)
):
    """Register a new user"""
    try:
        # Rate limiting for registration
        client_ip = get_client_ip(request)
        if not rate_limiter.is_allowed(client_ip, max_requests=5, window_minutes=60, action="register"):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Too many registration attempts. Please try again later."
            )
        
        auth_service = AuthService(db)
        result = auth_service.register_user(user_data)
        
        return AuthResponse(
            user=UserResponse.from_orm(result["user"]),
            access_token=result["access_token"],
            refresh_token=result["refresh_token"],
            token_type=result["token_type"]
        )
        
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Registration error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Registration failed"
        )


@router.post("/login", response_model=AuthResponse)
async def login(
    login_data: UserLogin,
    request: Request,
    db: Session = Depends(get_db)
):
    """Login user"""
    try:
        # Rate limiting for login
        client_ip = get_client_ip(request)
        if not rate_limiter.is_allowed(client_ip, max_requests=10, window_minutes=15, action="login"):
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Too many login attempts. Please try again later."
            )
        
        # Check if account is locked
        if rate_limiter.is_account_locked(login_data.email):
            remaining = rate_limiter.get_lockout_time_remaining(login_data.email)
            raise HTTPException(
                status_code=status.HTTP_423_LOCKED,
                detail=f"Account temporarily locked. Try again in {remaining // 60} minutes."
            )
        
        auth_service = AuthService(db)
        result = auth_service.login_user(login_data)
        
        # Clear failed attempts on successful login
        rate_limiter.clear_failed_attempts(login_data.email)
        
        return AuthResponse(
            user=UserResponse.from_orm(result["user"]),
            access_token=result["access_token"],
            refresh_token=result["refresh_token"],
            token_type=result["token_type"]
        )
        
    except ValueError as e:
        # Record failed login attempt
        rate_limiter.record_failed_login(login_data.email)
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Login error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Login failed"
        )


@router.post("/refresh", response_model=Token)
async def refresh_token(
    refresh_token: str,
    db: Session = Depends(get_db)
):
    """Refresh access token"""
    try:
        auth_service = AuthService(db)
        result = auth_service.refresh_access_token(refresh_token)
        
        return Token(
            access_token=result["access_token"],
            refresh_token=result["refresh_token"],
            token_type=result["token_type"]
        )
        
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Token refresh error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Token refresh failed"
        )


@router.post("/logout")
async def logout(
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Logout user"""
    try:
        auth_service = AuthService(db)
        success = auth_service.logout_user(current_user_id)
        
        if success:
            return {"message": "Successfully logged out"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Logout failed"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Logout error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Logout failed"
        )


@router.get("/me", response_model=UserResponse)
async def get_current_user(
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Get current user information"""
    try:
        auth_service = AuthService(db)
        user = auth_service.user_service.get(current_user_id)
        
        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
        
        return UserResponse.from_orm(user)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get current user error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get user information"
        )


@router.get("/profile", response_model=UserProfile)
async def get_user_profile(
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Get detailed user profile"""
    try:
        auth_service = AuthService(db)
        profile = auth_service.get_user_profile(current_user_id)
        
        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )
        
        return UserProfile(
            user=UserResponse.from_orm(profile["user"]),
            account_age_days=profile["account_age_days"],
            last_login=profile["last_login"],
            preferences=profile["preferences"]
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get user profile error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get user profile"
        )


@router.put("/change-password")
async def change_password(
    password_data: PasswordChange,
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Change user password"""
    try:
        auth_service = AuthService(db)
        success = auth_service.change_password(
            current_user_id,
            password_data.current_password,
            password_data.new_password
        )
        
        if success:
            return {"message": "Password changed successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Password change failed"
            )
            
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Change password error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Password change failed"
        )


@router.put("/preferences")
async def update_preferences(
    preferences: UserPreferences,
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Update user preferences"""
    try:
        auth_service = AuthService(db)
        user = auth_service.update_user_preferences(
            current_user_id,
            preferences.dict(exclude_unset=True)
        )
        
        if user:
            return {"message": "Preferences updated successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Update preferences error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update preferences"
        )


@router.delete("/deactivate")
async def deactivate_account(
    current_user_id: int = Depends(get_current_user_id),
    db: Session = Depends(get_db)
):
    """Deactivate user account"""
    try:
        auth_service = AuthService(db)
        success = auth_service.deactivate_user(current_user_id)
        
        if success:
            return {"message": "Account deactivated successfully"}
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Account deactivation failed"
            )
            
    except Exception as e:
        logger.error(f"Deactivate account error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Account deactivation failed"
        )


@router.post("/validate-token")
async def validate_token(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
):
    """Validate JWT token"""
    try:
        token = credentials.credentials
        auth_service = AuthService(db)
        is_valid = auth_service.validate_token(token)
        
        return {"valid": is_valid}
        
    except Exception as e:
        logger.error(f"Token validation error: {e}")
        return {"valid": False}