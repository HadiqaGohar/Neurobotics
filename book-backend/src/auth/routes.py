"""
Authentication routes for Better-Auth integration.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from fastapi.security import OAuth2PasswordRequestForm
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr, validator
from typing import Optional
import logging

from book_backend.src.database.database import get_db
from book_backend.src.auth.better_auth import (
    AuthService, 
    SessionManager, 
    get_current_user, 
    get_optional_user,
    OAuthService
)
from book_backend.src.database.models import User
from book_backend.src.security.security_utils import SecurityValidator, rate_limiter

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/auth", tags=["authentication"])


# Pydantic models
class UserSignup(BaseModel):
    email: EmailStr
    password: str
    full_name: Optional[str] = None
    
    @validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.islower() for c in v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one digit')
        return v
    
    @validator('email')
    def validate_email(cls, v):
        # Additional email validation
        if not SecurityValidator.sanitize_input(v):
            raise ValueError('Invalid email format')
        return v.lower()


class UserLogin(BaseModel):
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str
    user: dict


class RefreshTokenRequest(BaseModel):
    refresh_token: str


class UserProfile(BaseModel):
    id: str
    email: str
    full_name: Optional[str]
    is_active: bool
    created_at: str
    last_login: Optional[str]


class UpdateProfile(BaseModel):
    full_name: Optional[str] = None
    current_password: Optional[str] = None
    new_password: Optional[str] = None
    
    @validator('new_password')
    def validate_new_password(cls, v, values):
        if v and not values.get('current_password'):
            raise ValueError('Current password required to change password')
        if v and len(v) < 8:
            raise ValueError('New password must be at least 8 characters long')
        return v


# Authentication routes
@router.post("/signup", response_model=TokenResponse)
async def signup(
    user_data: UserSignup,
    request: Request,
    db: Session = Depends(get_db)
):
    """User registration endpoint."""
    try:
        # Rate limiting
        client_ip = request.client.host
        if not rate_limiter.is_allowed(client_ip, max_requests=5, window_minutes=60):
            raise HTTPException(status_code=429, detail="Too many signup attempts")
        
        # Create user
        user = AuthService.create_user(
            db=db,
            email=user_data.email,
            password=user_data.password,
            full_name=user_data.full_name
        )
        
        # Create session
        tokens = SessionManager.create_session(user)
        
        # Prepare user data for response
        user_dict = {
            "id": str(user.id),
            "email": user.email,
            "full_name": user.full_name,
            "is_active": user.is_active,
            "created_at": user.created_at.isoformat()
        }
        
        logger.info(f"New user registered: {user.email}")
        
        return TokenResponse(
            access_token=tokens["access_token"],
            refresh_token=tokens["refresh_token"],
            token_type=tokens["token_type"],
            user=user_dict
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(status_code=500, detail="Registration failed")


@router.post("/login", response_model=TokenResponse)
async def login(
    user_data: UserLogin,
    request: Request,
    db: Session = Depends(get_db)
):
    """User login endpoint."""
    try:
        # Rate limiting
        client_ip = request.client.host
        if not rate_limiter.is_allowed(client_ip, max_requests=10, window_minutes=60):
            raise HTTPException(status_code=429, detail="Too many login attempts")
        
        # Authenticate user
        user = AuthService.authenticate_user(
            db=db,
            email=user_data.email,
            password=user_data.password
        )
        
        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")
        
        # Create session
        tokens = SessionManager.create_session(user)
        
        # Prepare user data for response
        user_dict = {
            "id": str(user.id),
            "email": user.email,
            "full_name": user.full_name,
            "is_active": user.is_active,
            "created_at": user.created_at.isoformat(),
            "last_login": user.last_login.isoformat() if user.last_login else None
        }
        
        logger.info(f"User logged in: {user.email}")
        
        return TokenResponse(
            access_token=tokens["access_token"],
            refresh_token=tokens["refresh_token"],
            token_type=tokens["token_type"],
            user=user_dict
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {e}")
        raise HTTPException(status_code=500, detail="Login failed")


@router.post("/refresh", response_model=TokenResponse)
async def refresh_token(
    token_data: RefreshTokenRequest,
    db: Session = Depends(get_db)
):
    """Refresh access token using refresh token."""
    try:
        # Refresh session
        tokens = SessionManager.refresh_session(token_data.refresh_token, db)
        
        # Get user info for response
        payload = AuthService.verify_token(tokens["access_token"], "access")
        user_id = payload.get("sub")
        
        from book_backend.src.database import crud
        user = crud.get_user(db, user_id)
        
        user_dict = {
            "id": str(user.id),
            "email": user.email,
            "full_name": user.full_name,
            "is_active": user.is_active,
            "created_at": user.created_at.isoformat(),
            "last_login": user.last_login.isoformat() if user.last_login else None
        }
        
        return TokenResponse(
            access_token=tokens["access_token"],
            refresh_token=tokens["refresh_token"],
            token_type=tokens["token_type"],
            user=user_dict
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Token refresh error: {e}")
        raise HTTPException(status_code=401, detail="Could not refresh token")


@router.post("/logout")
async def logout(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """User logout endpoint."""
    try:
        # Revoke session
        success = SessionManager.revoke_session(str(current_user.id), db)
        
        if success:
            logger.info(f"User logged out: {current_user.email}")
            return {"message": "Successfully logged out"}
        else:
            raise HTTPException(status_code=500, detail="Logout failed")
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Logout error: {e}")
        raise HTTPException(status_code=500, detail="Logout failed")


@router.get("/me", response_model=UserProfile)
async def get_current_user_profile(
    current_user: User = Depends(get_current_user)
):
    """Get current user profile."""
    return UserProfile(
        id=str(current_user.id),
        email=current_user.email,
        full_name=current_user.full_name,
        is_active=current_user.is_active,
        created_at=current_user.created_at.isoformat(),
        last_login=current_user.last_login.isoformat() if current_user.last_login else None
    )


@router.put("/me", response_model=UserProfile)
async def update_user_profile(
    profile_data: UpdateProfile,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Update current user profile."""
    try:
        # Update full name if provided
        if profile_data.full_name is not None:
            current_user.full_name = profile_data.full_name
        
        # Update password if provided
        if profile_data.new_password:
            # Verify current password
            if not AuthService.verify_password(profile_data.current_password, current_user.hashed_password):
                raise HTTPException(status_code=400, detail="Current password is incorrect")
            
            # Hash new password
            current_user.hashed_password = AuthService.hash_password(profile_data.new_password)
        
        # Save changes
        db.commit()
        db.refresh(current_user)
        
        logger.info(f"User profile updated: {current_user.email}")
        
        return UserProfile(
            id=str(current_user.id),
            email=current_user.email,
            full_name=current_user.full_name,
            is_active=current_user.is_active,
            created_at=current_user.created_at.isoformat(),
            last_login=current_user.last_login.isoformat() if current_user.last_login else None
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Profile update error: {e}")
        raise HTTPException(status_code=500, detail="Profile update failed")


# OAuth routes (for future implementation)
@router.get("/oauth/{provider}")
async def oauth_login(provider: str, request: Request):
    """Initiate OAuth login with provider."""
    try:
        # Generate state for CSRF protection
        state = SecurityValidator.generate_session_token()
        
        # Get OAuth URL
        oauth_url = OAuthService.get_oauth_url(provider, state)
        
        # Store state in session (in production, use Redis or database)
        # For now, return the URL
        return {"oauth_url": oauth_url, "state": state}
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"OAuth initiation error: {e}")
        raise HTTPException(status_code=500, detail="OAuth login failed")


@router.get("/oauth/{provider}/callback")
async def oauth_callback(
    provider: str,
    code: str,
    state: str,
    db: Session = Depends(get_db)
):
    """Handle OAuth callback."""
    try:
        # Verify state (CSRF protection)
        # In production, verify against stored state
        
        # Handle OAuth callback
        result = await OAuthService.handle_oauth_callback(provider, code, db)
        
        return result
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"OAuth callback error: {e}")
        raise HTTPException(status_code=500, detail="OAuth callback failed")


# Password reset routes (for future implementation)
@router.post("/forgot-password")
async def forgot_password(email: EmailStr, db: Session = Depends(get_db)):
    """Initiate password reset process."""
    # Implementation would:
    # 1. Check if user exists
    # 2. Generate reset token
    # 3. Send reset email
    # 4. Store reset token with expiration
    
    return {"message": "If the email exists, a password reset link has been sent"}


@router.post("/reset-password")
async def reset_password(
    token: str,
    new_password: str,
    db: Session = Depends(get_db)
):
    """Reset password using reset token."""
    # Implementation would:
    # 1. Verify reset token
    # 2. Check token expiration
    # 3. Update user password
    # 4. Invalidate reset token
    
    raise HTTPException(status_code=501, detail="Password reset not implemented yet")


# Admin routes (for future implementation)
@router.get("/admin/users")
async def list_users(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """List all users (admin only)."""
    # Check if user is admin
    if not hasattr(current_user, 'role') or current_user.role != 'admin':
        raise HTTPException(status_code=403, detail="Admin access required")
    
    # Implementation would return paginated user list
    raise HTTPException(status_code=501, detail="Admin features not implemented yet")