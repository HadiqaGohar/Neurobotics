"""
User profile management endpoints
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from app.core.database import get_db
from app.core.security import get_current_user
from app.models.database import User
from app.models.user import UserProfileModel, UserProfileUpdate, UserResponse
from app.core.security import verify_password, get_password_hash
from datetime import datetime
import logging

logger = logging.getLogger(__name__)
router = APIRouter()


@router.get("/profile", response_model=UserProfileModel)
async def get_user_profile(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get current user's profile information
    """
    try:
        return UserProfileModel(
            id=current_user.id,
            email=current_user.email,
            full_name=current_user.full_name,
            is_active=current_user.is_active,
            created_at=current_user.created_at,
            last_login=current_user.last_login
        )
    except Exception as e:
        logger.error(f"Error getting user profile: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get user profile"
        )


@router.put("/profile", response_model=UserResponse)
async def update_user_profile(
    profile_update: UserProfileUpdate,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update current user's profile information
    """
    try:
        # Validate current password if changing password
        if profile_update.new_password:
            if not profile_update.current_password:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Current password is required to change password"
                )
            
            if not verify_password(profile_update.current_password, current_user.password_hash):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Current password is incorrect"
                )
            
            # Update password
            current_user.password_hash = get_password_hash(profile_update.new_password)
            logger.info(f"Password updated for user {current_user.email}")

        # Update full name if provided
        if profile_update.full_name is not None:
            current_user.full_name = profile_update.full_name.strip()
            logger.info(f"Full name updated for user {current_user.email}")

        # Update last modified timestamp
        current_user.updated_at = datetime.utcnow()
        
        # Commit changes
        db.commit()
        db.refresh(current_user)
        
        return UserResponse(
            id=current_user.id,
            email=current_user.email,
            full_name=current_user.full_name,
            is_active=current_user.is_active,
            created_at=current_user.created_at,
            last_login=current_user.last_login
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating user profile: {e}")
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update user profile"
        )


@router.get("/me", response_model=UserResponse)
async def get_current_user_info(
    current_user: User = Depends(get_current_user)
):
    """
    Get current user information (alias for profile)
    """
    return UserResponse(
        id=current_user.id,
        email=current_user.email,
        full_name=current_user.full_name,
        is_active=current_user.is_active,
        created_at=current_user.created_at,
        last_login=current_user.last_login
    )


@router.delete("/account")
async def delete_user_account(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Deactivate user account (soft delete)
    """
    try:
        # Soft delete - just deactivate the account
        current_user.is_active = False
        current_user.updated_at = datetime.utcnow()
        
        db.commit()
        
        logger.info(f"Account deactivated for user {current_user.email}")
        
        return {"message": "Account deactivated successfully"}
        
    except Exception as e:
        logger.error(f"Error deactivating account: {e}")
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to deactivate account"
        )


@router.post("/reactivate")
async def reactivate_user_account(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Reactivate user account
    """
    try:
        current_user.is_active = True
        current_user.updated_at = datetime.utcnow()
        
        db.commit()
        
        logger.info(f"Account reactivated for user {current_user.email}")
        
        return {"message": "Account reactivated successfully"}
        
    except Exception as e:
        logger.error(f"Error reactivating account: {e}")
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to reactivate account"
        )