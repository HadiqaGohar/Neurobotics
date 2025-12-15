
"""User Pydantic models for request/response validation/field_validator"""

from typing import Optional, Dict, Any
from datetime import datetime
from pydantic import BaseModel, EmailStr, validator
from app.core.security import validate_password_strength, sanitize_input, validate_email_format


class UserBase(BaseModel):
    """Base user model"""
    email: EmailStr
    full_name: Optional[str] = None
    persona: str = "general"
    experience_level: str = "intermediate"
    preferred_language: str = "en"
    is_active: bool = True


class UserCreate(UserBase):
    """User creation model"""
    password: str
    confirm_password: str
    
    @validator('email')
    def validate_email_security(cls, v):
        if not validate_email_format(v):
            raise ValueError('Invalid email format')
        return sanitize_input(v, 254)
    
    @validator('full_name')
    def validate_full_name(cls, v):
        if v:
            sanitized = sanitize_input(v, 100)
            if len(sanitized.strip()) < 2:
                raise ValueError('Name must be at least 2 characters long')
            return sanitized
        return v
    
    @validator('password')
    def validate_password(cls, v):
        if not validate_password_strength(v):
            raise ValueError(
                'Password must be at least 8 characters long and contain '
                'uppercase, lowercase, digit, and special character'
            )
        return v
    
    @validator('confirm_password')
    def passwords_match(cls, v, values, **kwargs):
        if 'password' in values and v != values['password']:
            raise ValueError('Passwords do not match')
        return v


class UserUpdate(BaseModel):
    """User update model"""
    email: Optional[EmailStr] = None
    full_name: Optional[str] = None
    persona: Optional[str] = None
    experience_level: Optional[str] = None
    preferred_language: Optional[str] = None
    preferences: Optional[Dict[str, Any]] = None
    is_active: Optional[bool] = None


class UserLogin(BaseModel):
    """User login model"""
    email: EmailStr
    password: str


class UserResponse(UserBase):
    """User response model"""
    id: int
    created_at: datetime
    updated_at: datetime
    last_login: Optional[datetime] = None
    preferences: Optional[Dict[str, Any]] = None
    
    class Config:
        from_attributes = True


class Token(BaseModel):
    """Token response model"""
    access_token: str
    refresh_token: str
    token_type: str = "bearer"


class TokenData(BaseModel):
    """Token data model"""
    user_id: Optional[int] = None


class AuthResponse(BaseModel):
    """Authentication response model"""
    user: UserResponse
    access_token: str
    refresh_token: str
    token_type: str = "bearer"


class PasswordChange(BaseModel):
    """Password change model"""
    current_password: str
    new_password: str
    confirm_new_password: str
    
    @validator('new_password')
    def validate_new_password(cls, v):
        if not validate_password_strength(v):
            raise ValueError(
                'Password must be at least 8 characters long and contain '
                'uppercase, lowercase, digit, and special character'
            )
        return v
    
    @validator('confirm_new_password')
    def passwords_match(cls, v, values, **kwargs):
        if 'new_password' in values and v != values['new_password']:
            raise ValueError('Passwords do not match')
        return v


class UserPreferences(BaseModel):
    """User preferences model"""
    theme: Optional[str] = "light"
    language: Optional[str] = "en"
    notifications: Optional[Dict[str, bool]] = None
    chat_settings: Optional[Dict[str, Any]] = None
    display_settings: Optional[Dict[str, Any]] = None


class UserProfile(BaseModel):
    """Extended user profile model"""
    user: UserResponse
    account_age_days: int
    last_login: Optional[datetime] = None
    preferences: Dict[str, Any]
    
    class Config:
        from_attributes = True


class RefreshToken(BaseModel):
    """Refresh token model"""
    refresh_token: str


class UserPasswordUpdate(BaseModel):
    """User password update model"""
    current_password: str
    new_password: str
    
    @validator('new_password')
    def validate_new_password(cls, v):
        if not validate_password_strength(v):
            raise ValueError(
                'Password must be at least 8 characters long and contain '
                'uppercase, lowercase, digit, and special character'
            )
        return v


class User(UserResponse):
    """User model for responses (alias for UserResponse)"""
    pass


class UserProfileModel(BaseModel):
    """User profile model for profile endpoints"""
    id: int
    email: EmailStr
    full_name: Optional[str] = None
    is_active: bool
    created_at: datetime
    last_login: Optional[datetime] = None
    
    class Config:
        from_attributes = True


class UserProfileUpdate(BaseModel):
    """User profile update model"""
    full_name: Optional[str] = None
    current_password: Optional[str] = None
    new_password: Optional[str] = None
    
    @validator('full_name')
    def validate_full_name(cls, v):
        if v is not None:
            sanitized = sanitize_input(v, 100)
            if len(sanitized.strip()) < 2:
                raise ValueError('Name must be at least 2 characters long')
            return sanitized
        return v
    
    @validator('new_password')
    def validate_new_password(cls, v):
        if v is not None:
            if not validate_password_strength(v):
                raise ValueError(
                    'Password must be at least 8 characters long and contain '
                    'uppercase, lowercase, digit, and special character'
                )
        return v