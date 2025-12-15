"""Security utilities for authentication and authorization"""

import os
import jwt
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from passlib.context import CryptContext
from fastapi import HTTPException, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from sqlalchemy.orm import Session
from app.core.config import get_settings

settings = get_settings()

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT settings
ALGORITHM = settings.jwt_algorithm
SECRET_KEY = settings.jwt_secret_key
ACCESS_TOKEN_EXPIRE_MINUTES = settings.access_token_expire_minutes


def get_password_hash(password: str) -> str:
    """Hash a password using bcrypt"""
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash"""
    return pwd_context.verify(plain_password, hashed_password)


def create_access_token(
    subject: str, 
    expires_delta: Optional[timedelta] = None,
    additional_claims: Optional[Dict[str, Any]] = None
) -> str:
    """Create a JWT access token"""
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    
    to_encode = {
        "exp": expire,
        "iat": datetime.utcnow(),
        "sub": subject,
        "type": "access"
    }
    
    if additional_claims:
        to_encode.update(additional_claims)
    
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def create_refresh_token(
    subject: str,
    expires_delta: Optional[timedelta] = None
) -> str:
    """Create a JWT refresh token"""
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=7)  # Refresh tokens last 7 days
    
    to_encode = {
        "exp": expire,
        "iat": datetime.utcnow(),
        "sub": subject,
        "type": "refresh"
    }
    
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str, token_type: str = "access") -> Dict[str, Any]:
    """
    Verify and decode a JWT token
    
    Args:
        token: JWT token string
        token_type: Expected token type ("access" or "refresh")
    
    Returns:
        Decoded token payload
    
    Raises:
        HTTPException: If token is invalid or expired
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        
        # Check token type
        if payload.get("type") != token_type:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token type",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        # Check expiration
        exp = payload.get("exp")
        if exp is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token missing expiration",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        if datetime.utcnow() > datetime.fromtimestamp(exp):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token expired",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return payload
        
    except jwt.InvalidTokenError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )


def get_user_id_from_token(token: str) -> int:
    """Extract user ID from JWT token"""
    payload = verify_token(token)
    user_id = payload.get("sub")
    
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token missing user ID",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    try:
        return int(user_id)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
            headers={"WWW-Authenticate": "Bearer"},
        )


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = None,
    db: Session = None
):
    """
    Get current user from JWT token
    This is a FastAPI dependency function
    """
    from fastapi import Depends, HTTPException, status
    from fastapi.security import HTTPBearer
    from sqlalchemy.orm import Session
    from app.core.database import get_db
    from app.models.database import User
    
    if credentials is None or db is None:
        # Return the dependency function
        security = HTTPBearer()
        
        async def _get_current_user_dependency(
            creds: HTTPAuthorizationCredentials = Depends(security),
            database: Session = Depends(get_db)
        ) -> User:
            return await get_current_user(creds, database)
        
        return _get_current_user_dependency
    
    try:
        # Verify token and get user ID
        user_id = get_user_id_from_token(credentials.credentials)
        
        # Get user from database
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        if not user.is_active:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User account is disabled",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return user
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )


def validate_password_strength(password: str) -> bool:
    """
    Validate password strength
    
    Requirements:
    - At least 8 characters long
    - Contains at least one uppercase letter
    - Contains at least one lowercase letter
    - Contains at least one digit
    - Contains at least one special character
    """
    if len(password) < 8:
        return False
    
    has_upper = any(c.isupper() for c in password)
    has_lower = any(c.islower() for c in password)
    has_digit = any(c.isdigit() for c in password)
    has_special = any(c in "!@#$%^&*()_+-=[]{}|;:,.<>?" for c in password)
    
    return has_upper and has_lower and has_digit and has_special


def sanitize_input(input_string: str, max_length: int = 1000) -> str:
    """
    Sanitize user input to prevent XSS and other attacks
    
    Args:
        input_string: Raw input string
        max_length: Maximum allowed length
    
    Returns:
        Sanitized string
    """
    if not input_string:
        return ""
    
    # Truncate to max length
    sanitized = input_string[:max_length]
    
    # Remove null bytes and control characters
    sanitized = ''.join(char for char in sanitized if ord(char) >= 32 or char in '\t\n\r')
    
    # Remove potential XSS patterns
    dangerous_patterns = ['<script', '</script', 'javascript:', 'vbscript:', 'onload=', 'onerror=']
    for pattern in dangerous_patterns:
        sanitized = sanitized.replace(pattern.lower(), '')
        sanitized = sanitized.replace(pattern.upper(), '')
    
    # Strip whitespace
    sanitized = sanitized.strip()
    
    return sanitized


def validate_email_format(email: str) -> bool:
    """
    Validate email format with additional security checks
    
    Args:
        email: Email address to validate
    
    Returns:
        True if valid, False otherwise
    """
    import re
    
    if not email or len(email) > 254:  # RFC 5321 limit
        return False
    
    # Basic email regex
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    if not re.match(pattern, email):
        return False
    
    # Additional security checks
    local, domain = email.rsplit('@', 1)
    
    # Local part checks
    if len(local) > 64:  # RFC 5321 limit
        return False
    
    # Domain part checks
    if len(domain) > 253:  # RFC 5321 limit
        return False
    
    # Check for suspicious patterns
    suspicious_patterns = ['..', '--', '__']
    for pattern in suspicious_patterns:
        if pattern in email:
            return False
    
    return True


def generate_secure_filename(filename: str) -> str:
    """
    Generate a secure filename by removing dangerous characters
    
    Args:
        filename: Original filename
    
    Returns:
        Secure filename
    """
    import re
    import uuid
    
    # Remove path separators and dangerous characters
    secure_name = re.sub(r'[^\w\-_\.]', '', filename)
    
    # Limit length
    if len(secure_name) > 100:
        name, ext = os.path.splitext(secure_name)
        secure_name = name[:90] + ext
    
    # Add UUID prefix to prevent conflicts
    unique_prefix = str(uuid.uuid4())[:8]
    secure_name = f"{unique_prefix}_{secure_name}"
    
    return secure_name


def check_rate_limit(user_id: Optional[int], action: str, limit: int, window: int) -> bool:
    """
    Check if user has exceeded rate limit for a specific action
    
    Args:
        user_id: User ID (None for anonymous users)
        action: Action being performed
        limit: Maximum number of actions allowed
        window: Time window in seconds
    
    Returns:
        True if within rate limit, False if exceeded
    
    Note: This is a basic implementation. In production, use Redis or similar.
    """
    # This is a placeholder implementation
    # In production, implement with Redis or database-backed rate limiting
    return True


def mask_sensitive_data(data: str, mask_char: str = "*", visible_chars: int = 4) -> str:
    """
    Mask sensitive data for logging
    
    Args:
        data: Sensitive data to mask
        mask_char: Character to use for masking
        visible_chars: Number of characters to leave visible at the end
    
    Returns:
        Masked string
    """
    if not data or len(data) <= visible_chars:
        return mask_char * len(data) if data else ""
    
    visible_part = data[-visible_chars:]
    masked_part = mask_char * (len(data) - visible_chars)
    
    return masked_part + visible_part