"""
Better-Auth integration for user authentication.
"""

import os
import jwt
import bcrypt
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from fastapi import HTTPException, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from book_backend.src.database.database import get_db
from book_backend.src.database import crud
from book_backend.src.database.models import User
import logging

logger = logging.getLogger(__name__)

# Security configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
JWT_ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30
REFRESH_TOKEN_EXPIRE_DAYS = 7

security = HTTPBearer()


class AuthService:
    """Authentication service using Better-Auth patterns."""
    
    @staticmethod
    def hash_password(password: str) -> str:
        """Hash password using bcrypt."""
        salt = bcrypt.gensalt()
        return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')
    
    @staticmethod
    def verify_password(password: str, hashed_password: str) -> bool:
        """Verify password against hash."""
        return bcrypt.checkpw(password.encode('utf-8'), hashed_password.encode('utf-8'))
    
    @staticmethod
    def create_access_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
        """Create JWT access token."""
        to_encode = data.copy()
        
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        
        to_encode.update({"exp": expire, "type": "access"})
        
        try:
            encoded_jwt = jwt.encode(to_encode, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
            return encoded_jwt
        except Exception as e:
            logger.error(f"Error creating access token: {e}")
            raise HTTPException(status_code=500, detail="Could not create access token")
    
    @staticmethod
    def create_refresh_token(data: Dict[str, Any]) -> str:
        """Create JWT refresh token."""
        to_encode = data.copy()
        expire = datetime.utcnow() + timedelta(days=REFRESH_TOKEN_EXPIRE_DAYS)
        to_encode.update({"exp": expire, "type": "refresh"})
        
        try:
            encoded_jwt = jwt.encode(to_encode, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
            return encoded_jwt
        except Exception as e:
            logger.error(f"Error creating refresh token: {e}")
            raise HTTPException(status_code=500, detail="Could not create refresh token")
    
    @staticmethod
    def verify_token(token: str, token_type: str = "access") -> Dict[str, Any]:
        """Verify and decode JWT token."""
        try:
            payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
            
            # Check token type
            if payload.get("type") != token_type:
                raise HTTPException(status_code=401, detail="Invalid token type")
            
            # Check expiration
            exp = payload.get("exp")
            if exp and datetime.utcnow() > datetime.fromtimestamp(exp):
                raise HTTPException(status_code=401, detail="Token expired")
            
            return payload
            
        except jwt.ExpiredSignatureError:
            raise HTTPException(status_code=401, detail="Token expired")
        except jwt.JWTError as e:
            logger.error(f"JWT verification error: {e}")
            raise HTTPException(status_code=401, detail="Could not validate credentials")
    
    @staticmethod
    def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
        """Authenticate user with email and password."""
        try:
            user = crud.get_user_by_email(db, email)
            if not user:
                return None
            
            if not AuthService.verify_password(password, user.hashed_password):
                return None
            
            # Update last login
            user.last_login = datetime.utcnow()
            db.commit()
            
            return user
            
        except Exception as e:
            logger.error(f"Authentication error: {e}")
            return None
    
    @staticmethod
    def create_user(db: Session, email: str, password: str, full_name: str = None) -> User:
        """Create new user account."""
        try:
            # Check if user already exists
            existing_user = crud.get_user_by_email(db, email)
            if existing_user:
                raise HTTPException(status_code=400, detail="Email already registered")
            
            # Hash password
            hashed_password = AuthService.hash_password(password)
            
            # Create user
            user_data = {
                "email": email,
                "hashed_password": hashed_password,
                "full_name": full_name,
                "is_active": True,
                "created_at": datetime.utcnow()
            }
            
            user = crud.create_user(db, user_data)
            return user
            
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"User creation error: {e}")
            raise HTTPException(status_code=500, detail="Could not create user")


def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> User:
    """Get current authenticated user."""
    try:
        # Extract token from credentials
        token = credentials.credentials
        
        # Verify token
        payload = AuthService.verify_token(token, "access")
        
        # Get user ID from token
        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token payload")
        
        # Get user from database
        user = crud.get_user(db, user_id)
        if not user:
            raise HTTPException(status_code=401, detail="User not found")
        
        if not user.is_active:
            raise HTTPException(status_code=401, detail="User account disabled")
        
        return user
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get current user error: {e}")
        raise HTTPException(status_code=401, detail="Could not validate credentials")


def get_optional_user(
    request: Request,
    db: Session = Depends(get_db)
) -> Optional[User]:
    """Get current user if authenticated, otherwise return None."""
    try:
        # Check for Authorization header
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            return None
        
        # Extract token
        token = auth_header.split(" ")[1]
        
        # Verify token
        payload = AuthService.verify_token(token, "access")
        
        # Get user
        user_id = payload.get("sub")
        if user_id:
            user = crud.get_user(db, user_id)
            if user and user.is_active:
                return user
        
        return None
        
    except Exception:
        # Silently fail for optional authentication
        return None


class AuthMiddleware:
    """Authentication middleware for request processing."""
    
    @staticmethod
    def require_auth(func):
        """Decorator to require authentication for endpoints."""
        def wrapper(*args, **kwargs):
            # This would be implemented as a FastAPI dependency
            return func(*args, **kwargs)
        return wrapper
    
    @staticmethod
    def require_role(required_role: str):
        """Decorator to require specific role for endpoints."""
        def decorator(func):
            def wrapper(current_user: User = Depends(get_current_user), *args, **kwargs):
                if not hasattr(current_user, 'role') or current_user.role != required_role:
                    raise HTTPException(status_code=403, detail="Insufficient permissions")
                return func(current_user=current_user, *args, **kwargs)
            return wrapper
        return decorator


# OAuth providers configuration (for future implementation)
OAUTH_PROVIDERS = {
    "google": {
        "client_id": os.getenv("GOOGLE_CLIENT_ID"),
        "client_secret": os.getenv("GOOGLE_CLIENT_SECRET"),
        "redirect_uri": os.getenv("GOOGLE_REDIRECT_URI"),
        "scope": "openid email profile"
    },
    "github": {
        "client_id": os.getenv("GITHUB_CLIENT_ID"),
        "client_secret": os.getenv("GITHUB_CLIENT_SECRET"),
        "redirect_uri": os.getenv("GITHUB_REDIRECT_URI"),
        "scope": "user:email"
    }
}


class OAuthService:
    """OAuth service for third-party authentication."""
    
    @staticmethod
    def get_oauth_url(provider: str, state: str = None) -> str:
        """Generate OAuth authorization URL."""
        if provider not in OAUTH_PROVIDERS:
            raise HTTPException(status_code=400, detail="Unsupported OAuth provider")
        
        config = OAUTH_PROVIDERS[provider]
        
        if provider == "google":
            base_url = "https://accounts.google.com/o/oauth2/v2/auth"
            params = {
                "client_id": config["client_id"],
                "redirect_uri": config["redirect_uri"],
                "scope": config["scope"],
                "response_type": "code",
                "state": state or ""
            }
        elif provider == "github":
            base_url = "https://github.com/login/oauth/authorize"
            params = {
                "client_id": config["client_id"],
                "redirect_uri": config["redirect_uri"],
                "scope": config["scope"],
                "state": state or ""
            }
        
        # Build URL with parameters
        param_string = "&".join([f"{k}={v}" for k, v in params.items()])
        return f"{base_url}?{param_string}"
    
    @staticmethod
    async def handle_oauth_callback(provider: str, code: str, db: Session) -> Dict[str, Any]:
        """Handle OAuth callback and create/login user."""
        # This would implement the OAuth flow
        # For now, return a placeholder
        raise HTTPException(status_code=501, detail="OAuth implementation pending")


# Session management
class SessionManager:
    """Manage user sessions and tokens."""
    
    @staticmethod
    def create_session(user: User) -> Dict[str, str]:
        """Create new session with access and refresh tokens."""
        user_data = {
            "sub": str(user.id),
            "email": user.email,
            "full_name": user.full_name
        }
        
        access_token = AuthService.create_access_token(user_data)
        refresh_token = AuthService.create_refresh_token({"sub": str(user.id)})
        
        return {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer"
        }
    
    @staticmethod
    def refresh_session(refresh_token: str, db: Session) -> Dict[str, str]:
        """Refresh access token using refresh token."""
        try:
            # Verify refresh token
            payload = AuthService.verify_token(refresh_token, "refresh")
            user_id = payload.get("sub")
            
            if not user_id:
                raise HTTPException(status_code=401, detail="Invalid refresh token")
            
            # Get user
            user = crud.get_user(db, user_id)
            if not user or not user.is_active:
                raise HTTPException(status_code=401, detail="User not found or inactive")
            
            # Create new session
            return SessionManager.create_session(user)
            
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Session refresh error: {e}")
            raise HTTPException(status_code=401, detail="Could not refresh session")
    
    @staticmethod
    def revoke_session(user_id: str, db: Session) -> bool:
        """Revoke user session (logout)."""
        try:
            # In a full implementation, you would:
            # 1. Add token to blacklist
            # 2. Update user's last_logout timestamp
            # 3. Clear any cached session data
            
            user = crud.get_user(db, user_id)
            if user:
                user.last_logout = datetime.utcnow()
                db.commit()
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Session revocation error: {e}")
            return False