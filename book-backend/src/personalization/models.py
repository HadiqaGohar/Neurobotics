"""
Data models for the personalization service.
"""

from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field, validator
from enum import Enum
from datetime import datetime


class ExperienceLevel(str, Enum):
    """User experience levels for software and hardware."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate" 
    ADVANCED = "advanced"
    EXPERT = "expert"


class SoftwareCategory(str, Enum):
    """Software development categories."""
    WEB_DEVELOPMENT = "web_development"
    MOBILE_DEVELOPMENT = "mobile_development"
    DESKTOP_DEVELOPMENT = "desktop_development"
    DEVOPS = "devops"
    DATA_SCIENCE = "data_science"
    MACHINE_LEARNING = "machine_learning"
    GAME_DEVELOPMENT = "game_development"
    EMBEDDED_SYSTEMS = "embedded_systems"
    CYBERSECURITY = "cybersecurity"
    CLOUD_COMPUTING = "cloud_computing"


class HardwareCategory(str, Enum):
    """Hardware development categories."""
    EMBEDDED_SYSTEMS = "embedded_systems"
    IOT_DEVICES = "iot_devices"
    ROBOTICS = "robotics"
    ELECTRONICS = "electronics"
    MICROCONTROLLERS = "microcontrollers"
    SENSORS = "sensors"
    NETWORKING_HARDWARE = "networking_hardware"
    AUTOMOTIVE = "automotive"
    AEROSPACE = "aerospace"
    INDUSTRIAL_AUTOMATION = "industrial_automation"


class ContentComplexity(str, Enum):
    """Content complexity levels."""
    SIMPLE = "simple"
    MODERATE = "moderate"
    DETAILED = "detailed"
    COMPREHENSIVE = "comprehensive"


class SoftwareBackground(BaseModel):
    """User's software development background."""
    categories: List[SoftwareCategory] = Field(default_factory=list, max_items=10)
    experience_level: ExperienceLevel = ExperienceLevel.BEGINNER
    preferred_languages: List[str] = Field(default_factory=list, max_items=8)
    frameworks: List[str] = Field(default_factory=list, max_items=10)
    
    @validator('preferred_languages')
    def validate_languages(cls, v):
        """Validate programming languages."""
        valid_languages = {
            'python', 'javascript', 'typescript', 'java', 'c++', 'c', 'c#',
            'go', 'rust', 'swift', 'kotlin', 'php', 'ruby', 'scala',
            'r', 'matlab', 'sql', 'html', 'css', 'bash', 'powershell'
        }
        validated_langs = [lang.lower().strip() for lang in v if lang.lower().strip() in valid_languages]
        return list(dict.fromkeys(validated_langs))  # Remove duplicates while preserving order
    
    @validator('frameworks')
    def validate_frameworks(cls, v):
        """Validate frameworks."""
        # Remove duplicates and empty strings
        return [fw.strip() for fw in v if fw.strip()]
    
    @validator('categories')
    def validate_categories(cls, v):
        """Validate software categories."""
        # Remove duplicates while preserving order
        return list(dict.fromkeys(v))


class HardwareBackground(BaseModel):
    """User's hardware development background."""
    categories: List[HardwareCategory] = Field(default_factory=list, max_items=8)
    experience_level: ExperienceLevel = ExperienceLevel.BEGINNER
    platforms: List[str] = Field(default_factory=list, max_items=6)
    components: List[str] = Field(default_factory=list, max_items=12)
    
    @validator('platforms')
    def validate_platforms(cls, v):
        """Validate hardware platforms."""
        valid_platforms = {
            'arduino', 'raspberry_pi', 'esp32', 'esp8266', 'stm32',
            'pic', 'arm', 'fpga', 'beaglebone', 'nvidia_jetson',
            'intel_nuc', 'teensy', 'micro_bit'
        }
        validated_platforms = [platform.lower().strip().replace(" ", "_") for platform in v 
                             if platform.lower().strip().replace(" ", "_") in valid_platforms]
        return list(dict.fromkeys(validated_platforms))  # Remove duplicates while preserving order
    
    @validator('components')
    def validate_components(cls, v):
        """Validate hardware components."""
        # Remove duplicates and empty strings
        return [comp.strip() for comp in v if comp.strip()]
    
    @validator('categories')
    def validate_categories(cls, v):
        """Validate hardware categories."""
        # Remove duplicates while preserving order
        return list(dict.fromkeys(v))


class UserPreferences(BaseModel):
    """Complete user preferences for content personalization."""
    user_id: str = Field(..., min_length=1, max_length=50)
    software_background: SoftwareBackground = Field(default_factory=SoftwareBackground)
    hardware_background: HardwareBackground = Field(default_factory=HardwareBackground)
    content_complexity: ContentComplexity = ContentComplexity.MODERATE
    explanation_depth: str = Field(default="standard", regex="^(overview|standard|detailed)$")
    example_style: str = Field(default="practical", regex="^(basic|practical|advanced)$")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
    @validator('user_id')
    def validate_user_id(cls, v):
        """Validate user ID."""
        if not v or not v.strip():
            raise ValueError("User ID cannot be empty")
        return v.strip()
    
    @validator('updated_at', always=True)
    def set_updated_at(cls, v, values):
        """Ensure updated_at is set to current time when preferences are modified."""
        return datetime.utcnow()
    
    class Config:
        """Pydantic configuration."""
        use_enum_values = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }
        validate_assignment = True


class ContentVariant(BaseModel):
    """Content variant for different personalization levels."""
    content_id: str
    chapter_id: str
    section_id: str
    variant_type: ExperienceLevel
    target_audience: Dict[str, Any] = Field(default_factory=dict)
    content: Dict[str, Any] = Field(default_factory=dict)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        """Pydantic configuration."""
        use_enum_values = True


class PersonalizationRequest(BaseModel):
    """Request for content personalization."""
    user_id: str
    chapter_id: str
    section_id: Optional[str] = None
    requested_complexity: Optional[ContentComplexity] = None
    force_regenerate: bool = False
    
    class Config:
        """Pydantic configuration."""
        use_enum_values = True


class PersonalizationResponse(BaseModel):
    """Response containing personalized content."""
    content_id: str
    personalized_content: Dict[str, Any]
    personalization_applied: Dict[str, Any]
    cache_hit: bool = False
    generation_time_ms: int = 0
    
    class Config:
        """Pydantic configuration."""
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class PreferenceUpdateRequest(BaseModel):
    """Request to update user preferences."""
    software_background: Optional[SoftwareBackground] = None
    hardware_background: Optional[HardwareBackground] = None
    content_complexity: Optional[ContentComplexity] = None
    explanation_depth: Optional[str] = Field(None, regex="^(overview|standard|detailed)$")
    example_style: Optional[str] = Field(None, regex="^(basic|practical|advanced)$")
    
    class Config:
        """Pydantic configuration."""
        use_enum_values = True


class PersonalizationStats(BaseModel):
    """Statistics about personalization usage."""
    total_requests: int = 0
    cache_hit_rate: float = 0.0
    average_generation_time_ms: float = 0.0
    popular_preferences: Dict[str, int] = Field(default_factory=dict)
    content_variants_count: int = 0
    
    class Config:
        """Pydantic configuration."""
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


# Database models (SQLAlchemy)
from sqlalchemy import Column, String, Text, DateTime, JSON, Integer, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()


class UserPreferencesDB(Base):
    """Database model for user preferences."""
    __tablename__ = "user_preferences"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, unique=True, index=True, nullable=False)
    software_background = Column(JSON, nullable=False, default={})
    hardware_background = Column(JSON, nullable=False, default={})
    content_complexity = Column(String, nullable=False, default="moderate")
    explanation_depth = Column(String, nullable=False, default="standard")
    example_style = Column(String, nullable=False, default="practical")
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationship to content variants
    content_variants = relationship("ContentVariantDB", back_populates="user_preferences")


class ContentVariantDB(Base):
    """Database model for content variants."""
    __tablename__ = "content_variants"
    
    id = Column(Integer, primary_key=True, index=True)
    content_id = Column(String, unique=True, index=True, nullable=False)
    chapter_id = Column(String, index=True, nullable=False)
    section_id = Column(String, index=True, nullable=True)
    variant_type = Column(String, nullable=False)
    target_audience = Column(JSON, nullable=False, default={})
    content = Column(JSON, nullable=False, default={})
    metadata = Column(JSON, nullable=False, default={})
    is_ai_generated = Column(Boolean, nullable=False, default=False)
    quality_score = Column(Integer, nullable=True)
    usage_count = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Foreign key to user preferences (optional, for user-specific variants)
    user_preferences_id = Column(Integer, ForeignKey("user_preferences.id"), nullable=True)
    user_preferences = relationship("UserPreferencesDB", back_populates="content_variants")


class PersonalizationLogDB(Base):
    """Database model for personalization request logs."""
    __tablename__ = "personalization_logs"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, index=True, nullable=False)
    chapter_id = Column(String, index=True, nullable=False)
    section_id = Column(String, index=True, nullable=True)
    requested_complexity = Column(String, nullable=True)
    applied_personalization = Column(JSON, nullable=False, default={})
    cache_hit = Column(Boolean, nullable=False, default=False)
    generation_time_ms = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)