"""
Configuration for the personalization service.
"""

import os
from typing import Dict, Any
from pydantic import BaseSettings


class PersonalizationConfig(BaseSettings):
    """Configuration settings for personalization service."""
    
    # Cache settings
    cache_ttl_seconds: int = 3600  # 1 hour
    max_cache_size: int = 1000
    
    # Content generation settings
    max_generation_time_seconds: int = 30
    default_content_complexity: str = "moderate"
    
    # AI service settings (for future Phase 4)
    ai_service_enabled: bool = False
    ai_service_url: str = ""
    ai_service_api_key: str = ""
    ai_max_tokens: int = 2000
    ai_temperature: float = 0.7
    
    # Performance settings
    max_concurrent_requests: int = 100
    request_timeout_seconds: int = 30
    
    # Analytics settings
    enable_analytics: bool = True
    analytics_batch_size: int = 100
    
    # Content variant settings
    max_variants_per_content: int = 10
    variant_cleanup_days: int = 30
    
    class Config:
        env_prefix = "PERSONALIZATION_"
        case_sensitive = False


# Global configuration instance
config = PersonalizationConfig()


def get_personalization_config() -> PersonalizationConfig:
    """Get personalization configuration."""
    return config


# Default personalization rules
DEFAULT_PERSONALIZATION_RULES = {
    "experience_levels": {
        "beginner": {
            "complexity_score": 1,
            "explanation_depth": "detailed",
            "code_complexity": "simple",
            "terminology_level": "basic"
        },
        "intermediate": {
            "complexity_score": 2,
            "explanation_depth": "standard",
            "code_complexity": "moderate",
            "terminology_level": "standard"
        },
        "advanced": {
            "complexity_score": 3,
            "explanation_depth": "concise",
            "code_complexity": "advanced",
            "terminology_level": "technical"
        },
        "expert": {
            "complexity_score": 4,
            "explanation_depth": "minimal",
            "code_complexity": "complex",
            "terminology_level": "expert"
        }
    },
    
    "software_categories": {
        "web_development": {
            "preferred_languages": ["javascript", "typescript", "html", "css"],
            "frameworks": ["react", "vue", "angular", "express", "django"],
            "focus_areas": ["frontend", "backend", "fullstack"]
        },
        "mobile_development": {
            "preferred_languages": ["swift", "kotlin", "java", "javascript"],
            "frameworks": ["react_native", "flutter", "ionic", "xamarin"],
            "focus_areas": ["ios", "android", "cross_platform"]
        },
        "data_science": {
            "preferred_languages": ["python", "r", "sql", "scala"],
            "frameworks": ["pandas", "numpy", "scikit_learn", "tensorflow"],
            "focus_areas": ["analysis", "visualization", "machine_learning"]
        },
        "devops": {
            "preferred_languages": ["bash", "python", "go", "yaml"],
            "frameworks": ["docker", "kubernetes", "terraform", "ansible"],
            "focus_areas": ["automation", "deployment", "monitoring"]
        }
    },
    
    "hardware_categories": {
        "embedded_systems": {
            "preferred_languages": ["c", "c++", "assembly"],
            "platforms": ["arduino", "stm32", "pic", "arm"],
            "focus_areas": ["microcontrollers", "real_time", "low_power"]
        },
        "iot_devices": {
            "preferred_languages": ["c++", "python", "javascript"],
            "platforms": ["esp32", "raspberry_pi", "arduino"],
            "focus_areas": ["connectivity", "sensors", "cloud_integration"]
        },
        "robotics": {
            "preferred_languages": ["c++", "python", "matlab"],
            "platforms": ["raspberry_pi", "nvidia_jetson", "arduino"],
            "focus_areas": ["control_systems", "computer_vision", "navigation"]
        }
    }
}


def get_personalization_rules() -> Dict[str, Any]:
    """Get default personalization rules."""
    return DEFAULT_PERSONALIZATION_RULES


# Content adaptation templates
CONTENT_TEMPLATES = {
    "explanation": {
        "beginner": "Let's start with the basics. {concept} is {simple_definition}. Think of it like {analogy}.",
        "intermediate": "{concept} is {standard_definition}. It works by {mechanism}.",
        "advanced": "{concept}: {technical_definition}. Key implementation details: {details}.",
        "expert": "{concept} - {concise_definition}. Performance considerations: {optimizations}."
    },
    
    "code_example": {
        "beginner": "Here's a simple example to get you started:",
        "intermediate": "Here's a practical example showing how to use this:",
        "advanced": "Here's an implementation demonstrating advanced usage:",
        "expert": "Optimized implementation with performance considerations:"
    },
    
    "hardware_reference": {
        "beginner": "For hardware implementation, you can use {platform} which is beginner-friendly.",
        "intermediate": "This can be implemented on {platform} using {components}.",
        "advanced": "Advanced implementation on {platform} with {advanced_components}.",
        "expert": "Production-ready implementation considering {constraints} and {optimizations}."
    }
}


def get_content_templates() -> Dict[str, Dict[str, str]]:
    """Get content adaptation templates."""
    return CONTENT_TEMPLATES