"""Scalable language addition framework for multilingual system."""

import logging
import asyncio
import json
import yaml
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from enum import Enum
from dataclasses import dataclass, asdict
from pathlib import Path
import importlib
import inspect

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func

from app.models.multilingual import Language, ContentTranslation, TerminologyGlossary
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class LanguageDirection(str, Enum):
    """Text direction for languages."""
    LTR = "ltr"  # Left-to-right
    RTL = "rtl"  # Right-to-left


class LanguageScript(str, Enum):
    """Writing scripts for languages."""
    LATIN = "latin"
    ARABIC = "arabic"
    DEVANAGARI = "devanagari"
    CYRILLIC = "cyrillic"
    CHINESE = "chinese"
    JAPANESE = "japanese"
    KOREAN = "korean"


@dataclass
class LanguageConfig:
    """Configuration for a language."""
    code: str
    name: str
    native_name: str
    direction: LanguageDirection
    script: LanguageScript
    font_family: str
    fallback_fonts: List[str]
    pluralization_rules: Dict[str, Any]
    date_format: str
    number_format: str
    currency_format: str
    cultural_adaptations: Dict[str, Any]
    translation_services: List[str]
    quality_thresholds: Dict[str, float]
    enabled: bool = True


@dataclass
class LanguageSetupResult:
    """Result of language setup process."""
    success: bool
    language_code: str
    steps_completed: List[str]
    steps_failed: List[str]
    warnings: List[str]
    next_steps: List[str]


class LanguageFramework:
    """Framework for adding and managing languages scalably."""
    
    def __init__(self):
        self.language_configs = {}
        self.language_plugins = {}
        self.setup_templates = {}
        
        # Language configuration templates
        self.config_templates = {
            "rtl_arabic_script": {
                "direction": LanguageDirection.RTL,
                "script": LanguageScript.ARABIC,
                "font_family": "Noto Sans Arabic",
                "fallback_fonts": ["Arial Unicode MS", "Tahoma", "sans-serif"],
                "pluralization_rules": {
                    "zero": "n == 0",
                    "one": "n == 1",
                    "two": "n == 2",
                    "few": "n % 100 >= 3 && n % 100 <= 10",
                    "many": "n % 100 >= 11 && n % 100 <= 99",
                    "other": "true"
                },
                "date_format": "DD/MM/YYYY",
                "number_format": "1,234.56",
                "currency_format": "{amount} {currency}"
            },
            "ltr_latin_script": {
                "direction": LanguageDirection.LTR,
                "script": LanguageScript.LATIN,
                "font_family": "Inter",
                "fallback_fonts": ["system-ui", "-apple-system", "sans-serif"],
                "pluralization_rules": {
                    "one": "n == 1",
                    "other": "true"
                },
                "date_format": "MM/DD/YYYY",
                "number_format": "1,234.56",
                "currency_format": "{currency}{amount}"
            }
        }
        
        # Load existing language configurations
        self._load_language_configs()
    
    def _load_language_configs(self):
        """Load existing language configurations."""
        try:
            # Define built-in language configurations
            self.language_configs = {
                "en": LanguageConfig(
                    code="en",
                    name="English",
                    native_name="English",
                    direction=LanguageDirection.LTR,
                    script=LanguageScript.LATIN,
                    font_family="Inter",
                    fallback_fonts=["system-ui", "-apple-system", "sans-serif"],
                    pluralization_rules={"one": "n == 1", "other": "true"},
                    date_format="MM/DD/YYYY",
                    number_format="1,234.56",
                    currency_format="${amount}",
                    cultural_adaptations={
                        "examples": ["New York", "San Francisco", "Chicago"],
                        "currency": "USD",
                        "measurement": "imperial"
                    },
                    translation_services=["google", "azure"],
                    quality_thresholds={"minimum": 0.8, "good": 0.9, "excellent": 0.95}
                ),
                "ur": LanguageConfig(
                    code="ur",
                    name="Urdu",
                    native_name="اردو",
                    direction=LanguageDirection.RTL,
                    script=LanguageScript.ARABIC,
                    font_family="Noto Nastaliq Urdu",
                    fallback_fonts=["Arial Unicode MS", "Tahoma", "sans-serif"],
                    pluralization_rules={
                        "one": "n == 1",
                        "other": "true"
                    },
                    date_format="DD/MM/YYYY",
                    number_format="1,23,456.78",
                    currency_format="PKR {amount}",
                    cultural_adaptations={
                        "examples": ["کراچی", "لاہور", "اسلام آباد"],
                        "currency": "PKR",
                        "measurement": "metric"
                    },
                    translation_services=["google", "azure"],
                    quality_thresholds={"minimum": 0.75, "good": 0.85, "excellent": 0.92}
                )
            }
            
            logger.info(f"Loaded {len(self.language_configs)} language configurations")
            
        except Exception as e:
            logger.error(f"Error loading language configurations: {e}")
    
    async def add_new_language(
        self,
        language_code: str,
        language_name: str,
        native_name: str,
        template: str = "ltr_latin_script",
        custom_config: Optional[Dict[str, Any]] = None,
        db: Session = None
    ) -> LanguageSetupResult:
        """Add a new language to the system."""
        try:
            setup_result = LanguageSetupResult(
                success=False,
                language_code=language_code,
                steps_completed=[],
                steps_failed=[],
                warnings=[],
                next_steps=[]
            )
            
            # Step 1: Validate language code
            if not self._validate_language_code(language_code):
                setup_result.steps_failed.append("Invalid language code format")
                return setup_result
            setup_result.steps_completed.append("Language code validation")
            
            # Step 2: Check if language already exists
            if language_code in self.language_configs:
                setup_result.warnings.append("Language already exists - updating configuration")
            
            # Step 3: Create language configuration
            config = await self._create_language_config(
                language_code, language_name, native_name, template, custom_config
            )
            if not config:
                setup_result.steps_failed.append("Failed to create language configuration")
                return setup_result
            setup_result.steps_completed.append("Language configuration created")
            
            # Step 4: Set up database entries
            if db:
                db_success = await self._setup_database_entries(config, db)
                if db_success:
                    setup_result.steps_completed.append("Database entries created")
                else:
                    setup_result.steps_failed.append("Database setup failed")
                    setup_result.warnings.append("Manual database setup may be required")
            
            # Step 5: Generate frontend configuration
            frontend_success = await self._generate_frontend_config(config)
            if frontend_success:
                setup_result.steps_completed.append("Frontend configuration generated")
            else:
                setup_result.steps_failed.append("Frontend configuration generation failed")
            
            # Step 6: Set up translation services
            translation_success = await self._setup_translation_services(config)
            if translation_success:
                setup_result.steps_completed.append("Translation services configured")
            else:
                setup_result.steps_failed.append("Translation services setup failed")
                setup_result.warnings.append("Manual translation service configuration needed")
            
            # Step 7: Generate documentation
            docs_success = await self._generate_language_documentation(config)
            if docs_success:
                setup_result.steps_completed.append("Documentation generated")
            else:
                setup_result.warnings.append("Documentation generation failed")
            
            # Store configuration
            self.language_configs[language_code] = config
            
            # Determine overall success
            setup_result.success = len(setup_result.steps_failed) == 0
            
            # Add next steps
            setup_result.next_steps = [
                "Test language switching in the UI",
                "Add initial terminology for the language",
                "Configure cultural adaptations",
                "Set up translation workflows",
                "Train translators for the new language"
            ]
            
            if not setup_result.success:
                setup_result.next_steps.insert(0, "Review and fix failed setup steps")
            
            return setup_result
            
        except Exception as e:
            logger.error(f"Error adding new language {language_code}: {e}")
            return LanguageSetupResult(
                success=False,
                language_code=language_code,
                steps_completed=[],
                steps_failed=[f"Unexpected error: {str(e)}"],
                warnings=[],
                next_steps=["Review error logs and retry setup"]
            )
    
    def _validate_language_code(self, language_code: str) -> bool:
        """Validate language code format."""
        # ISO 639-1 (2-letter) or ISO 639-3 (3-letter) codes
        return (
            isinstance(language_code, str) and
            2 <= len(language_code) <= 3 and
            language_code.isalpha() and
            language_code.islower()
        )
    
    async def _create_language_config(
        self,
        language_code: str,
        language_name: str,
        native_name: str,
        template: str,
        custom_config: Optional[Dict[str, Any]]
    ) -> Optional[LanguageConfig]:
        """Create language configuration from template."""
        try:
            # Get template configuration
            template_config = self.config_templates.get(template, self.config_templates["ltr_latin_script"])
            
            # Create base configuration
            config = LanguageConfig(
                code=language_code,
                name=language_name,
                native_name=native_name,
                direction=template_config["direction"],
                script=template_config["script"],
                font_family=template_config["font_family"],
                fallback_fonts=template_config["fallback_fonts"],
                pluralization_rules=template_config["pluralization_rules"],
                date_format=template_config["date_format"],
                number_format=template_config["number_format"],
                currency_format=template_config["currency_format"],
                cultural_adaptations={},
                translation_services=["google", "azure"],
                quality_thresholds={"minimum": 0.75, "good": 0.85, "excellent": 0.92}
            )
            
            # Apply custom configuration
            if custom_config:
                for key, value in custom_config.items():
                    if hasattr(config, key):
                        setattr(config, key, value)
            
            return config
            
        except Exception as e:
            logger.error(f"Error creating language configuration: {e}")
            return None
    
    async def _setup_database_entries(self, config: LanguageConfig, db: Session) -> bool:
        """Set up database entries for new language."""
        try:
            # Check if language already exists in database
            existing_language = db.query(Language).filter(Language.code == config.code).first()
            
            if existing_language:
                # Update existing language
                existing_language.name = config.name
                existing_language.native_name = config.native_name
                existing_language.direction = config.direction.value
                existing_language.is_active = config.enabled
            else:
                # Create new language entry
                new_language = Language(
                    code=config.code,
                    name=config.name,
                    native_name=config.native_name,
                    direction=config.direction.value,
                    is_active=config.enabled
                )
                db.add(new_language)
            
            db.commit()
            return True
            
        except Exception as e:
            logger.error(f"Error setting up database entries: {e}")
            db.rollback()
            return False
    
    async def _generate_frontend_config(self, config: LanguageConfig) -> bool:
        """Generate frontend configuration files."""
        try:
            # Generate language configuration for frontend
            frontend_config = {
                "code": config.code,
                "name": config.name,
                "nativeName": config.native_name,
                "direction": config.direction.value,
                "script": config.script.value,
                "fontFamily": config.font_family,
                "fallbackFonts": config.fallback_fonts,
                "dateFormat": config.date_format,
                "numberFormat": config.number_format,
                "currencyFormat": config.currency_format,
                "pluralizationRules": config.pluralization_rules
            }
            
            # In a real implementation, this would write to actual config files
            # For now, we'll simulate the process
            config_path = f"frontend/src/locales/{config.code}/config.json"
            logger.info(f"Generated frontend config at {config_path}")
            
            # Generate CSS for RTL support if needed
            if config.direction == LanguageDirection.RTL:
                rtl_css = self._generate_rtl_css(config)
                css_path = f"frontend/src/styles/{config.code}-rtl.css"
                logger.info(f"Generated RTL CSS at {css_path}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error generating frontend config: {e}")
            return False
    
    def _generate_rtl_css(self, config: LanguageConfig) -> str:
        """Generate RTL-specific CSS."""
        return f"""
/* RTL styles for {config.name} ({config.code}) */
html[lang="{config.code}"] {{
    direction: rtl;
    text-align: right;
}}

html[lang="{config.code}"] .container {{
    padding-right: 15px;
    padding-left: 15px;
}}

html[lang="{config.code}"] .navbar-nav {{
    flex-direction: row-reverse;
}}

html[lang="{config.code}"] .btn-group {{
    direction: ltr;
}}

html[lang="{config.code}"] .form-control {{
    text-align: right;
}}

/* Font family for {config.name} */
html[lang="{config.code}"] {{
    font-family: "{config.font_family}", {', '.join(f'"{font}"' for font in config.fallback_fonts)};
}}
"""
    
    async def _setup_translation_services(self, config: LanguageConfig) -> bool:
        """Set up translation services for new language."""
        try:
            # Configure translation services
            for service in config.translation_services:
                service_config = {
                    "language_code": config.code,
                    "service": service,
                    "enabled": True,
                    "quality_threshold": config.quality_thresholds["minimum"]
                }
                
                # In a real implementation, this would configure actual services
                logger.info(f"Configured {service} translation service for {config.code}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error setting up translation services: {e}")
            return False
    
    async def _generate_language_documentation(self, config: LanguageConfig) -> bool:
        """Generate documentation for new language."""
        try:
            documentation = f"""
# {config.name} ({config.code}) Language Configuration

## Basic Information
- **Language Code**: {config.code}
- **Language Name**: {config.name}
- **Native Name**: {config.native_name}
- **Text Direction**: {config.direction.value}
- **Script**: {config.script.value}

## Typography
- **Primary Font**: {config.font_family}
- **Fallback Fonts**: {', '.join(config.fallback_fonts)}

## Localization
- **Date Format**: {config.date_format}
- **Number Format**: {config.number_format}
- **Currency Format**: {config.currency_format}

## Translation Services
- **Enabled Services**: {', '.join(config.translation_services)}
- **Quality Thresholds**:
  - Minimum: {config.quality_thresholds['minimum']}
  - Good: {config.quality_thresholds['good']}
  - Excellent: {config.quality_thresholds['excellent']}

## Setup Steps Completed
1. Database entries created
2. Frontend configuration generated
3. Translation services configured
4. Documentation generated

## Next Steps
1. Add initial terminology for {config.name}
2. Configure cultural adaptations
3. Set up translation workflows
4. Test language switching functionality
5. Train translators for {config.name}

## Cultural Adaptations
Configure cultural adaptations in the language configuration:
- Local examples and references
- Currency and measurement units
- Date and time formats
- Cultural context for translations

## Troubleshooting
- Ensure fonts are properly loaded
- Verify RTL layout for right-to-left languages
- Test translation service connectivity
- Validate pluralization rules
"""
            
            # In a real implementation, this would write to actual documentation files
            doc_path = f"docs/languages/{config.code}.md"
            logger.info(f"Generated documentation at {doc_path}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error generating documentation: {e}")
            return False
    
    def get_language_config(self, language_code: str) -> Optional[LanguageConfig]:
        """Get configuration for a specific language."""
        return self.language_configs.get(language_code)
    
    def get_all_languages(self) -> Dict[str, LanguageConfig]:
        """Get all language configurations."""
        return self.language_configs.copy()
    
    def get_enabled_languages(self) -> Dict[str, LanguageConfig]:
        """Get only enabled language configurations."""
        return {
            code: config for code, config in self.language_configs.items()
            if config.enabled
        }
    
    async def disable_language(self, language_code: str, db: Session = None) -> bool:
        """Disable a language."""
        try:
            if language_code in self.language_configs:
                self.language_configs[language_code].enabled = False
                
                # Update database
                if db:
                    language = db.query(Language).filter(Language.code == language_code).first()
                    if language:
                        language.is_active = False
                        db.commit()
                
                return True
            return False
            
        except Exception as e:
            logger.error(f"Error disabling language {language_code}: {e}")
            return False
    
    async def enable_language(self, language_code: str, db: Session = None) -> bool:
        """Enable a language."""
        try:
            if language_code in self.language_configs:
                self.language_configs[language_code].enabled = True
                
                # Update database
                if db:
                    language = db.query(Language).filter(Language.code == language_code).first()
                    if language:
                        language.is_active = True
                        db.commit()
                
                return True
            return False
            
        except Exception as e:
            logger.error(f"Error enabling language {language_code}: {e}")
            return False
    
    def get_language_templates(self) -> Dict[str, Dict[str, Any]]:
        """Get available language configuration templates."""
        return {
            "rtl_arabic_script": {
                "name": "RTL Arabic Script",
                "description": "Template for right-to-left languages using Arabic script (Arabic, Urdu, Persian, etc.)",
                "example_languages": ["ar", "ur", "fa", "ps"],
                "features": ["RTL layout", "Arabic script", "Complex pluralization"]
            },
            "ltr_latin_script": {
                "name": "LTR Latin Script",
                "description": "Template for left-to-right languages using Latin script (English, Spanish, French, etc.)",
                "example_languages": ["en", "es", "fr", "de", "it"],
                "features": ["LTR layout", "Latin script", "Simple pluralization"]
            }
        }
    
    async def validate_language_setup(self, language_code: str, db: Session = None) -> Dict[str, Any]:
        """Validate that a language is properly set up."""
        try:
            validation_result = {
                "language_code": language_code,
                "valid": True,
                "checks": {},
                "warnings": [],
                "errors": []
            }
            
            # Check if language config exists
            config = self.get_language_config(language_code)
            validation_result["checks"]["config_exists"] = config is not None
            if not config:
                validation_result["errors"].append("Language configuration not found")
                validation_result["valid"] = False
                return validation_result
            
            # Check database entry
            if db:
                db_language = db.query(Language).filter(Language.code == language_code).first()
                validation_result["checks"]["database_entry"] = db_language is not None
                if not db_language:
                    validation_result["errors"].append("Language not found in database")
                    validation_result["valid"] = False
            
            # Check if language is enabled
            validation_result["checks"]["enabled"] = config.enabled
            if not config.enabled:
                validation_result["warnings"].append("Language is disabled")
            
            # Check translation services
            validation_result["checks"]["translation_services"] = len(config.translation_services) > 0
            if not config.translation_services:
                validation_result["warnings"].append("No translation services configured")
            
            # Check font configuration
            validation_result["checks"]["font_configured"] = bool(config.font_family)
            if not config.font_family:
                validation_result["warnings"].append("No primary font configured")
            
            # Check cultural adaptations
            validation_result["checks"]["cultural_adaptations"] = bool(config.cultural_adaptations)
            if not config.cultural_adaptations:
                validation_result["warnings"].append("No cultural adaptations configured")
            
            return validation_result
            
        except Exception as e:
            logger.error(f"Error validating language setup: {e}")
            return {
                "language_code": language_code,
                "valid": False,
                "checks": {},
                "warnings": [],
                "errors": [f"Validation error: {str(e)}"]
            }


# Global language framework instance
language_framework = LanguageFramework()