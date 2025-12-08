"""Cultural Localization System for multilingual content adaptation."""

import logging
import re
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from enum import Enum
import json

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_

from app.models.multilingual import ContentTranslation, Language

logger = logging.getLogger(__name__)


class CulturalContext(str, Enum):
    """Cultural context categories."""
    RELIGIOUS = "religious"
    SOCIAL = "social"
    EDUCATIONAL = "educational"
    BUSINESS = "business"
    TECHNICAL = "technical"
    GENERAL = "general"


class LocalizationLevel(str, Enum):
    """Localization adaptation levels."""
    MINIMAL = "minimal"      # Basic translation only
    MODERATE = "moderate"    # Some cultural adaptation
    EXTENSIVE = "extensive"  # Full cultural adaptation


class CulturalLocalizationSystem:
    """System for adapting content to cultural contexts."""
    
    def __init__(self):
        # Cultural adaptation rules by language and context
        self.cultural_rules = {
            "ur": {  # Urdu/Pakistani context
                "greetings": {
                    "hello": "السلام علیکم",
                    "good morning": "صبح بخیر",
                    "good evening": "شام بخیر",
                    "goodbye": "خدا حافظ"
                },
                "honorifics": {
                    "mr": "جناب",
                    "mrs": "محترمہ",
                    "sir": "صاحب",
                    "madam": "بی بی"
                },
                "currency": {
                    "symbol": "₨",
                    "code": "PKR",
                    "format": "Rs. {amount}"
                },
                "date_format": "dd/mm/yyyy",
                "time_format": "12h",
                "number_format": "en-PK",
                "cultural_adaptations": {
                    "examples": {
                        # Replace Western examples with local ones
                        "john": "احمد",
                        "mary": "فاطمہ", 
                        "smith": "خان",
                        "new york": "کراچی",
                        "london": "لاہور",
                        "dollar": "روپیہ",
                        "christmas": "عید",
                        "sunday": "جمعہ"  # Friday is the holy day
                    },
                    "food_references": {
                        "pizza": "بریانی",
                        "burger": "کباب",
                        "coffee": "چائے",
                        "wine": "لسی"  # Replace alcohol with traditional drink
                    },
                    "educational_context": {
                        "university": "یونیورسٹی",
                        "college": "کالج",
                        "school": "سکول",
                        "grade": "جماعت",
                        "semester": "سمسٹر"
                    },
                    "business_context": {
                        "company": "کمپنی",
                        "office": "دفتر",
                        "meeting": "میٹنگ",
                        "project": "پروجیکٹ",
                        "deadline": "آخری تاریخ"
                    }
                },
                "sensitive_content": {
                    # Content that needs careful handling
                    "alcohol_references": ["wine", "beer", "alcohol", "bar", "pub"],
                    "pork_references": ["pork", "ham", "bacon"],
                    "gambling_references": ["casino", "gambling", "lottery", "bet"],
                    "inappropriate_imagery": ["revealing clothing", "intimate scenes"]
                },
                "religious_considerations": {
                    "prayer_times": True,
                    "ramadan_awareness": True,
                    "halal_food": True,
                    "islamic_calendar": True
                }
            }
        }
        
        # Programming-specific cultural adaptations
        self.programming_adaptations = {
            "ur": {
                "variable_names": {
                    # Suggest culturally appropriate variable names
                    "userName": "صارف_نام",
                    "firstName": "پہلا_نام", 
                    "lastName": "خاندانی_نام",
                    "phoneNumber": "فون_نمبر",
                    "emailAddress": "ای_میل_پتہ"
                },
                "example_data": {
                    # Use local names and data in examples
                    "names": ["احمد", "فاطمہ", "علی", "عائشہ", "حسن", "زینب"],
                    "cities": ["کراچی", "لاہور", "اسلام آباد", "فیصل آباد", "راولپنڈی"],
                    "companies": ["پاک ٹیلی کام", "نیشنل بینک", "پی آئی اے", "سٹیل ملز"],
                    "universities": ["کراچی یونیورسٹی", "پنجاب یونیورسٹی", "کوئیڈ اعظم یونیورسٹی"]
                },
                "code_comments": {
                    # Bilingual code comments
                    "style": "bilingual",  # English + Urdu
                    "format": "// {english_comment} - {urdu_comment}"
                }
            }
        }
        
        # Regional preferences
        self.regional_preferences = {
            "ur": {
                "country": "Pakistan",
                "timezone": "Asia/Karachi",
                "locale": "ur-PK",
                "currency": "PKR",
                "measurement_system": "metric",
                "phone_format": "+92-XXX-XXXXXXX",
                "postal_code_format": "XXXXX"
            }
        }
    
    async def localize_content(
        self,
        content: str,
        target_language: str,
        context: CulturalContext = CulturalContext.GENERAL,
        level: LocalizationLevel = LocalizationLevel.MODERATE,
        preserve_technical: bool = True
    ) -> Dict[str, Any]:
        """Localize content for cultural context."""
        try:
            if target_language not in self.cultural_rules:
                return {
                    "localized_content": content,
                    "adaptations_made": [],
                    "warnings": [f"No cultural rules defined for language: {target_language}"]
                }
            
            rules = self.cultural_rules[target_language]
            adaptations_made = []
            warnings = []
            localized_content = content
            
            # Apply localization based on level
            if level in [LocalizationLevel.MODERATE, LocalizationLevel.EXTENSIVE]:
                # Replace greetings
                localized_content, greeting_adaptations = self._adapt_greetings(
                    localized_content, rules.get("greetings", {})
                )
                adaptations_made.extend(greeting_adaptations)
                
                # Replace honorifics
                localized_content, honorific_adaptations = self._adapt_honorifics(
                    localized_content, rules.get("honorifics", {})
                )
                adaptations_made.extend(honorific_adaptations)
                
                # Adapt examples and references
                if "cultural_adaptations" in rules:
                    localized_content, example_adaptations = self._adapt_examples(
                        localized_content, 
                        rules["cultural_adaptations"],
                        context
                    )
                    adaptations_made.extend(example_adaptations)
            
            if level == LocalizationLevel.EXTENSIVE:
                # Handle sensitive content
                if "sensitive_content" in rules:
                    localized_content, sensitivity_warnings = self._handle_sensitive_content(
                        localized_content, rules["sensitive_content"]
                    )
                    warnings.extend(sensitivity_warnings)
                
                # Apply programming-specific adaptations
                if context == CulturalContext.TECHNICAL and not preserve_technical:
                    localized_content, prog_adaptations = self._adapt_programming_content(
                        localized_content, target_language
                    )
                    adaptations_made.extend(prog_adaptations)
            
            # Format numbers, dates, currency
            localized_content = self._format_regional_data(
                localized_content, target_language
            )
            
            return {
                "localized_content": localized_content,
                "adaptations_made": adaptations_made,
                "warnings": warnings,
                "localization_level": level.value,
                "cultural_context": context.value,
                "target_language": target_language
            }
            
        except Exception as e:
            logger.error(f"Error in content localization: {e}")
            return {
                "localized_content": content,
                "adaptations_made": [],
                "warnings": [f"Localization error: {str(e)}"],
                "error": str(e)
            }
    
    def _adapt_greetings(
        self, 
        content: str, 
        greeting_rules: Dict[str, str]
    ) -> Tuple[str, List[str]]:
        """Adapt greetings to cultural context."""
        adaptations = []
        adapted_content = content
        
        for english_greeting, local_greeting in greeting_rules.items():
            pattern = r'\b' + re.escape(english_greeting) + r'\b'
            if re.search(pattern, adapted_content, re.IGNORECASE):
                adapted_content = re.sub(
                    pattern, local_greeting, adapted_content, flags=re.IGNORECASE
                )
                adaptations.append(f"Greeting: '{english_greeting}' → '{local_greeting}'")
        
        return adapted_content, adaptations
    
    def _adapt_honorifics(
        self, 
        content: str, 
        honorific_rules: Dict[str, str]
    ) -> Tuple[str, List[str]]:
        """Adapt honorifics to cultural context."""
        adaptations = []
        adapted_content = content
        
        for english_honorific, local_honorific in honorific_rules.items():
            pattern = r'\b' + re.escape(english_honorific) + r'\.?\b'
            if re.search(pattern, adapted_content, re.IGNORECASE):
                adapted_content = re.sub(
                    pattern, local_honorific, adapted_content, flags=re.IGNORECASE
                )
                adaptations.append(f"Honorific: '{english_honorific}' → '{local_honorific}'")
        
        return adapted_content, adaptations
    
    def _adapt_examples(
        self, 
        content: str, 
        adaptation_rules: Dict[str, Dict[str, str]],
        context: CulturalContext
    ) -> Tuple[str, List[str]]:
        """Adapt examples and references to local context."""
        adaptations = []
        adapted_content = content
        
        # Apply general examples
        if "examples" in adaptation_rules:
            for english_term, local_term in adaptation_rules["examples"].items():
                pattern = r'\b' + re.escape(english_term) + r'\b'
                if re.search(pattern, adapted_content, re.IGNORECASE):
                    adapted_content = re.sub(
                        pattern, local_term, adapted_content, flags=re.IGNORECASE
                    )
                    adaptations.append(f"Example: '{english_term}' → '{local_term}'")
        
        # Apply context-specific adaptations
        context_key = f"{context.value}_context"
        if context_key in adaptation_rules:
            for english_term, local_term in adaptation_rules[context_key].items():
                pattern = r'\b' + re.escape(english_term) + r'\b'
                if re.search(pattern, adapted_content, re.IGNORECASE):
                    adapted_content = re.sub(
                        pattern, local_term, adapted_content, flags=re.IGNORECASE
                    )
                    adaptations.append(f"Context ({context.value}): '{english_term}' → '{local_term}'")
        
        # Food references
        if "food_references" in adaptation_rules:
            for english_food, local_food in adaptation_rules["food_references"].items():
                pattern = r'\b' + re.escape(english_food) + r'\b'
                if re.search(pattern, adapted_content, re.IGNORECASE):
                    adapted_content = re.sub(
                        pattern, local_food, adapted_content, flags=re.IGNORECASE
                    )
                    adaptations.append(f"Food: '{english_food}' → '{local_food}'")
        
        return adapted_content, adaptations
    
    def _handle_sensitive_content(
        self, 
        content: str, 
        sensitivity_rules: Dict[str, List[str]]
    ) -> Tuple[str, List[str]]:
        """Handle culturally sensitive content."""
        warnings = []
        adapted_content = content
        
        for category, sensitive_terms in sensitivity_rules.items():
            for term in sensitive_terms:
                pattern = r'\b' + re.escape(term) + r'\b'
                if re.search(pattern, adapted_content, re.IGNORECASE):
                    warnings.append(f"Sensitive content detected ({category}): '{term}'")
                    
                    # Replace with appropriate alternatives
                    if category == "alcohol_references":
                        adapted_content = re.sub(
                            pattern, "مشروب", adapted_content, flags=re.IGNORECASE
                        )
                    elif category == "pork_references":
                        adapted_content = re.sub(
                            pattern, "گوشت", adapted_content, flags=re.IGNORECASE
                        )
                    elif category == "gambling_references":
                        adapted_content = re.sub(
                            pattern, "کھیل", adapted_content, flags=re.IGNORECASE
                        )
        
        return adapted_content, warnings
    
    def _adapt_programming_content(
        self, 
        content: str, 
        target_language: str
    ) -> Tuple[str, List[str]]:
        """Adapt programming-specific content."""
        adaptations = []
        adapted_content = content
        
        if target_language not in self.programming_adaptations:
            return adapted_content, adaptations
        
        prog_rules = self.programming_adaptations[target_language]
        
        # Adapt variable names in examples
        if "variable_names" in prog_rules:
            for english_var, local_var in prog_rules["variable_names"].items():
                pattern = r'\b' + re.escape(english_var) + r'\b'
                if re.search(pattern, adapted_content):
                    adapted_content = re.sub(pattern, local_var, adapted_content)
                    adaptations.append(f"Variable: '{english_var}' → '{local_var}'")
        
        # Use local example data
        if "example_data" in prog_rules:
            example_data = prog_rules["example_data"]
            
            # Replace names in code examples
            if "names" in example_data:
                western_names = ["john", "jane", "bob", "alice", "mike", "sarah"]
                local_names = example_data["names"]
                
                for i, western_name in enumerate(western_names):
                    if i < len(local_names):
                        pattern = r'\b' + re.escape(western_name) + r'\b'
                        if re.search(pattern, adapted_content, re.IGNORECASE):
                            adapted_content = re.sub(
                                pattern, local_names[i], adapted_content, flags=re.IGNORECASE
                            )
                            adaptations.append(f"Name: '{western_name}' → '{local_names[i]}'")
        
        return adapted_content, adaptations
    
    def _format_regional_data(self, content: str, target_language: str) -> str:
        """Format numbers, dates, and currency for region."""
        if target_language not in self.regional_preferences:
            return content
        
        regional_prefs = self.regional_preferences[target_language]
        formatted_content = content
        
        # Format currency (basic implementation)
        currency_pattern = r'\$(\d+(?:\.\d{2})?)'
        if regional_prefs.get("currency") == "PKR":
            formatted_content = re.sub(
                currency_pattern, 
                lambda m: f"Rs. {m.group(1)}", 
                formatted_content
            )
        
        # Format phone numbers (basic implementation)
        phone_pattern = r'\b\d{3}-\d{3}-\d{4}\b'
        if regional_prefs.get("phone_format"):
            # This would implement proper phone number formatting
            pass
        
        return formatted_content
    
    async def get_cultural_guidelines(
        self, 
        target_language: str,
        context: CulturalContext = CulturalContext.GENERAL
    ) -> Dict[str, Any]:
        """Get cultural guidelines for content creation."""
        try:
            if target_language not in self.cultural_rules:
                return {"error": f"No guidelines available for {target_language}"}
            
            rules = self.cultural_rules[target_language]
            guidelines = {
                "language": target_language,
                "context": context.value,
                "guidelines": {
                    "greetings": {
                        "description": "Appropriate greetings for the culture",
                        "examples": rules.get("greetings", {})
                    },
                    "honorifics": {
                        "description": "Respectful forms of address",
                        "examples": rules.get("honorifics", {})
                    },
                    "sensitive_topics": {
                        "description": "Topics that require careful handling",
                        "categories": list(rules.get("sensitive_content", {}).keys())
                    }
                }
            }
            
            if "religious_considerations" in rules:
                guidelines["religious_considerations"] = rules["religious_considerations"]
            
            if target_language in self.regional_preferences:
                guidelines["regional_preferences"] = self.regional_preferences[target_language]
            
            return guidelines
            
        except Exception as e:
            logger.error(f"Error getting cultural guidelines: {e}")
            return {"error": str(e)}
    
    async def validate_cultural_appropriateness(
        self, 
        content: str,
        target_language: str,
        context: CulturalContext = CulturalContext.GENERAL
    ) -> Dict[str, Any]:
        """Validate content for cultural appropriateness."""
        try:
            validation_result = {
                "is_appropriate": True,
                "issues": [],
                "suggestions": [],
                "severity_score": 0  # 0-10, higher is more problematic
            }
            
            if target_language not in self.cultural_rules:
                return validation_result
            
            rules = self.cultural_rules[target_language]
            
            # Check for sensitive content
            if "sensitive_content" in rules:
                for category, sensitive_terms in rules["sensitive_content"].items():
                    for term in sensitive_terms:
                        if re.search(r'\b' + re.escape(term) + r'\b', content, re.IGNORECASE):
                            validation_result["is_appropriate"] = False
                            validation_result["issues"].append({
                                "type": "sensitive_content",
                                "category": category,
                                "term": term,
                                "severity": "high" if category in ["religious", "alcohol_references"] else "medium"
                            })
                            validation_result["severity_score"] += 3 if category in ["religious", "alcohol_references"] else 2
                            
                            # Provide suggestions
                            if category == "alcohol_references":
                                validation_result["suggestions"].append(
                                    f"Consider replacing '{term}' with a non-alcoholic alternative"
                                )
                            elif category == "pork_references":
                                validation_result["suggestions"].append(
                                    f"Consider replacing '{term}' with 'halal meat' or general 'meat'"
                                )
            
            # Check for missing cultural adaptations
            if "cultural_adaptations" in rules and "examples" in rules["cultural_adaptations"]:
                western_terms = list(rules["cultural_adaptations"]["examples"].keys())
                for term in western_terms:
                    if re.search(r'\b' + re.escape(term) + r'\b', content, re.IGNORECASE):
                        validation_result["suggestions"].append(
                            f"Consider localizing '{term}' to '{rules['cultural_adaptations']['examples'][term]}'"
                        )
            
            return validation_result
            
        except Exception as e:
            logger.error(f"Error validating cultural appropriateness: {e}")
            return {"error": str(e)}
    
    async def generate_localized_examples(
        self, 
        content_type: str,
        target_language: str,
        context: CulturalContext = CulturalContext.TECHNICAL
    ) -> Dict[str, Any]:
        """Generate culturally appropriate examples."""
        try:
            if target_language not in self.programming_adaptations:
                return {"error": f"No example data available for {target_language}"}
            
            prog_rules = self.programming_adaptations[target_language]
            examples = {}
            
            if content_type == "programming" and "example_data" in prog_rules:
                example_data = prog_rules["example_data"]
                
                examples = {
                    "variable_examples": {
                        "names": example_data.get("names", []),
                        "cities": example_data.get("cities", []),
                        "companies": example_data.get("companies", [])
                    },
                    "code_examples": {
                        "user_class": f"""
class صارف:
    def __init__(self, نام, عمر):
        self.نام = نام  # User name - صارف کا نام
        self.عمر = عمر    # Age - عمر
    
    def معلومات_دکھائیں(self):  # Show info - معلومات دکھائیں
        print(f"نام: {{self.نام}}, عمر: {{self.عمر}}")
""",
                        "database_example": f"""
# Database connection - ڈیٹابیس کنکشن
کنکشن = sqlite3.connect('{example_data["companies"][0]}.db')
کرسر = کنکشن.cursor()

# Create table - ٹیبل بنائیں
کرسر.execute('''
    CREATE TABLE صارفین (
        آئی_ڈی INTEGER PRIMARY KEY,
        نام TEXT NOT NULL,
        شہر TEXT
    )
''')
"""
                    }
                }
            
            return {
                "target_language": target_language,
                "context": context.value,
                "examples": examples
            }
            
        except Exception as e:
            logger.error(f"Error generating localized examples: {e}")
            return {"error": str(e)}


# Global cultural localization instance
cultural_localization = CulturalLocalizationSystem()