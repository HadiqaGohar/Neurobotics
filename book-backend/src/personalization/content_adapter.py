"""
Content adaptation engine for personalizing book content based on user preferences.
"""

import logging
from typing import Dict, Any, List, Optional, Tuple
from enum import Enum
from datetime import datetime

from .models import (
    UserPreferences, ExperienceLevel, SoftwareCategory, HardwareCategory,
    ContentComplexity
)
from .content_tagger import content_tagger, ContentTag

logger = logging.getLogger(__name__)


class AdaptationStrategy(str, Enum):
    """Content adaptation strategies."""
    EXPERIENCE_BASED = "experience_based"
    CATEGORY_BASED = "category_based"
    LANGUAGE_BASED = "language_based"
    PLATFORM_BASED = "platform_based"
    COMPLEXITY_BASED = "complexity_based"


class ContentType(str, Enum):
    """Types of content that can be adapted."""
    TEXT = "text"
    CODE_EXAMPLE = "code_example"
    DIAGRAM = "diagram"
    EXERCISE = "exercise"
    EXPLANATION = "explanation"
    HARDWARE_REFERENCE = "hardware_reference"


class ContentAdaptationEngine:
    """Core engine for adapting content based on user preferences."""
    
    def __init__(self):
        self.adaptation_rules = self._initialize_adaptation_rules()
        self.complexity_mappings = self._initialize_complexity_mappings()
        self.language_preferences = self._initialize_language_preferences()
        self.platform_mappings = self._initialize_platform_mappings()
    
    def adapt_content(
        self, 
        original_content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType = ContentType.TEXT
    ) -> Dict[str, Any]:
        """
        Adapt content based on user preferences.
        
        Args:
            original_content: The original content to adapt
            preferences: User's personalization preferences
            content_type: Type of content being adapted
            
        Returns:
            Adapted content with personalization applied
        """
        try:
            adapted_content = original_content.copy()
            
            # Apply different adaptation strategies
            adapted_content = self._apply_experience_adaptation(adapted_content, preferences, content_type)
            adapted_content = self._apply_category_adaptation(adapted_content, preferences, content_type)
            adapted_content = self._apply_language_adaptation(adapted_content, preferences, content_type)
            adapted_content = self._apply_platform_adaptation(adapted_content, preferences, content_type)
            adapted_content = self._apply_complexity_adaptation(adapted_content, preferences, content_type)
            
            # Tag the adapted content
            content_tags = content_tagger.tag_content(adapted_content)
            
            # Calculate content match score
            user_prefs_dict = {
                "experience_level": self._determine_target_experience(preferences).value,
                "software_categories": [cat.value for cat in preferences.software_background.categories],
                "hardware_categories": [cat.value for cat in preferences.hardware_background.categories],
                "preferred_languages": preferences.software_background.preferred_languages,
                "content_complexity": preferences.content_complexity.value,
                "example_style": preferences.example_style
            }
            
            match_score = content_tagger.match_content_to_preferences(content_tags, user_prefs_dict)
            
            # Add adaptation metadata
            adapted_content["adaptation_metadata"] = {
                "adapted_at": datetime.utcnow().isoformat(),
                "strategies_applied": self._get_applied_strategies(preferences),
                "target_experience": self._determine_target_experience(preferences).value,
                "content_type": content_type.value,
                "personalization_score": self._calculate_personalization_score(preferences),
                "content_tags": [tag.value for tag in content_tags],
                "match_score": match_score,
                "content_categories": content_tagger.get_content_categories(content_tags)
            }
            
            return adapted_content
            
        except Exception as e:
            logger.error(f"Error adapting content: {e}")
            return original_content
    
    def _apply_experience_adaptation(
        self, 
        content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType
    ) -> Dict[str, Any]:
        """Adapt content based on user's experience level."""
        target_level = self._determine_target_experience(preferences)
        
        if "text" in content:
            content["text"] = self._adapt_text_for_experience(content["text"], target_level)
        
        if "explanations" in content:
            content["explanations"] = self._adapt_explanations_for_experience(
                content["explanations"], target_level, preferences.explanation_depth
            )
        
        if "terminology" not in content:
            content["terminology"] = {}
        
        content["terminology"].update(self._get_terminology_for_level(target_level))
        
        return content
    
    def _apply_category_adaptation(
        self, 
        content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType
    ) -> Dict[str, Any]:
        """Adapt content based on user's software and hardware categories."""
        software_cats = preferences.software_background.categories
        hardware_cats = preferences.hardware_background.categories
        
        # Add category-specific examples
        if "examples" not in content:
            content["examples"] = []
        
        content["examples"].extend(self._generate_category_examples(software_cats, hardware_cats))
        
        # Add category-specific use cases
        if "use_cases" not in content:
            content["use_cases"] = []
        
        content["use_cases"].extend(self._generate_category_use_cases(software_cats, hardware_cats))
        
        return content
    
    def _apply_language_adaptation(
        self, 
        content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType
    ) -> Dict[str, Any]:
        """Adapt code examples based on preferred programming languages."""
        preferred_langs = preferences.software_background.preferred_languages
        
        if not preferred_langs:
            preferred_langs = ["python"]  # Default to Python
        
        if "code_examples" not in content:
            content["code_examples"] = []
        
        # Generate code examples in preferred languages
        for lang in preferred_langs[:3]:  # Limit to top 3 languages
            code_example = self._generate_code_example(content, lang, preferences)
            if code_example:
                content["code_examples"].append(code_example)
        
        return content
    
    def _apply_platform_adaptation(
        self, 
        content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType
    ) -> Dict[str, Any]:
        """Adapt hardware references based on preferred platforms."""
        preferred_platforms = preferences.hardware_background.platforms
        
        if not preferred_platforms:
            preferred_platforms = ["arduino", "raspberry_pi"]  # Default platforms
        
        if "hardware_examples" not in content:
            content["hardware_examples"] = []
        
        # Generate platform-specific examples
        for platform in preferred_platforms[:3]:  # Limit to top 3 platforms
            hw_example = self._generate_hardware_example(content, platform, preferences)
            if hw_example:
                content["hardware_examples"].append(hw_example)
        
        return content
    
    def _apply_complexity_adaptation(
        self, 
        content: Dict[str, Any], 
        preferences: UserPreferences,
        content_type: ContentType
    ) -> Dict[str, Any]:
        """Adapt content complexity based on user preferences."""
        complexity = preferences.content_complexity
        
        # Adjust content depth based on complexity preference
        if complexity == ContentComplexity.SIMPLE:
            content = self._simplify_content(content)
        elif complexity == ContentComplexity.COMPREHENSIVE:
            content = self._expand_content(content, preferences)
        
        # Add complexity indicators
        content["complexity_level"] = complexity.value
        content["estimated_reading_time"] = self._estimate_reading_time(content, complexity)
        
        return content
    
    def _determine_target_experience(self, preferences: UserPreferences) -> ExperienceLevel:
        """Determine the target experience level for content adaptation."""
        software_level = preferences.software_background.experience_level
        hardware_level = preferences.hardware_background.experience_level
        
        # Use the higher of the two experience levels
        level_order = [ExperienceLevel.BEGINNER, ExperienceLevel.INTERMEDIATE, 
                      ExperienceLevel.ADVANCED, ExperienceLevel.EXPERT]
        
        software_index = level_order.index(software_level)
        hardware_index = level_order.index(hardware_level)
        
        return level_order[max(software_index, hardware_index)]
    
    def _adapt_text_for_experience(self, text: str, level: ExperienceLevel) -> str:
        """Adapt text content based on experience level."""
        if level == ExperienceLevel.BEGINNER:
            # Add more explanatory text and simpler language
            return f"Let's start with the basics. {text} This concept is fundamental to understanding how systems work."
        elif level == ExperienceLevel.EXPERT:
            # Use more technical language and assume prior knowledge
            return f"As you know, {text} This builds on established patterns in system architecture."
        else:
            # Standard text for intermediate/advanced
            return text
    
    def _adapt_explanations_for_experience(
        self, 
        explanations: Dict[str, str], 
        level: ExperienceLevel,
        depth: str
    ) -> Dict[str, str]:
        """Adapt explanations based on experience level and depth preference."""
        adapted_explanations = explanations.copy()
        
        if level == ExperienceLevel.BEGINNER:
            if depth == "detailed":
                adapted_explanations["beginner_note"] = "Don't worry if this seems complex at first - we'll break it down step by step."
            adapted_explanations["prerequisites"] = "Basic understanding of programming concepts is helpful."
        
        elif level == ExperienceLevel.EXPERT:
            if depth == "overview":
                adapted_explanations["expert_summary"] = "Key implementation considerations and architectural implications."
            adapted_explanations["advanced_topics"] = "Consider performance implications and scalability factors."
        
        return adapted_explanations
    
    def _get_terminology_for_level(self, level: ExperienceLevel) -> Dict[str, str]:
        """Get appropriate terminology definitions for experience level."""
        if level == ExperienceLevel.BEGINNER:
            return {
                "api": "Application Programming Interface - a way for different software components to communicate",
                "framework": "A pre-built structure that provides common functionality for applications",
                "library": "A collection of pre-written code that you can use in your programs"
            }
        elif level == ExperienceLevel.EXPERT:
            return {
                "abstraction_layer": "Interface that hides implementation complexity",
                "dependency_injection": "Design pattern for providing dependencies externally",
                "microservice_architecture": "Distributed system design with loosely coupled services"
            }
        else:
            return {}
    
    def _generate_category_examples(
        self, 
        software_cats: List[SoftwareCategory], 
        hardware_cats: List[HardwareCategory]
    ) -> List[Dict[str, Any]]:
        """Generate examples based on user's category preferences."""
        examples = []
        
        for cat in software_cats[:3]:  # Limit to top 3 categories
            if cat == SoftwareCategory.WEB_DEVELOPMENT:
                examples.append({
                    "category": "web_development",
                    "title": "Web Application Example",
                    "description": "Building a responsive web interface with modern frameworks",
                    "technologies": ["HTML", "CSS", "JavaScript", "React"]
                })
            elif cat == SoftwareCategory.EMBEDDED_SYSTEMS:
                examples.append({
                    "category": "embedded_systems",
                    "title": "Embedded System Example",
                    "description": "Real-time sensor data processing on microcontrollers",
                    "technologies": ["C", "C++", "RTOS", "Hardware Abstraction Layer"]
                })
        
        for cat in hardware_cats[:2]:  # Limit to top 2 categories
            if cat == HardwareCategory.IOT_DEVICES:
                examples.append({
                    "category": "iot_devices",
                    "title": "IoT Device Example",
                    "description": "Connected sensor network for environmental monitoring",
                    "technologies": ["WiFi", "MQTT", "Sensors", "Cloud Integration"]
                })
        
        return examples
    
    def _generate_category_use_cases(
        self, 
        software_cats: List[SoftwareCategory], 
        hardware_cats: List[HardwareCategory]
    ) -> List[Dict[str, Any]]:
        """Generate use cases based on user's category preferences."""
        use_cases = []
        
        if SoftwareCategory.MACHINE_LEARNING in software_cats:
            use_cases.append({
                "title": "ML Model Deployment",
                "description": "Deploying machine learning models in production environments",
                "complexity": "advanced"
            })
        
        if HardwareCategory.ROBOTICS in hardware_cats:
            use_cases.append({
                "title": "Robot Control System",
                "description": "Implementing autonomous navigation and control algorithms",
                "complexity": "advanced"
            })
        
        return use_cases
    
    def _generate_code_example(
        self, 
        content: Dict[str, Any], 
        language: str, 
        preferences: UserPreferences
    ) -> Optional[Dict[str, Any]]:
        """Generate code example in specified language."""
        experience_level = self._determine_target_experience(preferences)
        
        # Basic code templates based on language and experience
        if language == "python":
            if experience_level == ExperienceLevel.BEGINNER:
                code = """# Simple Python example
def hello_world():
    print("Hello, World!")
    return True

# Call the function
result = hello_world()"""
            else:
                code = """# Advanced Python example with error handling
from typing import Optional
import logging

class DataProcessor:
    def __init__(self, config: dict):
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def process_data(self, data: list) -> Optional[dict]:
        try:
            # Process data with error handling
            result = {"processed": len(data), "status": "success"}
            self.logger.info(f"Processed {len(data)} items")
            return result
        except Exception as e:
            self.logger.error(f"Processing failed: {e}")
            return None"""
        
        elif language == "javascript":
            if experience_level == ExperienceLevel.BEGINNER:
                code = """// Simple JavaScript example
function greetUser(name) {
    console.log(`Hello, ${name}!`);
    return true;
}

// Call the function
const result = greetUser("World");"""
            else:
                code = """// Advanced JavaScript example with async/await
class APIClient {
    constructor(baseURL) {
        this.baseURL = baseURL;
    }
    
    async fetchData(endpoint) {
        try {
            const response = await fetch(`${this.baseURL}/${endpoint}`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return await response.json();
        } catch (error) {
            console.error('Fetch failed:', error);
            throw error;
        }
    }
}"""
        
        else:
            # Generic example for other languages
            code = f"// Example code in {language}\n// Implementation details would go here"
        
        return {
            "language": language,
            "code": code,
            "explanation": f"This example demonstrates key concepts using {language}",
            "difficulty": experience_level.value,
            "estimated_time": "5-10 minutes" if experience_level == ExperienceLevel.BEGINNER else "2-5 minutes"
        }
    
    def _generate_hardware_example(
        self, 
        content: Dict[str, Any], 
        platform: str, 
        preferences: UserPreferences
    ) -> Optional[Dict[str, Any]]:
        """Generate hardware example for specified platform."""
        experience_level = self._determine_target_experience(preferences)
        
        if platform == "arduino":
            if experience_level == ExperienceLevel.BEGINNER:
                example = {
                    "platform": "Arduino",
                    "title": "LED Blink Example",
                    "description": "Basic LED control using digital output",
                    "code": """// Arduino LED Blink
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}""",
                    "components": ["Arduino board", "LED", "Resistor"],
                    "difficulty": "beginner"
                }
            else:
                example = {
                    "platform": "Arduino",
                    "title": "Sensor Data Logger",
                    "description": "Advanced sensor reading with data logging and communication",
                    "code": """// Advanced Arduino sensor logger
#include <WiFi.h>
#include <ArduinoJson.h>

class SensorLogger {
private:
    float temperature;
    float humidity;
    
public:
    void readSensors() {
        // Read sensor values with error checking
        temperature = analogRead(A0) * 0.48828125;
        humidity = analogRead(A1) * 0.48828125;
    }
    
    void sendData() {
        StaticJsonDocument<200> doc;
        doc["temp"] = temperature;
        doc["humidity"] = humidity;
        doc["timestamp"] = millis();
        
        // Send to server
        String jsonString;
        serializeJson(doc, jsonString);
        // WiFi transmission code here
    }
};""",
                    "components": ["Arduino", "Temperature sensor", "WiFi module", "SD card"],
                    "difficulty": "advanced"
                }
        
        elif platform == "raspberry_pi":
            example = {
                "platform": "Raspberry Pi",
                "title": "GPIO Control with Python",
                "description": "Controlling GPIO pins for hardware interfacing",
                "code": """# Raspberry Pi GPIO control
import RPi.GPIO as GPIO
import time

class GPIOController:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
    
    def blink_led(self, duration=1.0):
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(duration)
    
    def cleanup(self):
        GPIO.cleanup()""",
                "components": ["Raspberry Pi", "LED", "Resistor", "Breadboard"],
                "difficulty": experience_level.value
            }
        
        else:
            # Generic hardware example
            example = {
                "platform": platform.replace("_", " ").title(),
                "title": f"{platform.replace('_', ' ').title()} Example",
                "description": f"Basic example for {platform} platform",
                "code": f"// {platform} implementation\n// Platform-specific code here",
                "components": [f"{platform} board", "Basic components"],
                "difficulty": experience_level.value
            }
        
        return example
    
    def _simplify_content(self, content: Dict[str, Any]) -> Dict[str, Any]:
        """Simplify content for easier understanding."""
        simplified = content.copy()
        
        # Add simplified explanations
        if "simplified_explanation" not in simplified:
            simplified["simplified_explanation"] = "This concept can be broken down into simple steps..."
        
        # Add visual aids suggestion
        simplified["visual_aids"] = ["diagrams", "flowcharts", "step-by-step illustrations"]
        
        # Add learning tips
        simplified["learning_tips"] = [
            "Take your time to understand each concept",
            "Practice with simple examples first",
            "Don't hesitate to review previous sections"
        ]
        
        return simplified
    
    def _expand_content(self, content: Dict[str, Any], preferences: UserPreferences) -> Dict[str, Any]:
        """Expand content with comprehensive details."""
        expanded = content.copy()
        
        # Add advanced topics
        if "advanced_topics" not in expanded:
            expanded["advanced_topics"] = []
        
        expanded["advanced_topics"].extend([
            "Performance considerations",
            "Scalability implications",
            "Security best practices",
            "Industry standards and compliance"
        ])
        
        # Add related concepts
        expanded["related_concepts"] = [
            "Design patterns",
            "Architecture principles",
            "Testing strategies"
        ]
        
        # Add further reading
        expanded["further_reading"] = [
            "Academic papers on the topic",
            "Industry case studies",
            "Open source implementations"
        ]
        
        return expanded
    
    def _estimate_reading_time(self, content: Dict[str, Any], complexity: ContentComplexity) -> str:
        """Estimate reading time based on content and complexity."""
        base_time = 5  # Base reading time in minutes
        
        # Adjust based on complexity
        if complexity == ContentComplexity.SIMPLE:
            multiplier = 0.7
        elif complexity == ContentComplexity.COMPREHENSIVE:
            multiplier = 2.0
        else:
            multiplier = 1.0
        
        # Adjust based on content length (rough estimation)
        content_length = len(str(content))
        if content_length > 5000:
            multiplier *= 1.5
        elif content_length < 1000:
            multiplier *= 0.5
        
        estimated_minutes = int(base_time * multiplier)
        return f"{estimated_minutes}-{estimated_minutes + 5} minutes"
    
    def _get_applied_strategies(self, preferences: UserPreferences) -> List[str]:
        """Get list of adaptation strategies that were applied."""
        strategies = [AdaptationStrategy.EXPERIENCE_BASED.value, AdaptationStrategy.COMPLEXITY_BASED.value]
        
        if preferences.software_background.categories:
            strategies.append(AdaptationStrategy.CATEGORY_BASED.value)
        
        if preferences.software_background.preferred_languages:
            strategies.append(AdaptationStrategy.LANGUAGE_BASED.value)
        
        if preferences.hardware_background.platforms:
            strategies.append(AdaptationStrategy.PLATFORM_BASED.value)
        
        return strategies
    
    def _calculate_personalization_score(self, preferences: UserPreferences) -> float:
        """Calculate a score indicating how personalized the content is."""
        score = 0.0
        
        # Base score for having preferences
        score += 0.2
        
        # Score for software background
        if preferences.software_background.categories:
            score += 0.2
        if preferences.software_background.preferred_languages:
            score += 0.2
        
        # Score for hardware background
        if preferences.hardware_background.categories:
            score += 0.2
        if preferences.hardware_background.platforms:
            score += 0.2
        
        return min(score, 1.0)  # Cap at 1.0
    
    def _initialize_adaptation_rules(self) -> Dict[str, Any]:
        """Initialize adaptation rules for different content types."""
        return {
            "experience_mapping": {
                ExperienceLevel.BEGINNER: {
                    "explanation_style": "detailed",
                    "code_complexity": "simple",
                    "terminology_level": "basic"
                },
                ExperienceLevel.INTERMEDIATE: {
                    "explanation_style": "balanced",
                    "code_complexity": "moderate",
                    "terminology_level": "standard"
                },
                ExperienceLevel.ADVANCED: {
                    "explanation_style": "concise",
                    "code_complexity": "complex",
                    "terminology_level": "technical"
                },
                ExperienceLevel.EXPERT: {
                    "explanation_style": "minimal",
                    "code_complexity": "advanced",
                    "terminology_level": "expert"
                }
            }
        }
    
    def _initialize_complexity_mappings(self) -> Dict[str, Any]:
        """Initialize complexity level mappings."""
        return {
            ContentComplexity.SIMPLE: {
                "detail_level": "basic",
                "example_count": 1,
                "explanation_depth": "surface"
            },
            ContentComplexity.MODERATE: {
                "detail_level": "standard",
                "example_count": 2,
                "explanation_depth": "moderate"
            },
            ContentComplexity.DETAILED: {
                "detail_level": "comprehensive",
                "example_count": 3,
                "explanation_depth": "deep"
            },
            ContentComplexity.COMPREHENSIVE: {
                "detail_level": "exhaustive",
                "example_count": 5,
                "explanation_depth": "complete"
            }
        }
    
    def _initialize_language_preferences(self) -> Dict[str, Any]:
        """Initialize language-specific preferences and mappings."""
        return {
            "python": {
                "style": "pythonic",
                "features": ["list_comprehensions", "context_managers", "decorators"],
                "libraries": ["requests", "pandas", "numpy"]
            },
            "javascript": {
                "style": "modern_es6",
                "features": ["arrow_functions", "async_await", "destructuring"],
                "libraries": ["react", "express", "lodash"]
            },
            "c++": {
                "style": "modern_cpp",
                "features": ["smart_pointers", "lambdas", "auto"],
                "libraries": ["stl", "boost", "eigen"]
            }
        }
    
    def _initialize_platform_mappings(self) -> Dict[str, Any]:
        """Initialize hardware platform mappings."""
        return {
            "arduino": {
                "programming_language": "c++",
                "common_libraries": ["Wire", "SPI", "WiFi"],
                "typical_components": ["sensors", "actuators", "displays"]
            },
            "raspberry_pi": {
                "programming_language": "python",
                "common_libraries": ["RPi.GPIO", "picamera", "sense_hat"],
                "typical_components": ["camera", "gpio_pins", "hat_modules"]
            },
            "esp32": {
                "programming_language": "c++",
                "common_libraries": ["WiFi", "BluetoothSerial", "SPIFFS"],
                "typical_components": ["wifi_module", "bluetooth", "sensors"]
            }
        }


# Global content adaptation engine instance
content_adaptation_engine = ContentAdaptationEngine()