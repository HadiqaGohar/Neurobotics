"""
Personalization rules engine for content adaptation logic.
"""

import logging
from typing import Dict, Any, List, Optional, Tuple, Callable
from enum import Enum
from dataclasses import dataclass
from datetime import datetime

from .models import UserPreferences, ExperienceLevel, SoftwareCategory, HardwareCategory, ContentComplexity
from .content_tagger import ContentTag

logger = logging.getLogger(__name__)


class RuleType(str, Enum):
    """Types of personalization rules."""
    EXPERIENCE_MATCHING = "experience_matching"
    TECHNOLOGY_MATCHING = "technology_matching"
    COMPLEXITY_ADAPTATION = "complexity_adaptation"
    LANGUAGE_PREFERENCE = "language_preference"
    PLATFORM_PREFERENCE = "platform_preference"
    FALLBACK_SELECTION = "fallback_selection"


class RulePriority(str, Enum):
    """Rule execution priorities."""
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


@dataclass
class PersonalizationRule:
    """A personalization rule definition."""
    rule_id: str
    name: str
    description: str
    rule_type: RuleType
    priority: RulePriority
    condition: Callable[[Dict[str, Any], Dict[str, Any]], bool]
    action: Callable[[Dict[str, Any], Dict[str, Any]], Dict[str, Any]]
    weight: float = 1.0
    enabled: bool = True


@dataclass
class RuleExecutionResult:
    """Result of rule execution."""
    rule_id: str
    executed: bool
    score: float
    modifications: Dict[str, Any]
    execution_time_ms: float


class PersonalizationRulesEngine:
    """Engine for executing personalization rules and logic."""
    
    def __init__(self):
        self.rules: Dict[str, PersonalizationRule] = {}
        self.rule_execution_stats = {}
        self._initialize_default_rules()
    
    def execute_rules(
        self,
        content: Dict[str, Any],
        user_preferences: UserPreferences,
        context: Optional[Dict[str, Any]] = None
    ) -> Tuple[Dict[str, Any], List[RuleExecutionResult]]:
        """
        Execute personalization rules on content.
        
        Args:
            content: Content to personalize
            user_preferences: User's preferences
            context: Additional context for rule execution
            
        Returns:
            Tuple of (personalized_content, execution_results)
        """
        try:
            if not context:
                context = {}
            
            # Prepare rule context
            rule_context = {
                "user_preferences": user_preferences.dict(),
                "content": content.copy(),
                "context": context,
                "execution_timestamp": datetime.utcnow().isoformat()
            }
            
            personalized_content = content.copy()
            execution_results = []
            
            # Sort rules by priority and weight
            sorted_rules = sorted(
                [rule for rule in self.rules.values() if rule.enabled],
                key=lambda r: (self._priority_to_int(r.priority), -r.weight)
            )
            
            # Execute rules in order
            for rule in sorted_rules:
                start_time = datetime.utcnow()
                
                try:
                    # Check rule condition
                    if rule.condition(personalized_content, rule_context):
                        # Execute rule action
                        modifications = rule.action(personalized_content, rule_context)
                        
                        # Apply modifications
                        personalized_content.update(modifications)
                        
                        # Calculate execution score
                        score = self._calculate_rule_score(rule, modifications, rule_context)
                        
                        execution_time = (datetime.utcnow() - start_time).total_seconds() * 1000
                        
                        execution_results.append(RuleExecutionResult(
                            rule_id=rule.rule_id,
                            executed=True,
                            score=score,
                            modifications=modifications,
                            execution_time_ms=execution_time
                        ))
                        
                        # Update rule stats
                        self._update_rule_stats(rule.rule_id, True, execution_time)
                        
                    else:
                        # Rule condition not met
                        execution_results.append(RuleExecutionResult(
                            rule_id=rule.rule_id,
                            executed=False,
                            score=0.0,
                            modifications={},
                            execution_time_ms=0.0
                        ))
                        
                except Exception as e:
                    logger.error(f"Error executing rule {rule.rule_id}: {e}")
                    execution_results.append(RuleExecutionResult(
                        rule_id=rule.rule_id,
                        executed=False,
                        score=0.0,
                        modifications={},
                        execution_time_ms=0.0
                    ))
            
            # Add rule execution metadata
            personalized_content["rule_execution_metadata"] = {
                "executed_rules": [r.rule_id for r in execution_results if r.executed],
                "total_score": sum(r.score for r in execution_results),
                "execution_count": len([r for r in execution_results if r.executed]),
                "total_execution_time_ms": sum(r.execution_time_ms for r in execution_results)
            }
            
            return personalized_content, execution_results
            
        except Exception as e:
            logger.error(f"Error executing personalization rules: {e}")
            return content, []
    
    def add_rule(self, rule: PersonalizationRule) -> bool:
        """Add a new personalization rule."""
        try:
            self.rules[rule.rule_id] = rule
            self.rule_execution_stats[rule.rule_id] = {
                "executions": 0,
                "successes": 0,
                "total_time_ms": 0.0,
                "average_time_ms": 0.0
            }
            logger.info(f"Added personalization rule: {rule.rule_id}")
            return True
        except Exception as e:
            logger.error(f"Error adding rule {rule.rule_id}: {e}")
            return False
    
    def remove_rule(self, rule_id: str) -> bool:
        """Remove a personalization rule."""
        try:
            if rule_id in self.rules:
                del self.rules[rule_id]
                if rule_id in self.rule_execution_stats:
                    del self.rule_execution_stats[rule_id]
                logger.info(f"Removed personalization rule: {rule_id}")
                return True
            return False
        except Exception as e:
            logger.error(f"Error removing rule {rule_id}: {e}")
            return False
    
    def enable_rule(self, rule_id: str) -> bool:
        """Enable a personalization rule."""
        if rule_id in self.rules:
            self.rules[rule_id].enabled = True
            return True
        return False
    
    def disable_rule(self, rule_id: str) -> bool:
        """Disable a personalization rule."""
        if rule_id in self.rules:
            self.rules[rule_id].enabled = False
            return True
        return False
    
    def get_rule_stats(self) -> Dict[str, Any]:
        """Get execution statistics for all rules."""
        return {
            "total_rules": len(self.rules),
            "enabled_rules": len([r for r in self.rules.values() if r.enabled]),
            "rule_stats": self.rule_execution_stats.copy()
        }
    
    def _initialize_default_rules(self):
        """Initialize default personalization rules."""
        
        # Experience Level Matching Rules
        self.add_rule(PersonalizationRule(
            rule_id="exp_beginner_simplification",
            name="Beginner Content Simplification",
            description="Simplify content for beginner users",
            rule_type=RuleType.EXPERIENCE_MATCHING,
            priority=RulePriority.HIGH,
            condition=self._is_beginner_user,
            action=self._simplify_for_beginner,
            weight=0.9
        ))
        
        self.add_rule(PersonalizationRule(
            rule_id="exp_expert_enhancement",
            name="Expert Content Enhancement",
            description="Add advanced details for expert users",
            rule_type=RuleType.EXPERIENCE_MATCHING,
            priority=RulePriority.HIGH,
            condition=self._is_expert_user,
            action=self._enhance_for_expert,
            weight=0.9
        ))
        
        # Technology Matching Rules
        self.add_rule(PersonalizationRule(
            rule_id="tech_web_dev_examples",
            name="Web Development Examples",
            description="Add web development specific examples",
            rule_type=RuleType.TECHNOLOGY_MATCHING,
            priority=RulePriority.MEDIUM,
            condition=self._is_web_developer,
            action=self._add_web_dev_examples,
            weight=0.7
        ))
        
        self.add_rule(PersonalizationRule(
            rule_id="tech_embedded_examples",
            name="Embedded Systems Examples",
            description="Add embedded systems specific examples",
            rule_type=RuleType.TECHNOLOGY_MATCHING,
            priority=RulePriority.MEDIUM,
            condition=self._is_embedded_developer,
            action=self._add_embedded_examples,
            weight=0.7
        ))
        
        # Language Preference Rules
        self.add_rule(PersonalizationRule(
            rule_id="lang_python_preference",
            name="Python Code Examples",
            description="Prioritize Python code examples",
            rule_type=RuleType.LANGUAGE_PREFERENCE,
            priority=RulePriority.MEDIUM,
            condition=self._prefers_python,
            action=self._add_python_examples,
            weight=0.6
        ))
        
        self.add_rule(PersonalizationRule(
            rule_id="lang_javascript_preference",
            name="JavaScript Code Examples",
            description="Prioritize JavaScript code examples",
            rule_type=RuleType.LANGUAGE_PREFERENCE,
            priority=RulePriority.MEDIUM,
            condition=self._prefers_javascript,
            action=self._add_javascript_examples,
            weight=0.6
        ))
        
        # Platform Preference Rules
        self.add_rule(PersonalizationRule(
            rule_id="platform_arduino_examples",
            name="Arduino Platform Examples",
            description="Add Arduino-specific examples",
            rule_type=RuleType.PLATFORM_PREFERENCE,
            priority=RulePriority.MEDIUM,
            condition=self._uses_arduino,
            action=self._add_arduino_examples,
            weight=0.6
        ))
        
        self.add_rule(PersonalizationRule(
            rule_id="platform_raspberry_pi_examples",
            name="Raspberry Pi Examples",
            description="Add Raspberry Pi specific examples",
            rule_type=RuleType.PLATFORM_PREFERENCE,
            priority=RulePriority.MEDIUM,
            condition=self._uses_raspberry_pi,
            action=self._add_raspberry_pi_examples,
            weight=0.6
        ))
        
        # Complexity Adaptation Rules
        self.add_rule(PersonalizationRule(
            rule_id="complexity_simple_content",
            name="Simple Content Adaptation",
            description="Adapt content for simple complexity preference",
            rule_type=RuleType.COMPLEXITY_ADAPTATION,
            priority=RulePriority.HIGH,
            condition=self._prefers_simple_content,
            action=self._adapt_to_simple,
            weight=0.8
        ))
        
        self.add_rule(PersonalizationRule(
            rule_id="complexity_comprehensive_content",
            name="Comprehensive Content Adaptation",
            description="Adapt content for comprehensive complexity preference",
            rule_type=RuleType.COMPLEXITY_ADAPTATION,
            priority=RulePriority.HIGH,
            condition=self._prefers_comprehensive_content,
            action=self._adapt_to_comprehensive,
            weight=0.8
        ))
        
        # Fallback Rules
        self.add_rule(PersonalizationRule(
            rule_id="fallback_default_enhancement",
            name="Default Content Enhancement",
            description="Apply default enhancements when no specific rules match",
            rule_type=RuleType.FALLBACK_SELECTION,
            priority=RulePriority.LOW,
            condition=self._always_true,
            action=self._apply_default_enhancements,
            weight=0.3
        ))
    
    # Rule Conditions
    def _is_beginner_user(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user is a beginner."""
        prefs = context["user_preferences"]
        software_exp = prefs.get("software_background", {}).get("experience_level", "intermediate")
        hardware_exp = prefs.get("hardware_background", {}).get("experience_level", "intermediate")
        return software_exp == "beginner" or hardware_exp == "beginner"
    
    def _is_expert_user(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user is an expert."""
        prefs = context["user_preferences"]
        software_exp = prefs.get("software_background", {}).get("experience_level", "intermediate")
        hardware_exp = prefs.get("hardware_background", {}).get("experience_level", "intermediate")
        return software_exp == "expert" or hardware_exp == "expert"
    
    def _is_web_developer(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user is a web developer."""
        prefs = context["user_preferences"]
        categories = prefs.get("software_background", {}).get("categories", [])
        return "web_development" in categories
    
    def _is_embedded_developer(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user works with embedded systems."""
        prefs = context["user_preferences"]
        software_cats = prefs.get("software_background", {}).get("categories", [])
        hardware_cats = prefs.get("hardware_background", {}).get("categories", [])
        return "embedded_systems" in software_cats or "embedded_systems" in hardware_cats
    
    def _prefers_python(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user prefers Python."""
        prefs = context["user_preferences"]
        languages = prefs.get("software_background", {}).get("preferred_languages", [])
        return "python" in [lang.lower() for lang in languages]
    
    def _prefers_javascript(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user prefers JavaScript."""
        prefs = context["user_preferences"]
        languages = prefs.get("software_background", {}).get("preferred_languages", [])
        return any(lang.lower() in ["javascript", "typescript"] for lang in languages)
    
    def _uses_arduino(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user uses Arduino."""
        prefs = context["user_preferences"]
        platforms = prefs.get("hardware_background", {}).get("platforms", [])
        return "arduino" in [platform.lower() for platform in platforms]
    
    def _uses_raspberry_pi(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user uses Raspberry Pi."""
        prefs = context["user_preferences"]
        platforms = prefs.get("hardware_background", {}).get("platforms", [])
        return "raspberry_pi" in [platform.lower().replace(" ", "_") for platform in platforms]
    
    def _prefers_simple_content(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user prefers simple content."""
        prefs = context["user_preferences"]
        return prefs.get("content_complexity", "moderate") == "simple"
    
    def _prefers_comprehensive_content(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if user prefers comprehensive content."""
        prefs = context["user_preferences"]
        return prefs.get("content_complexity", "moderate") == "comprehensive"
    
    def _always_true(self, content: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Always return true (for fallback rules)."""
        return True
    
    # Rule Actions
    def _simplify_for_beginner(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Simplify content for beginner users."""
        modifications = {}
        
        # Add beginner-friendly explanations
        if "explanations" not in content:
            content["explanations"] = {}
        
        modifications["explanations"] = content["explanations"].copy()
        modifications["explanations"]["beginner_note"] = "This concept is explained step-by-step for beginners."
        modifications["explanations"]["prerequisites"] = "Basic understanding of programming concepts is helpful."
        
        # Add glossary for technical terms
        modifications["glossary"] = {
            "API": "Application Programming Interface - a way for different programs to communicate",
            "Framework": "A pre-built structure that provides common functionality",
            "Library": "A collection of pre-written code that you can use"
        }
        
        # Add learning tips
        modifications["learning_tips"] = [
            "Take your time to understand each concept",
            "Practice with simple examples first",
            "Don't hesitate to review previous sections"
        ]
        
        return modifications
    
    def _enhance_for_expert(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Enhance content for expert users."""
        modifications = {}
        
        # Add advanced topics
        modifications["advanced_topics"] = [
            "Performance optimization techniques",
            "Scalability considerations",
            "Security best practices",
            "Industry standards and compliance"
        ]
        
        # Add architectural considerations
        modifications["architecture_notes"] = {
            "design_patterns": "Consider applying relevant design patterns",
            "performance": "Evaluate performance implications of implementation choices",
            "maintainability": "Focus on code maintainability and extensibility"
        }
        
        # Add references to advanced resources
        modifications["advanced_resources"] = [
            "Academic papers on the topic",
            "Industry case studies",
            "Open source implementations"
        ]
        
        return modifications
    
    def _add_web_dev_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add web development specific examples."""
        modifications = {}
        
        if "examples" not in content:
            content["examples"] = []
        
        modifications["examples"] = content["examples"].copy()
        modifications["examples"].append({
            "title": "Web Application Example",
            "description": "Building a responsive web interface with modern frameworks",
            "technologies": ["HTML", "CSS", "JavaScript", "React", "Node.js"],
            "use_case": "Creating interactive user interfaces for web applications"
        })
        
        return modifications
    
    def _add_embedded_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add embedded systems specific examples."""
        modifications = {}
        
        if "examples" not in content:
            content["examples"] = []
        
        modifications["examples"] = content["examples"].copy()
        modifications["examples"].append({
            "title": "Embedded System Example",
            "description": "Real-time sensor data processing on microcontrollers",
            "technologies": ["C", "C++", "RTOS", "Hardware Abstraction Layer"],
            "use_case": "Implementing real-time control systems for IoT devices"
        })
        
        return modifications
    
    def _add_python_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add Python code examples."""
        modifications = {}
        
        if "code_examples" not in content:
            content["code_examples"] = []
        
        modifications["code_examples"] = content["code_examples"].copy()
        modifications["code_examples"].append({
            "language": "python",
            "title": "Python Implementation",
            "code": """# Python example demonstrating the concept
def process_data(data):
    \"\"\"Process data with error handling.\"\"\"
    try:
        result = {"processed": len(data), "status": "success"}
        return result
    except Exception as e:
        print(f"Error processing data: {e}")
        return {"status": "error"}

# Usage example
data = [1, 2, 3, 4, 5]
result = process_data(data)
print(result)""",
            "explanation": "This Python example shows clean, readable code with proper error handling."
        })
        
        return modifications
    
    def _add_javascript_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add JavaScript code examples."""
        modifications = {}
        
        if "code_examples" not in content:
            content["code_examples"] = []
        
        modifications["code_examples"] = content["code_examples"].copy()
        modifications["code_examples"].append({
            "language": "javascript",
            "title": "JavaScript Implementation",
            "code": """// JavaScript example with modern ES6+ features
const processData = async (data) => {
    try {
        const result = {
            processed: data.length,
            status: 'success',
            timestamp: new Date().toISOString()
        };
        return result;
    } catch (error) {
        console.error('Error processing data:', error);
        return { status: 'error', message: error.message };
    }
};

// Usage with async/await
const data = [1, 2, 3, 4, 5];
processData(data).then(result => console.log(result));""",
            "explanation": "This JavaScript example uses modern async/await syntax and ES6 features."
        })
        
        return modifications
    
    def _add_arduino_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add Arduino platform examples."""
        modifications = {}
        
        if "hardware_examples" not in content:
            content["hardware_examples"] = []
        
        modifications["hardware_examples"] = content["hardware_examples"].copy()
        modifications["hardware_examples"].append({
            "platform": "Arduino",
            "title": "Arduino Implementation",
            "description": "Sensor reading and data processing on Arduino",
            "code": """// Arduino code for sensor data processing
#include <Arduino.h>

const int sensorPin = A0;
const int ledPin = 13;

void setup() {
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
    pinMode(sensorPin, INPUT);
}

void loop() {
    int sensorValue = analogRead(sensorPin);
    float voltage = sensorValue * (5.0 / 1023.0);
    
    // Process sensor data
    if (voltage > 2.5) {
        digitalWrite(ledPin, HIGH);
        Serial.println("Sensor threshold exceeded");
    } else {
        digitalWrite(ledPin, LOW);
    }
    
    Serial.print("Sensor voltage: ");
    Serial.println(voltage);
    
    delay(1000);
}""",
            "components": ["Arduino Uno", "Analog sensor", "LED", "Resistors"],
            "difficulty": "intermediate"
        })
        
        return modifications
    
    def _add_raspberry_pi_examples(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Add Raspberry Pi platform examples."""
        modifications = {}
        
        if "hardware_examples" not in content:
            content["hardware_examples"] = []
        
        modifications["hardware_examples"] = content["hardware_examples"].copy()
        modifications["hardware_examples"].append({
            "platform": "Raspberry Pi",
            "title": "Raspberry Pi Implementation",
            "description": "GPIO control and sensor interfacing with Python",
            "code": """# Raspberry Pi GPIO control with Python
import RPi.GPIO as GPIO
import time
import json
from datetime import datetime

class SensorController:
    def __init__(self, sensor_pin, led_pin):
        self.sensor_pin = sensor_pin
        self.led_pin = led_pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.sensor_pin, GPIO.IN)
    
    def read_sensor(self):
        \"\"\"Read sensor data and control LED.\"\"\"
        sensor_state = GPIO.input(self.sensor_pin)
        
        data = {
            "timestamp": datetime.now().isoformat(),
            "sensor_state": sensor_state,
            "led_activated": False
        }
        
        if sensor_state:
            GPIO.output(self.led_pin, GPIO.HIGH)
            data["led_activated"] = True
        else:
            GPIO.output(self.led_pin, GPIO.LOW)
        
        return data
    
    def cleanup(self):
        GPIO.cleanup()

# Usage
controller = SensorController(18, 24)
try:
    while True:
        data = controller.read_sensor()
        print(json.dumps(data, indent=2))
        time.sleep(1)
except KeyboardInterrupt:
    controller.cleanup()""",
            "components": ["Raspberry Pi", "GPIO pins", "Sensor", "LED"],
            "difficulty": "intermediate"
        })
        
        return modifications
    
    def _adapt_to_simple(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Adapt content for simple complexity preference."""
        modifications = {}
        
        # Add simplified summary
        modifications["simple_summary"] = "This section covers the essential concepts in an easy-to-understand way."
        
        # Add step-by-step breakdown
        modifications["step_by_step"] = [
            "Start with the basic concept",
            "Understand the key components",
            "See how they work together",
            "Practice with simple examples"
        ]
        
        # Add visual learning aids
        modifications["visual_aids"] = {
            "diagrams": "Simple diagrams to illustrate concepts",
            "flowcharts": "Step-by-step process flows",
            "infographics": "Visual summaries of key points"
        }
        
        return modifications
    
    def _adapt_to_comprehensive(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Adapt content for comprehensive complexity preference."""
        modifications = {}
        
        # Add detailed analysis
        modifications["detailed_analysis"] = {
            "technical_depth": "In-depth technical explanation with implementation details",
            "edge_cases": "Consideration of edge cases and error scenarios",
            "performance": "Performance implications and optimization strategies",
            "alternatives": "Alternative approaches and trade-offs"
        }
        
        # Add comprehensive examples
        modifications["comprehensive_examples"] = [
            "Complete implementation examples",
            "Real-world use cases and scenarios",
            "Integration with other systems",
            "Testing and validation approaches"
        ]
        
        # Add research references
        modifications["research_references"] = [
            "Academic papers and research",
            "Industry standards and specifications",
            "Open source projects and implementations",
            "Community discussions and best practices"
        ]
        
        return modifications
    
    def _apply_default_enhancements(self, content: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """Apply default enhancements to content."""
        modifications = {}
        
        # Add general learning objectives if not present
        if "learning_objectives" not in content:
            modifications["learning_objectives"] = [
                "Understand the core concepts",
                "Apply knowledge in practical scenarios",
                "Integrate with existing knowledge base"
            ]
        
        # Add estimated reading time
        content_length = len(str(content))
        reading_time = max(2, content_length // 1000)  # Rough estimate
        modifications["estimated_reading_time"] = f"{reading_time}-{reading_time + 3} minutes"
        
        # Add next steps
        modifications["next_steps"] = [
            "Review the key concepts",
            "Try the provided examples",
            "Explore related topics"
        ]
        
        return modifications
    
    # Helper methods
    def _priority_to_int(self, priority: RulePriority) -> int:
        """Convert priority to integer for sorting."""
        mapping = {
            RulePriority.CRITICAL: 1,
            RulePriority.HIGH: 2,
            RulePriority.MEDIUM: 3,
            RulePriority.LOW: 4
        }
        return mapping.get(priority, 3)
    
    def _calculate_rule_score(
        self,
        rule: PersonalizationRule,
        modifications: Dict[str, Any],
        context: Dict[str, Any]
    ) -> float:
        """Calculate execution score for a rule."""
        base_score = rule.weight
        
        # Bonus for number of modifications
        modification_bonus = min(len(modifications) * 0.1, 0.5)
        
        # Bonus for rule priority
        priority_bonus = {
            RulePriority.CRITICAL: 0.3,
            RulePriority.HIGH: 0.2,
            RulePriority.MEDIUM: 0.1,
            RulePriority.LOW: 0.05
        }.get(rule.priority, 0.1)
        
        return base_score + modification_bonus + priority_bonus
    
    def _update_rule_stats(self, rule_id: str, success: bool, execution_time_ms: float):
        """Update execution statistics for a rule."""
        if rule_id not in self.rule_execution_stats:
            self.rule_execution_stats[rule_id] = {
                "executions": 0,
                "successes": 0,
                "total_time_ms": 0.0,
                "average_time_ms": 0.0
            }
        
        stats = self.rule_execution_stats[rule_id]
        stats["executions"] += 1
        
        if success:
            stats["successes"] += 1
        
        stats["total_time_ms"] += execution_time_ms
        stats["average_time_ms"] = stats["total_time_ms"] / stats["executions"]


# Global rules engine instance
rules_engine = PersonalizationRulesEngine()