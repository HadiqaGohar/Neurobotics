"""
AI-powered content generation for personalization.
"""

import logging
import json
from typing import Dict, Any, List, Optional
from datetime import datetime

from .models import UserPreferences, ExperienceLevel

logger = logging.getLogger(__name__)


class AIContentGenerator:
    """AI service integration for generating personalized content."""
    
    def __init__(self):
        self.generation_stats = {
            "total_requests": 0,
            "successful_generations": 0,
            "failed_generations": 0,
            "average_generation_time_ms": 0.0
        }
    
    async def generate_personalized_content(
        self,
        base_content: Dict[str, Any],
        user_preferences: UserPreferences,
        content_type: str = "text"
    ) -> Dict[str, Any]:
        """Generate AI-powered personalized content."""
        try:
            self.generation_stats["total_requests"] += 1
            
            # Create personalization prompt
            prompt = self._create_personalization_prompt(base_content, user_preferences)
            
            # Generate content (placeholder implementation)
            generated_content = await self._call_ai_service(prompt, user_preferences)
            
            # Post-process and validate
            processed_content = self._post_process_content(generated_content, user_preferences)
            
            self.generation_stats["successful_generations"] += 1
            return processed_content
            
        except Exception as e:
            logger.error(f"Error generating AI content: {e}")
            self.generation_stats["failed_generations"] += 1
            return self._get_fallback_content(base_content)
    
    def _create_personalization_prompt(
        self,
        base_content: Dict[str, Any],
        user_preferences: UserPreferences
    ) -> str:
        """Create AI prompt for content personalization."""
        
        experience_level = max(
            user_preferences.software_background.experience_level,
            user_preferences.hardware_background.experience_level,
            key=lambda x: ["beginner", "intermediate", "advanced", "expert"].index(x.value)
        )
        
        prompt = f"""
        Personalize the following content for a {experience_level.value} level user with these preferences:
        
        Software Background:
        - Categories: {[cat.value for cat in user_preferences.software_background.categories]}
        - Languages: {user_preferences.software_background.preferred_languages}
        - Experience: {user_preferences.software_background.experience_level.value}
        
        Hardware Background:
        - Categories: {[cat.value for cat in user_preferences.hardware_background.categories]}
        - Platforms: {user_preferences.hardware_background.platforms}
        - Experience: {user_preferences.hardware_background.experience_level.value}
        
        Content Preferences:
        - Complexity: {user_preferences.content_complexity.value}
        - Explanation Depth: {user_preferences.explanation_depth}
        - Example Style: {user_preferences.example_style}
        
        Original Content:
        {json.dumps(base_content, indent=2)}
        
        Please adapt this content to match the user's background and preferences. Include:
        1. Appropriate technical depth
        2. Relevant code examples in preferred languages
        3. Hardware examples for preferred platforms
        4. Explanations at the right complexity level
        
        Return the personalized content in JSON format.
        """
        
        return prompt
    
    async def _call_ai_service(
        self,
        prompt: str,
        user_preferences: UserPreferences
    ) -> Dict[str, Any]:
        """Call AI service (placeholder implementation)."""
        
        # This is a placeholder implementation
        # In production, this would call Gemini API or similar service
        
        experience_level = max(
            user_preferences.software_background.experience_level,
            user_preferences.hardware_background.experience_level,
            key=lambda x: ["beginner", "intermediate", "advanced", "expert"].index(x.value)
        )
        
        # Generate mock personalized content based on preferences
        generated_content = {
            "title": f"Personalized Content for {experience_level.value.title()} Level",
            "text": f"This content has been adapted for {experience_level.value} level users with expertise in {', '.join([cat.value for cat in user_preferences.software_background.categories[:3]])}.",
            "code_examples": self._generate_code_examples(user_preferences),
            "hardware_examples": self._generate_hardware_examples(user_preferences),
            "explanations": self._generate_explanations(user_preferences),
            "complexity_indicators": {
                "level": experience_level.value,
                "estimated_time": "10-15 minutes" if experience_level.value == "beginner" else "5-10 minutes"
            },
            "ai_generated": True,
            "generation_timestamp": datetime.utcnow().isoformat()
        }
        
        return generated_content
    
    def _generate_code_examples(self, user_preferences: UserPreferences) -> List[Dict[str, Any]]:
        """Generate code examples based on user preferences."""
        examples = []
        
        preferred_languages = user_preferences.software_background.preferred_languages[:3]
        if not preferred_languages:
            preferred_languages = ["python"]
        
        for lang in preferred_languages:
            example = {
                "language": lang,
                "title": f"{lang.title()} Implementation",
                "code": f"# AI-generated {lang} example\n# Tailored for {user_preferences.software_background.experience_level.value} level\n\ndef example_function():\n    return 'Personalized content'",
                "explanation": f"This {lang} example demonstrates the concept at {user_preferences.software_background.experience_level.value} level."
            }
            examples.append(example)
        
        return examples
    
    def _generate_hardware_examples(self, user_preferences: UserPreferences) -> List[Dict[str, Any]]:
        """Generate hardware examples based on user preferences."""
        examples = []
        
        preferred_platforms = user_preferences.hardware_background.platforms[:2]
        if not preferred_platforms:
            preferred_platforms = ["arduino"]
        
        for platform in preferred_platforms:
            example = {
                "platform": platform.title(),
                "title": f"{platform.title()} Implementation",
                "description": f"AI-generated example for {platform} platform",
                "code": f"// AI-generated {platform} code\n// Adapted for {user_preferences.hardware_background.experience_level.value} level\nvoid setup() {\n  // Initialization\n}\n\nvoid loop() {\n  // Main logic\n}",
                "difficulty": user_preferences.hardware_background.experience_level.value
            }
            examples.append(example)
        
        return examples
    
    def _generate_explanations(self, user_preferences: UserPreferences) -> Dict[str, str]:
        """Generate explanations based on user preferences."""
        
        experience_level = max(
            user_preferences.software_background.experience_level,
            user_preferences.hardware_background.experience_level,
            key=lambda x: ["beginner", "intermediate", "advanced", "expert"].index(x.value)
        )
        
        explanations = {
            "main": f"AI-generated explanation tailored for {experience_level.value} level users.",
            "technical_depth": user_preferences.explanation_depth,
            "complexity": user_preferences.content_complexity.value
        }
        
        if experience_level.value == "beginner":
            explanations["beginner_note"] = "This explanation starts with the basics and builds up gradually."
        elif experience_level.value == "expert":
            explanations["expert_note"] = "This explanation assumes familiarity with advanced concepts."
        
        return explanations
    
    def _post_process_content(
        self,
        generated_content: Dict[str, Any],
        user_preferences: UserPreferences
    ) -> Dict[str, Any]:
        """Post-process and validate AI-generated content."""
        
        # Add metadata
        generated_content["ai_metadata"] = {
            "generated_at": datetime.utcnow().isoformat(),
            "user_preferences_applied": {
                "software_experience": user_preferences.software_background.experience_level.value,
                "hardware_experience": user_preferences.hardware_background.experience_level.value,
                "content_complexity": user_preferences.content_complexity.value,
                "explanation_depth": user_preferences.explanation_depth
            },
            "quality_score": self._calculate_quality_score(generated_content),
            "personalization_confidence": 0.85  # Mock confidence score
        }
        
        return generated_content
    
    def _calculate_quality_score(self, content: Dict[str, Any]) -> float:
        """Calculate quality score for generated content."""
        score = 0.5  # Base score
        
        # Check content completeness
        if "text" in content and len(content["text"]) > 50:
            score += 0.1
        
        if "code_examples" in content and content["code_examples"]:
            score += 0.15
        
        if "explanations" in content and content["explanations"]:
            score += 0.1
        
        if "hardware_examples" in content and content["hardware_examples"]:
            score += 0.1
        
        # Check for AI-specific quality indicators
        if content.get("ai_generated"):
            score += 0.05
        
        return min(score, 1.0)
    
    def _get_fallback_content(self, base_content: Dict[str, Any]) -> Dict[str, Any]:
        """Get fallback content when AI generation fails."""
        fallback = base_content.copy()
        fallback.update({
            "ai_generation_failed": True,
            "fallback_message": "AI content generation unavailable, showing default content.",
            "generation_timestamp": datetime.utcnow().isoformat()
        })
        return fallback
    
    def get_generation_stats(self) -> Dict[str, Any]:
        """Get AI content generation statistics."""
        return self.generation_stats.copy()


# Global AI content generator instance
ai_content_generator = AIContentGenerator()