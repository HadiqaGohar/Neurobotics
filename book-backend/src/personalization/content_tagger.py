"""
Content tagging and categorization system for personalization.
"""

import logging
from typing import Dict, Any, List, Set, Optional
from enum import Enum
from dataclasses import dataclass

from .models import ExperienceLevel, SoftwareCategory, HardwareCategory

logger = logging.getLogger(__name__)


class ContentTag(str, Enum):
    """Content tags for categorization."""
    # Experience level tags
    BEGINNER_FRIENDLY = "beginner_friendly"
    INTERMEDIATE_LEVEL = "intermediate_level"
    ADVANCED_CONCEPTS = "advanced_concepts"
    EXPERT_ONLY = "expert_only"
    
    # Content type tags
    THEORETICAL = "theoretical"
    PRACTICAL = "practical"
    HANDS_ON = "hands_on"
    CONCEPTUAL = "conceptual"
    
    # Technology tags
    SOFTWARE_FOCUSED = "software_focused"
    HARDWARE_FOCUSED = "hardware_focused"
    FULL_STACK = "full_stack"
    EMBEDDED_SYSTEMS = "embedded_systems"
    
    # Learning style tags
    VISUAL_LEARNER = "visual_learner"
    CODE_HEAVY = "code_heavy"
    DIAGRAM_RICH = "diagram_rich"
    STEP_BY_STEP = "step_by_step"
    
    # Complexity tags
    SIMPLE_EXPLANATION = "simple_explanation"
    DETAILED_ANALYSIS = "detailed_analysis"
    COMPREHENSIVE_COVERAGE = "comprehensive_coverage"
    QUICK_REFERENCE = "quick_reference"


@dataclass
class ContentMetadata:
    """Metadata for content pieces."""
    content_id: str
    title: str
    tags: Set[ContentTag]
    experience_level: ExperienceLevel
    estimated_time_minutes: int
    prerequisites: List[str]
    learning_objectives: List[str]
    software_categories: List[SoftwareCategory]
    hardware_categories: List[HardwareCategory]
    programming_languages: List[str]
    hardware_platforms: List[str]


class ContentTagger:
    """System for tagging and categorizing content for personalization."""
    
    def __init__(self):
        self.tag_rules = self._initialize_tag_rules()
        self.category_mappings = self._initialize_category_mappings()
    
    def tag_content(self, content: Dict[str, Any], metadata: Optional[ContentMetadata] = None) -> Set[ContentTag]:
        """
        Automatically tag content based on its characteristics.
        
        Args:
            content: The content to analyze and tag
            metadata: Optional existing metadata
            
        Returns:
            Set of content tags
        """
        try:
            tags = set()
            
            # Analyze content text for automatic tagging
            if "text" in content:
                tags.update(self._analyze_text_content(content["text"]))
            
            # Tag based on code examples
            if "code_examples" in content:
                tags.update(self._analyze_code_examples(content["code_examples"]))
            
            # Tag based on hardware references
            if "hardware_examples" in content:
                tags.update(self._analyze_hardware_content(content["hardware_examples"]))
            
            # Tag based on complexity
            if "complexity_level" in content:
                tags.update(self._get_complexity_tags(content["complexity_level"]))
            
            # Tag based on existing metadata
            if metadata:
                tags.update(self._get_metadata_tags(metadata))
            
            # Apply tag rules for consistency
            tags = self._apply_tag_rules(tags)
            
            return tags
            
        except Exception as e:
            logger.error(f"Error tagging content: {e}")
            return set()
    
    def match_content_to_preferences(
        self, 
        content_tags: Set[ContentTag], 
        user_preferences: Dict[str, Any]
    ) -> float:
        """
        Calculate how well content matches user preferences.
        
        Args:
            content_tags: Tags associated with the content
            user_preferences: User's personalization preferences
            
        Returns:
            Match score between 0.0 and 1.0
        """
        try:
            score = 0.0
            total_factors = 0
            
            # Experience level matching
            user_experience = user_preferences.get("experience_level", "intermediate")
            experience_score = self._calculate_experience_match(content_tags, user_experience)
            score += experience_score
            total_factors += 1
            
            # Content type preference matching
            content_type_score = self._calculate_content_type_match(
                content_tags, 
                user_preferences.get("example_style", "practical")
            )
            score += content_type_score
            total_factors += 1
            
            # Technology focus matching
            tech_score = self._calculate_technology_match(
                content_tags,
                user_preferences.get("software_categories", []),
                user_preferences.get("hardware_categories", [])
            )
            score += tech_score
            total_factors += 1
            
            # Complexity preference matching
            complexity_score = self._calculate_complexity_match(
                content_tags,
                user_preferences.get("content_complexity", "moderate")
            )
            score += complexity_score
            total_factors += 1
            
            return score / total_factors if total_factors > 0 else 0.0
            
        except Exception as e:
            logger.error(f"Error matching content to preferences: {e}")
            return 0.0
    
    def suggest_content_improvements(
        self, 
        content: Dict[str, Any], 
        target_preferences: Dict[str, Any]
    ) -> List[Dict[str, str]]:
        """
        Suggest improvements to make content better match target preferences.
        
        Args:
            content: The content to analyze
            target_preferences: Target user preferences
            
        Returns:
            List of improvement suggestions
        """
        try:
            suggestions = []
            current_tags = self.tag_content(content)
            
            # Check experience level alignment
            target_experience = target_preferences.get("experience_level", "intermediate")
            if not self._has_experience_level_tag(current_tags, target_experience):
                suggestions.append({
                    "type": "experience_level",
                    "suggestion": f"Add content suitable for {target_experience} level users",
                    "priority": "high"
                })
            
            # Check for code examples in preferred languages
            preferred_languages = target_preferences.get("preferred_languages", [])
            if preferred_languages and ContentTag.CODE_HEAVY in current_tags:
                if "code_examples" not in content or not content["code_examples"]:
                    suggestions.append({
                        "type": "code_examples",
                        "suggestion": f"Add code examples in {', '.join(preferred_languages[:3])}",
                        "priority": "medium"
                    })
            
            # Check for hardware platform examples
            preferred_platforms = target_preferences.get("hardware_platforms", [])
            if preferred_platforms and ContentTag.HARDWARE_FOCUSED in current_tags:
                if "hardware_examples" not in content or not content["hardware_examples"]:
                    suggestions.append({
                        "type": "hardware_examples",
                        "suggestion": f"Add examples for {', '.join(preferred_platforms[:2])} platforms",
                        "priority": "medium"
                    })
            
            # Check complexity alignment
            target_complexity = target_preferences.get("content_complexity", "moderate")
            if not self._has_complexity_tag(current_tags, target_complexity):
                suggestions.append({
                    "type": "complexity",
                    "suggestion": f"Adjust content complexity to {target_complexity} level",
                    "priority": "medium"
                })
            
            return suggestions
            
        except Exception as e:
            logger.error(f"Error suggesting content improvements: {e}")
            return []
    
    def get_content_categories(self, content_tags: Set[ContentTag]) -> Dict[str, List[str]]:
        """Get content categories based on tags."""
        try:
            categories = {
                "experience_levels": [],
                "content_types": [],
                "technology_focus": [],
                "learning_styles": [],
                "complexity_levels": []
            }
            
            for tag in content_tags:
                if tag in [ContentTag.BEGINNER_FRIENDLY, ContentTag.INTERMEDIATE_LEVEL, 
                          ContentTag.ADVANCED_CONCEPTS, ContentTag.EXPERT_ONLY]:
                    categories["experience_levels"].append(tag.value)
                
                elif tag in [ContentTag.THEORETICAL, ContentTag.PRACTICAL, 
                            ContentTag.HANDS_ON, ContentTag.CONCEPTUAL]:
                    categories["content_types"].append(tag.value)
                
                elif tag in [ContentTag.SOFTWARE_FOCUSED, ContentTag.HARDWARE_FOCUSED, 
                            ContentTag.FULL_STACK, ContentTag.EMBEDDED_SYSTEMS]:
                    categories["technology_focus"].append(tag.value)
                
                elif tag in [ContentTag.VISUAL_LEARNER, ContentTag.CODE_HEAVY, 
                            ContentTag.DIAGRAM_RICH, ContentTag.STEP_BY_STEP]:
                    categories["learning_styles"].append(tag.value)
                
                elif tag in [ContentTag.SIMPLE_EXPLANATION, ContentTag.DETAILED_ANALYSIS, 
                            ContentTag.COMPREHENSIVE_COVERAGE, ContentTag.QUICK_REFERENCE]:
                    categories["complexity_levels"].append(tag.value)
            
            return categories
            
        except Exception as e:
            logger.error(f"Error getting content categories: {e}")
            return {}
    
    def _analyze_text_content(self, text: str) -> Set[ContentTag]:
        """Analyze text content for automatic tagging."""
        tags = set()
        text_lower = text.lower()
        
        # Check for beginner-friendly language
        beginner_indicators = ["basic", "simple", "introduction", "getting started", "fundamentals"]
        if any(indicator in text_lower for indicator in beginner_indicators):
            tags.add(ContentTag.BEGINNER_FRIENDLY)
        
        # Check for advanced concepts
        advanced_indicators = ["advanced", "complex", "sophisticated", "optimization", "architecture"]
        if any(indicator in text_lower for indicator in advanced_indicators):
            tags.add(ContentTag.ADVANCED_CONCEPTS)
        
        # Check for practical content
        practical_indicators = ["example", "practice", "hands-on", "tutorial", "step-by-step"]
        if any(indicator in text_lower for indicator in practical_indicators):
            tags.add(ContentTag.PRACTICAL)
        
        # Check for theoretical content
        theoretical_indicators = ["theory", "concept", "principle", "abstract", "mathematical"]
        if any(indicator in text_lower for indicator in theoretical_indicators):
            tags.add(ContentTag.THEORETICAL)
        
        return tags
    
    def _analyze_code_examples(self, code_examples: List[Dict[str, Any]]) -> Set[ContentTag]:
        """Analyze code examples for tagging."""
        tags = set()
        
        if code_examples:
            tags.add(ContentTag.CODE_HEAVY)
            tags.add(ContentTag.PRACTICAL)
            
            # Check for different programming languages
            languages = set()
            for example in code_examples:
                if "language" in example:
                    languages.add(example["language"].lower())
            
            # Tag based on language types
            if any(lang in ["python", "javascript", "java"] for lang in languages):
                tags.add(ContentTag.SOFTWARE_FOCUSED)
            
            if any(lang in ["c", "c++", "assembly"] for lang in languages):
                tags.add(ContentTag.EMBEDDED_SYSTEMS)
        
        return tags
    
    def _analyze_hardware_content(self, hardware_examples: List[Dict[str, Any]]) -> Set[ContentTag]:
        """Analyze hardware content for tagging."""
        tags = set()
        
        if hardware_examples:
            tags.add(ContentTag.HARDWARE_FOCUSED)
            tags.add(ContentTag.HANDS_ON)
            
            # Check for embedded systems platforms
            platforms = set()
            for example in hardware_examples:
                if "platform" in example:
                    platforms.add(example["platform"].lower())
            
            if any(platform in ["arduino", "esp32", "stm32"] for platform in platforms):
                tags.add(ContentTag.EMBEDDED_SYSTEMS)
        
        return tags
    
    def _get_complexity_tags(self, complexity_level: str) -> Set[ContentTag]:
        """Get tags based on complexity level."""
        tags = set()
        
        if complexity_level == "simple":
            tags.add(ContentTag.SIMPLE_EXPLANATION)
            tags.add(ContentTag.BEGINNER_FRIENDLY)
        elif complexity_level == "detailed":
            tags.add(ContentTag.DETAILED_ANALYSIS)
        elif complexity_level == "comprehensive":
            tags.add(ContentTag.COMPREHENSIVE_COVERAGE)
            tags.add(ContentTag.ADVANCED_CONCEPTS)
        
        return tags
    
    def _get_metadata_tags(self, metadata: ContentMetadata) -> Set[ContentTag]:
        """Get tags based on content metadata."""
        tags = set()
        
        # Experience level tags
        if metadata.experience_level == ExperienceLevel.BEGINNER:
            tags.add(ContentTag.BEGINNER_FRIENDLY)
        elif metadata.experience_level == ExperienceLevel.EXPERT:
            tags.add(ContentTag.EXPERT_ONLY)
        elif metadata.experience_level == ExperienceLevel.ADVANCED:
            tags.add(ContentTag.ADVANCED_CONCEPTS)
        
        # Technology focus tags
        if metadata.software_categories:
            tags.add(ContentTag.SOFTWARE_FOCUSED)
        
        if metadata.hardware_categories:
            tags.add(ContentTag.HARDWARE_FOCUSED)
        
        if metadata.software_categories and metadata.hardware_categories:
            tags.add(ContentTag.FULL_STACK)
        
        return tags
    
    def _apply_tag_rules(self, tags: Set[ContentTag]) -> Set[ContentTag]:
        """Apply consistency rules to tags."""
        # Rule: If both beginner and expert tags exist, remove beginner
        if ContentTag.EXPERT_ONLY in tags and ContentTag.BEGINNER_FRIENDLY in tags:
            tags.remove(ContentTag.BEGINNER_FRIENDLY)
        
        # Rule: If comprehensive coverage, add detailed analysis
        if ContentTag.COMPREHENSIVE_COVERAGE in tags:
            tags.add(ContentTag.DETAILED_ANALYSIS)
        
        # Rule: If hands-on, add practical
        if ContentTag.HANDS_ON in tags:
            tags.add(ContentTag.PRACTICAL)
        
        return tags
    
    def _calculate_experience_match(self, content_tags: Set[ContentTag], user_experience: str) -> float:
        """Calculate experience level match score."""
        experience_mapping = {
            "beginner": ContentTag.BEGINNER_FRIENDLY,
            "intermediate": ContentTag.INTERMEDIATE_LEVEL,
            "advanced": ContentTag.ADVANCED_CONCEPTS,
            "expert": ContentTag.EXPERT_ONLY
        }
        
        target_tag = experience_mapping.get(user_experience)
        if target_tag and target_tag in content_tags:
            return 1.0
        
        # Partial matches
        if user_experience == "beginner" and ContentTag.INTERMEDIATE_LEVEL in content_tags:
            return 0.7
        elif user_experience == "intermediate" and ContentTag.BEGINNER_FRIENDLY in content_tags:
            return 0.8
        elif user_experience == "advanced" and ContentTag.INTERMEDIATE_LEVEL in content_tags:
            return 0.7
        
        return 0.5  # Default partial match
    
    def _calculate_content_type_match(self, content_tags: Set[ContentTag], example_style: str) -> float:
        """Calculate content type match score."""
        style_mapping = {
            "basic": ContentTag.SIMPLE_EXPLANATION,
            "practical": ContentTag.PRACTICAL,
            "advanced": ContentTag.DETAILED_ANALYSIS
        }
        
        target_tag = style_mapping.get(example_style)
        if target_tag and target_tag in content_tags:
            return 1.0
        
        return 0.5  # Default partial match
    
    def _calculate_technology_match(
        self, 
        content_tags: Set[ContentTag], 
        software_categories: List[str], 
        hardware_categories: List[str]
    ) -> float:
        """Calculate technology focus match score."""
        score = 0.0
        
        if software_categories and ContentTag.SOFTWARE_FOCUSED in content_tags:
            score += 0.5
        
        if hardware_categories and ContentTag.HARDWARE_FOCUSED in content_tags:
            score += 0.5
        
        if software_categories and hardware_categories and ContentTag.FULL_STACK in content_tags:
            score = 1.0
        
        return min(score, 1.0)
    
    def _calculate_complexity_match(self, content_tags: Set[ContentTag], complexity: str) -> float:
        """Calculate complexity match score."""
        complexity_mapping = {
            "simple": ContentTag.SIMPLE_EXPLANATION,
            "moderate": ContentTag.INTERMEDIATE_LEVEL,
            "detailed": ContentTag.DETAILED_ANALYSIS,
            "comprehensive": ContentTag.COMPREHENSIVE_COVERAGE
        }
        
        target_tag = complexity_mapping.get(complexity)
        if target_tag and target_tag in content_tags:
            return 1.0
        
        return 0.5  # Default partial match
    
    def _has_experience_level_tag(self, tags: Set[ContentTag], experience: str) -> bool:
        """Check if tags contain appropriate experience level."""
        experience_tags = {
            "beginner": ContentTag.BEGINNER_FRIENDLY,
            "intermediate": ContentTag.INTERMEDIATE_LEVEL,
            "advanced": ContentTag.ADVANCED_CONCEPTS,
            "expert": ContentTag.EXPERT_ONLY
        }
        
        return experience_tags.get(experience) in tags
    
    def _has_complexity_tag(self, tags: Set[ContentTag], complexity: str) -> bool:
        """Check if tags contain appropriate complexity level."""
        complexity_tags = {
            "simple": ContentTag.SIMPLE_EXPLANATION,
            "moderate": ContentTag.INTERMEDIATE_LEVEL,
            "detailed": ContentTag.DETAILED_ANALYSIS,
            "comprehensive": ContentTag.COMPREHENSIVE_COVERAGE
        }
        
        return complexity_tags.get(complexity) in tags
    
    def _initialize_tag_rules(self) -> Dict[str, Any]:
        """Initialize tagging rules."""
        return {
            "experience_hierarchy": [
                ContentTag.BEGINNER_FRIENDLY,
                ContentTag.INTERMEDIATE_LEVEL,
                ContentTag.ADVANCED_CONCEPTS,
                ContentTag.EXPERT_ONLY
            ],
            "complexity_hierarchy": [
                ContentTag.SIMPLE_EXPLANATION,
                ContentTag.DETAILED_ANALYSIS,
                ContentTag.COMPREHENSIVE_COVERAGE
            ]
        }
    
    def _initialize_category_mappings(self) -> Dict[str, Any]:
        """Initialize category mappings for content classification."""
        return {
            "software_categories": {
                SoftwareCategory.WEB_DEVELOPMENT: [ContentTag.SOFTWARE_FOCUSED, ContentTag.PRACTICAL],
                SoftwareCategory.EMBEDDED_SYSTEMS: [ContentTag.EMBEDDED_SYSTEMS, ContentTag.HARDWARE_FOCUSED],
                SoftwareCategory.MACHINE_LEARNING: [ContentTag.ADVANCED_CONCEPTS, ContentTag.SOFTWARE_FOCUSED]
            },
            "hardware_categories": {
                HardwareCategory.IOT_DEVICES: [ContentTag.HARDWARE_FOCUSED, ContentTag.EMBEDDED_SYSTEMS],
                HardwareCategory.ROBOTICS: [ContentTag.HARDWARE_FOCUSED, ContentTag.ADVANCED_CONCEPTS],
                HardwareCategory.ELECTRONICS: [ContentTag.HARDWARE_FOCUSED, ContentTag.HANDS_ON]
            }
        }


# Global content tagger instance
content_tagger = ContentTagger()