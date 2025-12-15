"""
User personalization service for content customization based on user background and preferences.
"""

import json
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from src.database import crud
from src.database.models import User
from src.services.embedding_service import EmbeddingService
from src.security.security_utils import SecurityValidator

logger = logging.getLogger(__name__)


class PersonalizationService:
    """Service for personalizing content based on user preferences and behavior."""
    
    def __init__(self, embedding_service: EmbeddingService):
        self.embedding_service = embedding_service
        
        # Predefined user personas and their characteristics
        self.user_personas = {
            "beginner": {
                "keywords": ["introduction", "basics", "fundamentals", "getting started", "overview"],
                "complexity_level": 1,
                "preferred_formats": ["tutorial", "guide", "example"],
                "avoid_topics": ["advanced", "complex", "expert-level"]
            },
            "intermediate": {
                "keywords": ["implementation", "practical", "application", "use case"],
                "complexity_level": 2,
                "preferred_formats": ["how-to", "case study", "best practices"],
                "avoid_topics": ["basic", "introduction"]
            },
            "advanced": {
                "keywords": ["optimization", "architecture", "performance", "scalability"],
                "complexity_level": 3,
                "preferred_formats": ["technical deep-dive", "research", "analysis"],
                "avoid_topics": ["basic", "beginner"]
            },
            "researcher": {
                "keywords": ["research", "analysis", "methodology", "findings", "study"],
                "complexity_level": 3,
                "preferred_formats": ["paper", "research", "analysis", "methodology"],
                "avoid_topics": ["tutorial", "basic guide"]
            },
            "developer": {
                "keywords": ["code", "implementation", "API", "framework", "library"],
                "complexity_level": 2,
                "preferred_formats": ["code example", "API reference", "documentation"],
                "avoid_topics": ["theory-only", "non-technical"]
            },
            "student": {
                "keywords": ["learning", "education", "concept", "theory", "explanation"],
                "complexity_level": 1,
                "preferred_formats": ["explanation", "tutorial", "exercise"],
                "avoid_topics": ["industry-specific", "commercial"]
            }
        }
        
        # Domain-specific preferences
        self.domain_preferences = {
            "machine_learning": {
                "topics": ["neural networks", "deep learning", "algorithms", "data science"],
                "formats": ["jupyter notebook", "code example", "research paper"]
            },
            "web_development": {
                "topics": ["frontend", "backend", "API", "database", "frameworks"],
                "formats": ["tutorial", "code example", "best practices"]
            },
            "data_science": {
                "topics": ["statistics", "visualization", "analysis", "modeling"],
                "formats": ["case study", "methodology", "code example"]
            },
            "cybersecurity": {
                "topics": ["security", "encryption", "vulnerability", "protection"],
                "formats": ["guide", "best practices", "case study"]
            },
            "cloud_computing": {
                "topics": ["AWS", "Azure", "GCP", "infrastructure", "deployment"],
                "formats": ["tutorial", "architecture guide", "best practices"]
            }
        }

    async def get_user_profile(self, db: Session, user_id: str) -> Dict[str, Any]:
        """Get comprehensive user profile including preferences and behavior."""
        try:
            user = crud.get_user(db, user_id)
            if not user:
                return self._get_default_profile()
            
            # Get user preferences from metadata
            preferences = user.metadata.get("preferences", {}) if user.metadata else {}
            
            # Get user interaction history
            interaction_history = self._get_user_interactions(db, user_id)
            
            # Analyze user behavior patterns
            behavior_analysis = self._analyze_user_behavior(interaction_history)
            
            # Determine user persona
            persona = self._determine_user_persona(preferences, behavior_analysis)
            
            profile = {
                "user_id": user_id,
                "persona": persona,
                "experience_level": preferences.get("experience_level", "beginner"),
                "domains_of_interest": preferences.get("domains", []),
                "preferred_formats": preferences.get("formats", []),
                "language_preference": preferences.get("language", "english"),
                "complexity_preference": preferences.get("complexity", 1),
                "behavior_patterns": behavior_analysis,
                "last_updated": datetime.utcnow().isoformat()
            }
            
            return profile
            
        except Exception as e:
            logger.error(f"Error getting user profile: {e}")
            return self._get_default_profile()

    def _get_default_profile(self) -> Dict[str, Any]:
        """Get default profile for new or anonymous users."""
        return {
            "user_id": None,
            "persona": "beginner",
            "experience_level": "beginner",
            "domains_of_interest": [],
            "preferred_formats": ["tutorial", "guide"],
            "language_preference": "english",
            "complexity_preference": 1,
            "behavior_patterns": {},
            "last_updated": datetime.utcnow().isoformat()
        }

    def _get_user_interactions(self, db: Session, user_id: str) -> List[Dict]:
        """Get user's recent interaction history."""
        try:
            # Get recent messages and queries
            recent_messages = crud.get_user_recent_messages(db, user_id, limit=50)
            
            interactions = []
            for message in recent_messages:
                if message.sender == "user":
                    interactions.append({
                        "type": "query",
                        "content": message.content,
                        "timestamp": message.created_at,
                        "metadata": message.metadata or {}
                    })
            
            return interactions
            
        except Exception as e:
            logger.error(f"Error getting user interactions: {e}")
            return []

    def _analyze_user_behavior(self, interactions: List[Dict]) -> Dict[str, Any]:
        """Analyze user behavior patterns from interaction history."""
        if not interactions:
            return {}
        
        try:
            # Analyze query complexity
            complexity_scores = []
            topics = []
            query_lengths = []
            
            for interaction in interactions:
                content = interaction.get("content", "")
                
                # Analyze query complexity (simple heuristic)
                complexity = self._calculate_query_complexity(content)
                complexity_scores.append(complexity)
                
                # Extract topics/keywords
                extracted_topics = self._extract_topics(content)
                topics.extend(extracted_topics)
                
                # Query length
                query_lengths.append(len(content.split()))
            
            # Calculate patterns
            avg_complexity = sum(complexity_scores) / len(complexity_scores) if complexity_scores else 1
            avg_query_length = sum(query_lengths) / len(query_lengths) if query_lengths else 10
            most_common_topics = self._get_most_common(topics, top_k=5)
            
            return {
                "average_complexity": avg_complexity,
                "average_query_length": avg_query_length,
                "common_topics": most_common_topics,
                "interaction_frequency": len(interactions),
                "preferred_question_types": self._analyze_question_types(interactions)
            }
            
        except Exception as e:
            logger.error(f"Error analyzing user behavior: {e}")
            return {}

    def _calculate_query_complexity(self, query: str) -> int:
        """Calculate query complexity on a scale of 1-3."""
        query_lower = query.lower()
        
        # Advanced indicators
        advanced_terms = ["optimize", "architecture", "scalability", "performance", "algorithm"]
        intermediate_terms = ["implement", "integrate", "configure", "deploy", "debug"]
        beginner_terms = ["what is", "how to", "explain", "introduction", "basic"]
        
        advanced_count = sum(1 for term in advanced_terms if term in query_lower)
        intermediate_count = sum(1 for term in intermediate_terms if term in query_lower)
        beginner_count = sum(1 for term in beginner_terms if term in query_lower)
        
        if advanced_count > 0:
            return 3
        elif intermediate_count > 0:
            return 2
        elif beginner_count > 0:
            return 1
        else:
            # Default based on query length and technical terms
            technical_terms = ["API", "database", "framework", "library", "server"]
            tech_count = sum(1 for term in technical_terms if term in query_lower)
            
            if len(query.split()) > 15 or tech_count > 2:
                return 2
            else:
                return 1

    def _extract_topics(self, content: str) -> List[str]:
        """Extract topics/keywords from content."""
        content_lower = content.lower()
        topics = []
        
        # Check for domain-specific topics
        for domain, info in self.domain_preferences.items():
            for topic in info["topics"]:
                if topic in content_lower:
                    topics.append(topic)
        
        return topics

    def _get_most_common(self, items: List[str], top_k: int = 5) -> List[str]:
        """Get most common items from a list."""
        from collections import Counter
        counter = Counter(items)
        return [item for item, count in counter.most_common(top_k)]

    def _analyze_question_types(self, interactions: List[Dict]) -> Dict[str, int]:
        """Analyze types of questions user asks."""
        question_types = {
            "how_to": 0,
            "what_is": 0,
            "why": 0,
            "comparison": 0,
            "troubleshooting": 0,
            "explanation": 0
        }
        
        for interaction in interactions:
            content = interaction.get("content", "").lower()
            
            if any(phrase in content for phrase in ["how to", "how do", "how can"]):
                question_types["how_to"] += 1
            elif any(phrase in content for phrase in ["what is", "what are", "define"]):
                question_types["what_is"] += 1
            elif "why" in content:
                question_types["why"] += 1
            elif any(phrase in content for phrase in ["vs", "versus", "compare", "difference"]):
                question_types["comparison"] += 1
            elif any(phrase in content for phrase in ["error", "problem", "issue", "fix", "debug"]):
                question_types["troubleshooting"] += 1
            else:
                question_types["explanation"] += 1
        
        return question_types

    def _determine_user_persona(self, preferences: Dict, behavior: Dict) -> str:
        """Determine user persona based on preferences and behavior."""
        # Check explicit preference
        if preferences.get("persona"):
            return preferences["persona"]
        
        # Determine from behavior
        avg_complexity = behavior.get("average_complexity", 1)
        common_topics = behavior.get("common_topics", [])
        
        # Check for researcher patterns
        research_indicators = ["research", "analysis", "study", "methodology"]
        if any(topic in research_indicators for topic in common_topics):
            return "researcher"
        
        # Check for developer patterns
        dev_indicators = ["code", "API", "framework", "implementation"]
        if any(topic in dev_indicators for topic in common_topics):
            return "developer"
        
        # Check complexity level
        if avg_complexity >= 2.5:
            return "advanced"
        elif avg_complexity >= 1.5:
            return "intermediate"
        else:
            return "beginner"

    async def personalize_content(
        self, 
        user_profile: Dict[str, Any], 
        content_chunks: List[str], 
        query: str
    ) -> List[str]:
        """Personalize content based on user profile."""
        try:
            if not content_chunks:
                return []
            
            persona = user_profile.get("persona", "beginner")
            complexity_pref = user_profile.get("complexity_preference", 1)
            domains = user_profile.get("domains_of_interest", [])
            
            # Score and rank content chunks
            scored_chunks = []
            
            for chunk in content_chunks:
                score = self._score_content_relevance(
                    chunk, query, persona, complexity_pref, domains
                )
                scored_chunks.append((chunk, score))
            
            # Sort by score and return top chunks
            scored_chunks.sort(key=lambda x: x[1], reverse=True)
            
            # Return personalized chunks (limit to top 5)
            return [chunk for chunk, score in scored_chunks[:5]]
            
        except Exception as e:
            logger.error(f"Error personalizing content: {e}")
            return content_chunks[:5]  # Fallback to original chunks

    def _score_content_relevance(
        self, 
        content: str, 
        query: str, 
        persona: str, 
        complexity_pref: int, 
        domains: List[str]
    ) -> float:
        """Score content relevance based on user preferences."""
        score = 0.0
        content_lower = content.lower()
        query_lower = query.lower()
        
        # Base relevance to query (simple keyword matching)
        query_words = query_lower.split()
        matching_words = sum(1 for word in query_words if word in content_lower)
        score += (matching_words / len(query_words)) * 0.4
        
        # Persona-based scoring
        if persona in self.user_personas:
            persona_info = self.user_personas[persona]
            
            # Preferred keywords
            keyword_matches = sum(
                1 for keyword in persona_info["keywords"] 
                if keyword in content_lower
            )
            score += (keyword_matches / len(persona_info["keywords"])) * 0.3
            
            # Avoid topics penalty
            avoid_matches = sum(
                1 for avoid_topic in persona_info["avoid_topics"] 
                if avoid_topic in content_lower
            )
            score -= avoid_matches * 0.2
            
            # Complexity level matching
            content_complexity = self._estimate_content_complexity(content)
            complexity_diff = abs(content_complexity - complexity_pref)
            score += (3 - complexity_diff) / 3 * 0.2
        
        # Domain relevance
        if domains:
            domain_score = 0
            for domain in domains:
                if domain in self.domain_preferences:
                    domain_topics = self.domain_preferences[domain]["topics"]
                    domain_matches = sum(
                        1 for topic in domain_topics 
                        if topic in content_lower
                    )
                    domain_score += domain_matches / len(domain_topics)
            
            score += (domain_score / len(domains)) * 0.1
        
        return max(0.0, min(1.0, score))  # Clamp between 0 and 1

    def _estimate_content_complexity(self, content: str) -> int:
        """Estimate content complexity level (1-3)."""
        content_lower = content.lower()
        
        # Technical terms and advanced concepts
        advanced_indicators = [
            "algorithm", "optimization", "architecture", "scalability", 
            "performance", "complexity", "distributed", "concurrent"
        ]
        
        intermediate_indicators = [
            "implementation", "configuration", "integration", "deployment",
            "framework", "library", "API", "database"
        ]
        
        beginner_indicators = [
            "introduction", "basic", "simple", "getting started",
            "overview", "fundamentals", "tutorial"
        ]
        
        advanced_count = sum(1 for term in advanced_indicators if term in content_lower)
        intermediate_count = sum(1 for term in intermediate_indicators if term in content_lower)
        beginner_count = sum(1 for term in beginner_indicators if term in content_lower)
        
        if advanced_count > intermediate_count and advanced_count > beginner_count:
            return 3
        elif intermediate_count > beginner_count:
            return 2
        else:
            return 1

    async def update_user_preferences(
        self, 
        db: Session, 
        user_id: str, 
        preferences: Dict[str, Any]
    ) -> bool:
        """Update user preferences."""
        try:
            user = crud.get_user(db, user_id)
            if not user:
                return False
            
            # Validate preferences
            validated_prefs = self._validate_preferences(preferences)
            
            # Update user metadata
            current_metadata = user.metadata or {}
            current_metadata["preferences"] = validated_prefs
            current_metadata["preferences_updated"] = datetime.utcnow().isoformat()
            
            user.metadata = current_metadata
            db.commit()
            
            logger.info(f"Updated preferences for user {user_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error updating user preferences: {e}")
            return False

    def _validate_preferences(self, preferences: Dict[str, Any]) -> Dict[str, Any]:
        """Validate and sanitize user preferences."""
        validated = {}
        
        # Validate persona
        if "persona" in preferences:
            persona = preferences["persona"]
            if persona in self.user_personas:
                validated["persona"] = persona
        
        # Validate experience level
        if "experience_level" in preferences:
            level = preferences["experience_level"]
            if level in ["beginner", "intermediate", "advanced"]:
                validated["experience_level"] = level
        
        # Validate domains
        if "domains" in preferences:
            domains = preferences["domains"]
            if isinstance(domains, list):
                valid_domains = [
                    domain for domain in domains 
                    if domain in self.domain_preferences
                ]
                validated["domains"] = valid_domains[:5]  # Limit to 5 domains
        
        # Validate complexity preference
        if "complexity" in preferences:
            complexity = preferences["complexity"]
            if isinstance(complexity, int) and 1 <= complexity <= 3:
                validated["complexity"] = complexity
        
        # Validate language preference
        if "language" in preferences:
            language = SecurityValidator.sanitize_input(preferences["language"])
            if language and len(language) <= 20:
                validated["language"] = language
        
        return validated

    async def get_personalized_suggestions(
        self, 
        user_profile: Dict[str, Any], 
        current_query: str
    ) -> List[str]:
        """Get personalized query suggestions based on user profile."""
        try:
            persona = user_profile.get("persona", "beginner")
            domains = user_profile.get("domains_of_interest", [])
            behavior = user_profile.get("behavior_patterns", {})
            
            suggestions = []
            
            # Persona-based suggestions
            if persona in self.user_personas:
                persona_info = self.user_personas[persona]
                for keyword in persona_info["keywords"][:3]:
                    suggestions.append(f"Tell me about {keyword} in {current_query}")
            
            # Domain-based suggestions
            for domain in domains[:2]:
                if domain in self.domain_preferences:
                    domain_topics = self.domain_preferences[domain]["topics"]
                    for topic in domain_topics[:2]:
                        suggestions.append(f"How does {topic} relate to {current_query}?")
            
            # Behavior-based suggestions
            common_topics = behavior.get("common_topics", [])
            for topic in common_topics[:2]:
                suggestions.append(f"Explain {topic} in the context of {current_query}")
            
            # Remove duplicates and limit
            unique_suggestions = list(set(suggestions))
            return unique_suggestions[:5]
            
        except Exception as e:
            logger.error(f"Error generating personalized suggestions: {e}")
            return []