"""User Experience Enhancement System for Multilingual Features."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict
import uuid
from collections import defaultdict, Counter
import numpy as np

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, desc

from app.models.database import User
from app.models.multilingual import ContentTranslation, Language

logger = logging.getLogger(__name__)


class LearningStyle(str, Enum):
    """Learning style types."""
    VISUAL = "visual"
    AUDITORY = "auditory"
    KINESTHETIC = "kinesthetic"
    READING = "reading"


class InsightType(str, Enum):
    """Types of learning insights."""
    IMPROVEMENT = "improvement"
    RECOMMENDATION = "recommendation"
    PATTERN = "pattern"
    ACHIEVEMENT = "achievement"


@dataclass
class UserBehaviorData:
    """User behavior tracking data."""
    user_id: int
    session_id: str
    timestamp: datetime
    action: str
    context: Dict[str, Any]
    language_code: str
    duration: Optional[float] = None
    success: Optional[bool] = None


@dataclass
class LearningInsight:
    """Learning insight for user."""
    type: InsightType
    title: str
    description: str
    confidence: float
    actionable: bool
    implemented: bool
    metadata: Dict[str, Any]


@dataclass
class PersonalizationRule:
    """Personalization rule based on user behavior."""
    condition: str
    action: str
    priority: int
    confidence: float
    metadata: Dict[str, Any]


class UXEnhancementSystem:
    """System for enhancing user experience based on feedback and behavior."""
    
    def __init__(self):
        self.behavior_data = defaultdict(list)
        self.user_insights = defaultdict(list)
        self.personalization_rules = []
        self.learning_patterns = {}
        
        # Behavior tracking thresholds
        self.thresholds = {
            "language_switch_frequency": 5,  # switches per session
            "translation_view_time": 30,     # seconds
            "error_rate": 0.1,               # 10% error rate
            "engagement_time": 300           # 5 minutes per session
        }
        
        # Initialize default personalization rules
        self._initialize_personalization_rules()
    
    def _initialize_personalization_rules(self):
        """Initialize default personalization rules."""
        self.personalization_rules = [
            PersonalizationRule(
                condition="frequent_language_switcher",
                action="show_language_toggle_shortcut",
                priority=1,
                confidence=0.9,
                metadata={"trigger_count": 5}
            ),
            PersonalizationRule(
                condition="slow_reader",
                action="increase_font_size",
                priority=2,
                confidence=0.8,
                metadata={"reading_speed_threshold": 150}  # words per minute
            ),
            PersonalizationRule(
                condition="translation_quality_seeker",
                action="show_confidence_scores",
                priority=1,
                confidence=0.85,
                metadata={"quality_threshold": 0.9}
            ),
            PersonalizationRule(
                condition="mobile_user",
                action="optimize_mobile_layout",
                priority=3,
                confidence=0.95,
                metadata={"screen_width_threshold": 768}
            )
        ]
    
    async def track_user_behavior(
        self,
        user_id: int,
        action: str,
        context: Dict[str, Any],
        language_code: str,
        duration: Optional[float] = None,
        success: Optional[bool] = None
    ) -> None:
        """Track user behavior for analysis."""
        try:
            behavior = UserBehaviorData(
                user_id=user_id,
                session_id=context.get("session_id", str(uuid.uuid4())),
                timestamp=datetime.utcnow(),
                action=action,
                context=context,
                language_code=language_code,
                duration=duration,
                success=success
            )
            
            self.behavior_data[user_id].append(behavior)
            
            # Keep only recent behavior data (last 30 days)
            cutoff_date = datetime.utcnow() - timedelta(days=30)
            self.behavior_data[user_id] = [
                b for b in self.behavior_data[user_id]
                if b.timestamp >= cutoff_date
            ]
            
            # Trigger real-time analysis for immediate insights
            await self._analyze_immediate_behavior(user_id, behavior)
            
        except Exception as e:
            logger.error(f"Error tracking user behavior: {e}")
    
    async def _analyze_immediate_behavior(
        self,
        user_id: int,
        behavior: UserBehaviorData
    ) -> None:
        """Analyze behavior for immediate insights."""
        try:
            user_behaviors = self.behavior_data[user_id]
            
            # Check for patterns that need immediate action
            recent_behaviors = [
                b for b in user_behaviors
                if (datetime.utcnow() - b.timestamp).total_seconds() < 3600  # Last hour
            ]
            
            # Detect frustration patterns
            if len(recent_behaviors) >= 3:
                error_rate = sum(1 for b in recent_behaviors if b.success is False) / len(recent_behaviors)
                if error_rate > self.thresholds["error_rate"]:
                    await self._generate_frustration_insight(user_id, error_rate)
            
            # Detect engagement patterns
            if behavior.action == "language_switch":
                switch_count = sum(1 for b in recent_behaviors if b.action == "language_switch")
                if switch_count >= self.thresholds["language_switch_frequency"]:
                    await self._generate_language_switching_insight(user_id, switch_count)
            
        except Exception as e:
            logger.error(f"Error in immediate behavior analysis: {e}")
    
    async def analyze_user_patterns(self, user_id: int) -> List[LearningInsight]:
        """Analyze user behavior patterns and generate insights."""
        try:
            user_behaviors = self.behavior_data.get(user_id, [])
            if not user_behaviors:
                return []
            
            insights = []
            
            # Analyze learning style
            learning_style_insight = await self._analyze_learning_style(user_id, user_behaviors)
            if learning_style_insight:
                insights.append(learning_style_insight)
            
            # Analyze language preferences
            language_insights = await self._analyze_language_preferences(user_id, user_behaviors)
            insights.extend(language_insights)
            
            # Analyze engagement patterns
            engagement_insights = await self._analyze_engagement_patterns(user_id, user_behaviors)
            insights.extend(engagement_insights)
            
            # Analyze performance patterns
            performance_insights = await self._analyze_performance_patterns(user_id, user_behaviors)
            insights.extend(performance_insights)
            
            # Store insights
            self.user_insights[user_id] = insights
            
            return insights
            
        except Exception as e:
            logger.error(f"Error analyzing user patterns: {e}")
            return []
    
    async def _analyze_learning_style(
        self,
        user_id: int,
        behaviors: List[UserBehaviorData]
    ) -> Optional[LearningInsight]:
        """Analyze user's learning style based on behavior."""
        try:
            # Count different types of interactions
            visual_actions = sum(1 for b in behaviors if b.action in [
                "view_image", "use_diagram", "color_coding", "visual_aid"
            ])
            
            auditory_actions = sum(1 for b in behaviors if b.action in [
                "play_audio", "pronunciation", "listen_explanation"
            ])
            
            kinesthetic_actions = sum(1 for b in behaviors if b.action in [
                "interactive_exercise", "drag_drop", "hands_on_practice"
            ])
            
            reading_actions = sum(1 for b in behaviors if b.action in [
                "read_text", "view_translation", "compare_languages"
            ])
            
            total_actions = len(behaviors)
            if total_actions < 10:  # Need sufficient data
                return None
            
            # Calculate percentages
            style_scores = {
                LearningStyle.VISUAL: visual_actions / total_actions,
                LearningStyle.AUDITORY: auditory_actions / total_actions,
                LearningStyle.KINESTHETIC: kinesthetic_actions / total_actions,
                LearningStyle.READING: reading_actions / total_actions
            }
            
            # Find dominant learning style
            dominant_style = max(style_scores, key=style_scores.get)
            confidence = style_scores[dominant_style]
            
            if confidence > 0.3:  # Minimum confidence threshold
                return LearningInsight(
                    type=InsightType.PATTERN,
                    title=f"You're a {dominant_style.value} learner",
                    description=f"Based on your interactions, you prefer {dominant_style.value} learning methods. We can optimize your experience accordingly.",
                    confidence=confidence,
                    actionable=True,
                    implemented=False,
                    metadata={
                        "learning_style": dominant_style.value,
                        "style_scores": style_scores
                    }
                )
            
            return None
            
        except Exception as e:
            logger.error(f"Error analyzing learning style: {e}")
            return None
    
    async def _analyze_language_preferences(
        self,
        user_id: int,
        behaviors: List[UserBehaviorData]
    ) -> List[LearningInsight]:
        """Analyze language usage preferences."""
        try:
            insights = []
            
            # Analyze language usage frequency
            language_usage = Counter(b.language_code for b in behaviors)
            total_usage = sum(language_usage.values())
            
            if total_usage < 5:
                return insights
            
            # Find primary language
            primary_lang = language_usage.most_common(1)[0]
            primary_percentage = primary_lang[1] / total_usage
            
            if primary_percentage > 0.8:
                insights.append(LearningInsight(
                    type=InsightType.PATTERN,
                    title=f"You primarily use {primary_lang[0].upper()}",
                    description=f"You use {primary_lang[0].upper()} for {primary_percentage:.0%} of your interactions. Consider exploring bilingual features for better learning.",
                    confidence=primary_percentage,
                    actionable=True,
                    implemented=False,
                    metadata={
                        "primary_language": primary_lang[0],
                        "usage_percentage": primary_percentage
                    }
                ))
            
            # Analyze language switching patterns
            language_switches = sum(1 for b in behaviors if b.action == "language_switch")
            if language_switches > 10:
                switch_rate = language_switches / total_usage
                insights.append(LearningInsight(
                    type=InsightType.RECOMMENDATION,
                    title="Frequent language switcher",
                    description="You switch languages frequently. We can add a quick language toggle for easier access.",
                    confidence=min(switch_rate * 2, 1.0),
                    actionable=True,
                    implemented=False,
                    metadata={
                        "switch_count": language_switches,
                        "switch_rate": switch_rate
                    }
                ))
            
            return insights
            
        except Exception as e:
            logger.error(f"Error analyzing language preferences: {e}")
            return []
    
    async def _analyze_engagement_patterns(
        self,
        user_id: int,
        behaviors: List[UserBehaviorData]
    ) -> List[LearningInsight]:
        """Analyze user engagement patterns."""
        try:
            insights = []
            
            # Calculate session durations
            sessions = defaultdict(list)
            for behavior in behaviors:
                sessions[behavior.session_id].append(behavior)
            
            session_durations = []
            for session_behaviors in sessions.values():
                if len(session_behaviors) > 1:
                    start_time = min(b.timestamp for b in session_behaviors)
                    end_time = max(b.timestamp for b in session_behaviors)
                    duration = (end_time - start_time).total_seconds()
                    session_durations.append(duration)
            
            if session_durations:
                avg_session_duration = np.mean(session_durations)
                
                if avg_session_duration > self.thresholds["engagement_time"]:
                    insights.append(LearningInsight(
                        type=InsightType.ACHIEVEMENT,
                        title="High engagement learner",
                        description=f"Your average session lasts {avg_session_duration/60:.1f} minutes, showing strong engagement with the content.",
                        confidence=min(avg_session_duration / 1800, 1.0),  # Max at 30 minutes
                        actionable=False,
                        implemented=False,
                        metadata={
                            "avg_session_duration": avg_session_duration,
                            "total_sessions": len(session_durations)
                        }
                    ))
                elif avg_session_duration < 60:  # Less than 1 minute
                    insights.append(LearningInsight(
                        type=InsightType.RECOMMENDATION,
                        title="Short session pattern",
                        description="Your sessions are quite short. Consider setting learning goals or using our progress tracking features.",
                        confidence=0.8,
                        actionable=True,
                        implemented=False,
                        metadata={
                            "avg_session_duration": avg_session_duration,
                            "recommendation": "goal_setting"
                        }
                    ))
            
            return insights
            
        except Exception as e:
            logger.error(f"Error analyzing engagement patterns: {e}")
            return []
    
    async def _analyze_performance_patterns(
        self,
        user_id: int,
        behaviors: List[UserBehaviorData]
    ) -> List[LearningInsight]:
        """Analyze user performance patterns."""
        try:
            insights = []
            
            # Analyze success rates
            success_behaviors = [b for b in behaviors if b.success is not None]
            if len(success_behaviors) >= 10:
                success_rate = sum(1 for b in success_behaviors if b.success) / len(success_behaviors)
                
                if success_rate > 0.9:
                    insights.append(LearningInsight(
                        type=InsightType.ACHIEVEMENT,
                        title="Excellent performance",
                        description=f"You have a {success_rate:.0%} success rate. You're mastering the multilingual features!",
                        confidence=success_rate,
                        actionable=False,
                        implemented=False,
                        metadata={
                            "success_rate": success_rate,
                            "total_attempts": len(success_behaviors)
                        }
                    ))
                elif success_rate < 0.6:
                    insights.append(LearningInsight(
                        type=InsightType.IMPROVEMENT,
                        title="Room for improvement",
                        description=f"Your success rate is {success_rate:.0%}. Consider using our guided tutorials or help features.",
                        confidence=1.0 - success_rate,
                        actionable=True,
                        implemented=False,
                        metadata={
                            "success_rate": success_rate,
                            "recommendation": "guided_tutorials"
                        }
                    ))
            
            # Analyze reading speed (if available)
            reading_behaviors = [b for b in behaviors if b.action == "read_content" and b.duration]
            if len(reading_behaviors) >= 5:
                avg_reading_time = np.mean([b.duration for b in reading_behaviors])
                # Estimate words per minute (assuming average content length)
                estimated_wpm = 200 / (avg_reading_time / 60)  # Assuming 200 words average
                
                if estimated_wpm < 150:  # Slow reader
                    insights.append(LearningInsight(
                        type=InsightType.RECOMMENDATION,
                        title="Optimize for comfortable reading",
                        description="You take time to read content carefully. We can increase font size and spacing for better readability.",
                        confidence=0.8,
                        actionable=True,
                        implemented=False,
                        metadata={
                            "estimated_wpm": estimated_wpm,
                            "recommendation": "increase_font_size"
                        }
                    ))
            
            return insights
            
        except Exception as e:
            logger.error(f"Error analyzing performance patterns: {e}")
            return []
    
    async def _generate_frustration_insight(self, user_id: int, error_rate: float) -> None:
        """Generate insight for user frustration."""
        insight = LearningInsight(
            type=InsightType.IMPROVEMENT,
            title="Experiencing some difficulties",
            description=f"We noticed you're having some challenges (error rate: {error_rate:.0%}). Would you like some help or guidance?",
            confidence=error_rate,
            actionable=True,
            implemented=False,
            metadata={
                "error_rate": error_rate,
                "trigger": "immediate_frustration",
                "recommendation": "offer_help"
            }
        )
        
        self.user_insights[user_id].append(insight)
    
    async def _generate_language_switching_insight(self, user_id: int, switch_count: int) -> None:
        """Generate insight for frequent language switching."""
        insight = LearningInsight(
            type=InsightType.RECOMMENDATION,
            title="Frequent language switching detected",
            description=f"You've switched languages {switch_count} times recently. We can add a keyboard shortcut for quicker switching.",
            confidence=min(switch_count / 10, 1.0),
            actionable=True,
            implemented=False,
            metadata={
                "switch_count": switch_count,
                "trigger": "immediate_switching",
                "recommendation": "keyboard_shortcut"
            }
        )
        
        self.user_insights[user_id].append(insight)
    
    async def get_personalization_recommendations(
        self,
        user_id: int,
        context: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        """Get personalization recommendations for user."""
        try:
            recommendations = []
            user_behaviors = self.behavior_data.get(user_id, [])
            
            if not user_behaviors:
                return recommendations
            
            # Check each personalization rule
            for rule in self.personalization_rules:
                if await self._evaluate_rule_condition(rule, user_id, user_behaviors, context):
                    recommendations.append({
                        "action": rule.action,
                        "priority": rule.priority,
                        "confidence": rule.confidence,
                        "metadata": rule.metadata
                    })
            
            # Sort by priority and confidence
            recommendations.sort(key=lambda x: (x["priority"], -x["confidence"]))
            
            return recommendations
            
        except Exception as e:
            logger.error(f"Error getting personalization recommendations: {e}")
            return []
    
    async def _evaluate_rule_condition(
        self,
        rule: PersonalizationRule,
        user_id: int,
        behaviors: List[UserBehaviorData],
        context: Dict[str, Any]
    ) -> bool:
        """Evaluate if a personalization rule condition is met."""
        try:
            if rule.condition == "frequent_language_switcher":
                switch_count = sum(1 for b in behaviors if b.action == "language_switch")
                return switch_count >= rule.metadata.get("trigger_count", 5)
            
            elif rule.condition == "slow_reader":
                reading_behaviors = [b for b in behaviors if b.action == "read_content" and b.duration]
                if len(reading_behaviors) >= 3:
                    avg_reading_time = np.mean([b.duration for b in reading_behaviors])
                    estimated_wpm = 200 / (avg_reading_time / 60)
                    return estimated_wpm < rule.metadata.get("reading_speed_threshold", 150)
            
            elif rule.condition == "translation_quality_seeker":
                quality_checks = sum(1 for b in behaviors if b.action == "check_translation_quality")
                return quality_checks >= 3
            
            elif rule.condition == "mobile_user":
                screen_width = context.get("screen_width", 1920)
                return screen_width <= rule.metadata.get("screen_width_threshold", 768)
            
            return False
            
        except Exception as e:
            logger.error(f"Error evaluating rule condition: {e}")
            return False
    
    def get_user_insights(self, user_id: int) -> List[LearningInsight]:
        """Get stored insights for user."""
        return self.user_insights.get(user_id, [])
    
    async def implement_insight(self, user_id: int, insight_index: int) -> bool:
        """Mark an insight as implemented."""
        try:
            user_insights = self.user_insights.get(user_id, [])
            if 0 <= insight_index < len(user_insights):
                user_insights[insight_index].implemented = True
                return True
            return False
            
        except Exception as e:
            logger.error(f"Error implementing insight: {e}")
            return False


# Global UX enhancement system instance
ux_enhancement = UXEnhancementSystem()