"""Translation workflow optimization with ML and quality prediction."""

import logging
import asyncio
import json
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict
import uuid
import numpy as np
from collections import defaultdict, Counter
from sklearn.ensemble import RandomForestRegressor
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func, desc

from app.models.multilingual import (
    ContentTranslation, TranslationMemory, TerminologyGlossary,
    TranslationStatus, TranslationMethod, Language
)
from app.models.database import User

logger = logging.getLogger(__name__)


class TranslatorExpertise(str, Enum):
    """Translator expertise levels."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"
    NATIVE = "native"


class WorkflowStage(str, Enum):
    """Translation workflow stages."""
    PREPARATION = "preparation"
    TRANSLATION = "translation"
    REVIEW = "review"
    QUALITY_CHECK = "quality_check"
    APPROVAL = "approval"
    PUBLICATION = "publication"


@dataclass
class TranslatorProfile:
    """Translator profile with expertise and performance metrics."""
    user_id: int
    expertise_level: TranslatorExpertise
    specializations: List[str]
    languages: Dict[str, str]  # language_code -> proficiency_level
    performance_metrics: Dict[str, float]
    availability: Dict[str, Any]
    quality_score: float
    productivity_score: float


@dataclass
class QualityPrediction:
    """Quality prediction for translation."""
    predicted_score: float
    confidence: float
    factors: Dict[str, float]
    recommendations: List[str]
    estimated_time: float


@dataclass
class WorkflowOptimization:
    """Workflow optimization recommendation."""
    stage: WorkflowStage
    optimization_type: str
    description: str
    impact_score: float
    implementation_effort: str
    metadata: Dict[str, Any]


class TranslationWorkflowOptimizer:
    """Advanced translation workflow optimizer with ML capabilities."""
    
    def __init__(self):
        self.translator_profiles = {}
        self.quality_model = None
        self.productivity_model = None
        self.terminology_consistency_checker = None
        self.workflow_analytics = defaultdict(list)
        
        # Quality factors and weights
        self.quality_factors = {
            "translator_experience": 0.25,
            "content_complexity": 0.20,
            "terminology_consistency": 0.20,
            "review_thoroughness": 0.15,
            "time_pressure": 0.10,
            "domain_expertise": 0.10
        }
        
        # Initialize ML models
        self._initialize_ml_models()
        
        # Workflow optimization rules
        self.optimization_rules = self._initialize_optimization_rules()
    
    def _initialize_ml_models(self):
        """Initialize machine learning models for quality prediction."""
        try:
            # Quality prediction model
            self.quality_model = RandomForestRegressor(
                n_estimators=100,
                random_state=42,
                max_depth=10
            )
            
            # Productivity prediction model
            self.productivity_model = RandomForestRegressor(
                n_estimators=50,
                random_state=42,
                max_depth=8
            )
            
            # Initialize with some sample data (in production, load from historical data)
            self._train_initial_models()
            
        except Exception as e:
            logger.error(f"Error initializing ML models: {e}")
    
    def _train_initial_models(self):
        """Train models with initial sample data."""
        try:
            # Sample training data for quality prediction
            # Features: [translator_exp, complexity, terminology_score, review_time, domain_match]
            X_quality = np.array([
                [0.9, 0.3, 0.8, 120, 0.9],  # Expert translator, simple content
                [0.5, 0.7, 0.6, 60, 0.5],   # Intermediate, complex content
                [0.8, 0.4, 0.9, 180, 0.8],  # Advanced, good terminology
                [0.3, 0.8, 0.4, 30, 0.3],   # Beginner, complex content
                [0.95, 0.2, 0.95, 240, 0.95] # Native speaker, simple content
            ])
            
            y_quality = np.array([0.92, 0.65, 0.88, 0.45, 0.98])
            
            self.quality_model.fit(X_quality, y_quality)
            
            # Sample training data for productivity prediction
            # Features: [translator_exp, content_length, familiarity, tools_used, time_of_day]
            X_productivity = np.array([
                [0.9, 1000, 0.8, 1, 0.8],   # Expert, long content, familiar
                [0.5, 500, 0.6, 0, 0.5],    # Intermediate, short content
                [0.8, 1500, 0.9, 1, 0.9],   # Advanced, very familiar
                [0.3, 800, 0.3, 0, 0.3],    # Beginner, unfamiliar
                [0.95, 1200, 0.95, 1, 0.95] # Native, very familiar
            ])
            
            y_productivity = np.array([450, 200, 600, 120, 800])  # words per hour
            
            self.productivity_model.fit(X_productivity, y_productivity)
            
            logger.info("ML models initialized with sample data")
            
        except Exception as e:
            logger.error(f"Error training initial models: {e}")
    
    def _initialize_optimization_rules(self) -> List[WorkflowOptimization]:
        """Initialize workflow optimization rules."""
        return [
            WorkflowOptimization(
                stage=WorkflowStage.PREPARATION,
                optimization_type="content_analysis",
                description="Analyze content complexity and domain before assignment",
                impact_score=0.8,
                implementation_effort="medium",
                metadata={"automated": True, "time_saving": 30}
            ),
            WorkflowOptimization(
                stage=WorkflowStage.TRANSLATION,
                optimization_type="smart_assignment",
                description="Assign translations based on translator expertise and availability",
                impact_score=0.9,
                implementation_effort="high",
                metadata={"automated": True, "quality_improvement": 25}
            ),
            WorkflowOptimization(
                stage=WorkflowStage.REVIEW,
                optimization_type="parallel_review",
                description="Enable parallel review by multiple reviewers for critical content",
                impact_score=0.7,
                implementation_effort="medium",
                metadata={"quality_improvement": 35, "time_increase": 20}
            ),
            WorkflowOptimization(
                stage=WorkflowStage.QUALITY_CHECK,
                optimization_type="automated_checks",
                description="Implement automated quality checks before human review",
                impact_score=0.85,
                implementation_effort="high",
                metadata={"automated": True, "error_reduction": 40}
            )
        ]
    
    async def predict_translation_quality(
        self,
        content: str,
        translator_id: int,
        domain: str,
        target_language: str,
        db: Session
    ) -> QualityPrediction:
        """Predict translation quality using ML model."""
        try:
            # Get translator profile
            translator = await self._get_translator_profile(translator_id, db)
            if not translator:
                return QualityPrediction(
                    predicted_score=0.5,
                    confidence=0.3,
                    factors={},
                    recommendations=["Translator profile not found"],
                    estimated_time=0
                )
            
            # Extract features
            features = await self._extract_quality_features(
                content, translator, domain, target_language, db
            )
            
            # Predict quality
            feature_vector = np.array([list(features.values())]).reshape(1, -1)
            predicted_score = self.quality_model.predict(feature_vector)[0]
            
            # Calculate confidence based on feature certainty
            confidence = self._calculate_prediction_confidence(features)
            
            # Generate recommendations
            recommendations = await self._generate_quality_recommendations(
                features, predicted_score
            )
            
            # Estimate time
            estimated_time = await self._estimate_translation_time(
                content, translator, domain
            )
            
            return QualityPrediction(
                predicted_score=max(0.0, min(1.0, predicted_score)),
                confidence=confidence,
                factors=features,
                recommendations=recommendations,
                estimated_time=estimated_time
            )
            
        except Exception as e:
            logger.error(f"Error predicting translation quality: {e}")
            return QualityPrediction(
                predicted_score=0.5,
                confidence=0.2,
                factors={},
                recommendations=["Error in quality prediction"],
                estimated_time=0
            )
    
    async def _extract_quality_features(
        self,
        content: str,
        translator: TranslatorProfile,
        domain: str,
        target_language: str,
        db: Session
    ) -> Dict[str, float]:
        """Extract features for quality prediction."""
        try:
            features = {}
            
            # Translator experience feature
            experience_mapping = {
                TranslatorExpertise.BEGINNER: 0.2,
                TranslatorExpertise.INTERMEDIATE: 0.4,
                TranslatorExpertise.ADVANCED: 0.7,
                TranslatorExpertise.EXPERT: 0.9,
                TranslatorExpertise.NATIVE: 1.0
            }
            features["translator_experience"] = experience_mapping.get(
                translator.expertise_level, 0.5
            )
            
            # Content complexity feature
            features["content_complexity"] = await self._calculate_content_complexity(content)
            
            # Terminology consistency feature
            features["terminology_consistency"] = await self._check_terminology_availability(
                content, target_language, domain, db
            )
            
            # Domain expertise feature
            domain_match = 1.0 if domain in translator.specializations else 0.5
            features["domain_expertise"] = domain_match
            
            # Language proficiency feature
            lang_proficiency = translator.languages.get(target_language, "intermediate")
            proficiency_mapping = {
                "beginner": 0.3,
                "intermediate": 0.6,
                "advanced": 0.8,
                "native": 1.0
            }
            features["language_proficiency"] = proficiency_mapping.get(lang_proficiency, 0.6)
            
            return features
            
        except Exception as e:
            logger.error(f"Error extracting quality features: {e}")
            return {}
    
    async def _calculate_content_complexity(self, content: str) -> float:
        """Calculate content complexity score."""
        try:
            # Simple complexity metrics
            word_count = len(content.split())
            sentence_count = len([s for s in content.split('.') if s.strip()])
            avg_sentence_length = word_count / max(sentence_count, 1)
            
            # Technical term density
            technical_terms = [
                'algorithm', 'function', 'variable', 'database', 'system',
                'framework', 'library', 'api', 'interface', 'protocol'
            ]
            technical_density = sum(1 for word in content.lower().split() 
                                  if word in technical_terms) / max(word_count, 1)
            
            # Complexity score (0-1)
            complexity = min(1.0, (
                (avg_sentence_length / 20) * 0.4 +  # Sentence complexity
                technical_density * 0.6              # Technical density
            ))
            
            return complexity
            
        except Exception as e:
            logger.error(f"Error calculating content complexity: {e}")
            return 0.5
    
    async def _check_terminology_availability(
        self,
        content: str,
        target_language: str,
        domain: str,
        db: Session
    ) -> float:
        """Check availability of terminology for content."""
        try:
            # Extract potential terms from content
            words = content.lower().split()
            
            # Query terminology database
            terminology_entries = db.query(TerminologyGlossary).filter(
                and_(
                    TerminologyGlossary.target_language == target_language,
                    TerminologyGlossary.domain == domain,
                    TerminologyGlossary.approved == True
                )
            ).all()
            
            available_terms = {entry.term.lower() for entry in terminology_entries}
            
            # Calculate coverage
            technical_words = [word for word in words if len(word) > 4]  # Assume longer words are more technical
            covered_terms = sum(1 for word in technical_words if word in available_terms)
            
            coverage = covered_terms / max(len(technical_words), 1)
            return min(1.0, coverage)
            
        except Exception as e:
            logger.error(f"Error checking terminology availability: {e}")
            return 0.5
    
    def _calculate_prediction_confidence(self, features: Dict[str, float]) -> float:
        """Calculate confidence in quality prediction."""
        try:
            # Confidence based on feature completeness and certainty
            feature_completeness = len(features) / 5  # Expected 5 features
            feature_certainty = np.mean(list(features.values()))
            
            confidence = (feature_completeness * 0.3 + feature_certainty * 0.7)
            return min(1.0, confidence)
            
        except Exception as e:
            logger.error(f"Error calculating prediction confidence: {e}")
            return 0.5
    
    async def _generate_quality_recommendations(
        self,
        features: Dict[str, float],
        predicted_score: float
    ) -> List[str]:
        """Generate recommendations to improve quality."""
        recommendations = []
        
        try:
            if predicted_score < 0.7:
                recommendations.append("Consider assigning to a more experienced translator")
            
            if features.get("terminology_consistency", 0) < 0.6:
                recommendations.append("Review and update terminology glossary for this domain")
            
            if features.get("content_complexity", 0) > 0.8:
                recommendations.append("Break down complex content into smaller sections")
            
            if features.get("domain_expertise", 0) < 0.7:
                recommendations.append("Assign to translator with domain expertise")
            
            if not recommendations:
                recommendations.append("Quality prediction looks good - proceed with translation")
            
            return recommendations
            
        except Exception as e:
            logger.error(f"Error generating recommendations: {e}")
            return ["Error generating recommendations"]
    
    async def _estimate_translation_time(
        self,
        content: str,
        translator: TranslatorProfile,
        domain: str
    ) -> float:
        """Estimate translation time in minutes."""
        try:
            word_count = len(content.split())
            
            # Base productivity (words per hour)
            base_productivity = translator.productivity_score or 300
            
            # Adjust for domain expertise
            domain_multiplier = 1.2 if domain in translator.specializations else 0.8
            
            # Adjust for content complexity
            complexity = await self._calculate_content_complexity(content)
            complexity_multiplier = 1.0 - (complexity * 0.3)
            
            adjusted_productivity = base_productivity * domain_multiplier * complexity_multiplier
            
            # Convert to minutes
            estimated_hours = word_count / adjusted_productivity
            estimated_minutes = estimated_hours * 60
            
            return max(15, estimated_minutes)  # Minimum 15 minutes
            
        except Exception as e:
            logger.error(f"Error estimating translation time: {e}")
            return 60  # Default 1 hour
    
    async def _get_translator_profile(
        self,
        translator_id: int,
        db: Session
    ) -> Optional[TranslatorProfile]:
        """Get or create translator profile."""
        try:
            if translator_id in self.translator_profiles:
                return self.translator_profiles[translator_id]
            
            # Get translator from database
            user = db.query(User).filter(User.id == translator_id).first()
            if not user:
                return None
            
            # Get translation history for performance metrics
            translations = db.query(ContentTranslation).filter(
                ContentTranslation.translator_id == translator_id
            ).limit(50).all()
            
            # Calculate performance metrics
            quality_scores = [t.quality_score for t in translations if t.quality_score]
            avg_quality = np.mean(quality_scores) if quality_scores else 0.7
            
            # Estimate expertise level based on translation count and quality
            translation_count = len(translations)
            if translation_count >= 100 and avg_quality >= 0.9:
                expertise = TranslatorExpertise.EXPERT
            elif translation_count >= 50 and avg_quality >= 0.8:
                expertise = TranslatorExpertise.ADVANCED
            elif translation_count >= 20 and avg_quality >= 0.7:
                expertise = TranslatorExpertise.INTERMEDIATE
            else:
                expertise = TranslatorExpertise.BEGINNER
            
            # Create profile
            profile = TranslatorProfile(
                user_id=translator_id,
                expertise_level=expertise,
                specializations=["technical"],  # Default, would be from user profile
                languages={"ur": "native", "en": "advanced"},  # Default
                performance_metrics={
                    "avg_quality": avg_quality,
                    "translation_count": translation_count
                },
                availability={},
                quality_score=avg_quality,
                productivity_score=300  # Default words per hour
            )
            
            self.translator_profiles[translator_id] = profile
            return profile
            
        except Exception as e:
            logger.error(f"Error getting translator profile: {e}")
            return None
    
    async def optimize_translator_assignment(
        self,
        content: str,
        domain: str,
        target_language: str,
        deadline: Optional[datetime],
        db: Session
    ) -> List[Dict[str, Any]]:
        """Optimize translator assignment based on multiple factors."""
        try:
            # Get available translators
            available_translators = db.query(User).filter(
                User.is_active == True
            ).limit(20).all()  # In practice, filter by translator role
            
            assignments = []
            
            for translator_user in available_translators:
                # Get translator profile
                profile = await self._get_translator_profile(translator_user.id, db)
                if not profile:
                    continue
                
                # Predict quality
                quality_prediction = await self.predict_translation_quality(
                    content, translator_user.id, domain, target_language, db
                )
                
                # Calculate assignment score
                score = await self._calculate_assignment_score(
                    profile, quality_prediction, deadline, domain, target_language
                )
                
                assignments.append({
                    "translator_id": translator_user.id,
                    "translator_name": translator_user.full_name or translator_user.username,
                    "expertise_level": profile.expertise_level.value,
                    "predicted_quality": quality_prediction.predicted_score,
                    "estimated_time": quality_prediction.estimated_time,
                    "assignment_score": score,
                    "specializations": profile.specializations,
                    "recommendations": quality_prediction.recommendations
                })
            
            # Sort by assignment score
            assignments.sort(key=lambda x: x["assignment_score"], reverse=True)
            
            return assignments[:5]  # Top 5 candidates
            
        except Exception as e:
            logger.error(f"Error optimizing translator assignment: {e}")
            return []
    
    async def _calculate_assignment_score(
        self,
        profile: TranslatorProfile,
        quality_prediction: QualityPrediction,
        deadline: Optional[datetime],
        domain: str,
        target_language: str
    ) -> float:
        """Calculate assignment score for translator."""
        try:
            score = 0.0
            
            # Quality score (40% weight)
            score += quality_prediction.predicted_score * 0.4
            
            # Domain expertise (25% weight)
            domain_match = 1.0 if domain in profile.specializations else 0.5
            score += domain_match * 0.25
            
            # Language proficiency (20% weight)
            lang_proficiency = profile.languages.get(target_language, "intermediate")
            proficiency_scores = {
                "beginner": 0.3,
                "intermediate": 0.6,
                "advanced": 0.8,
                "native": 1.0
            }
            score += proficiency_scores.get(lang_proficiency, 0.6) * 0.2
            
            # Availability/deadline factor (15% weight)
            if deadline:
                time_available = (deadline - datetime.utcnow()).total_seconds() / 3600  # hours
                time_needed = quality_prediction.estimated_time / 60  # convert to hours
                time_factor = min(1.0, time_available / max(time_needed, 1))
                score += time_factor * 0.15
            else:
                score += 0.15  # No deadline pressure
            
            return min(1.0, score)
            
        except Exception as e:
            logger.error(f"Error calculating assignment score: {e}")
            return 0.5
    
    async def check_terminology_consistency(
        self,
        source_text: str,
        translated_text: str,
        source_language: str,
        target_language: str,
        domain: str,
        db: Session
    ) -> Dict[str, Any]:
        """Check terminology consistency in translation."""
        try:
            # Get domain terminology
            terminology_entries = db.query(TerminologyGlossary).filter(
                and_(
                    TerminologyGlossary.source_language == source_language,
                    TerminologyGlossary.target_language == target_language,
                    TerminologyGlossary.domain == domain,
                    TerminologyGlossary.approved == True
                )
            ).all()
            
            terminology_dict = {
                entry.term.lower(): entry.translation.lower()
                for entry in terminology_entries
            }
            
            # Check consistency
            inconsistencies = []
            suggestions = []
            
            source_words = source_text.lower().split()
            translated_words = translated_text.lower().split()
            
            for source_word in source_words:
                if source_word in terminology_dict:
                    expected_translation = terminology_dict[source_word]
                    if expected_translation not in translated_text.lower():
                        inconsistencies.append({
                            "term": source_word,
                            "expected": expected_translation,
                            "context": f"Term '{source_word}' should be translated as '{expected_translation}'"
                        })
                        
                        suggestions.append(f"Use '{expected_translation}' for '{source_word}'")
            
            consistency_score = 1.0 - (len(inconsistencies) / max(len([w for w in source_words if w in terminology_dict]), 1))
            
            return {
                "consistency_score": consistency_score,
                "inconsistencies": inconsistencies,
                "suggestions": suggestions,
                "terminology_coverage": len([w for w in source_words if w in terminology_dict]) / max(len(source_words), 1)
            }
            
        except Exception as e:
            logger.error(f"Error checking terminology consistency: {e}")
            return {
                "consistency_score": 0.5,
                "inconsistencies": [],
                "suggestions": [],
                "terminology_coverage": 0.0
            }
    
    async def generate_productivity_dashboard(
        self,
        translator_id: Optional[int] = None,
        time_period: str = "week",
        db: Session = None
    ) -> Dict[str, Any]:
        """Generate productivity dashboard data."""
        try:
            # Calculate time range
            end_date = datetime.utcnow()
            if time_period == "week":
                start_date = end_date - timedelta(weeks=1)
            elif time_period == "month":
                start_date = end_date - timedelta(days=30)
            elif time_period == "year":
                start_date = end_date - timedelta(days=365)
            else:
                start_date = end_date - timedelta(weeks=1)
            
            # Base query
            query = db.query(ContentTranslation).filter(
                ContentTranslation.created_at >= start_date
            )
            
            if translator_id:
                query = query.filter(ContentTranslation.translator_id == translator_id)
            
            translations = query.all()
            
            # Calculate metrics
            total_translations = len(translations)
            completed_translations = len([t for t in translations if t.translation_status == TranslationStatus.PUBLISHED])
            avg_quality = np.mean([t.quality_score for t in translations if t.quality_score]) if translations else 0
            
            # Calculate productivity by translator
            translator_stats = defaultdict(lambda: {"count": 0, "quality": [], "time": []})
            
            for translation in translations:
                if translation.translator_id:
                    translator_stats[translation.translator_id]["count"] += 1
                    if translation.quality_score:
                        translator_stats[translation.translator_id]["quality"].append(translation.quality_score)
            
            # Top performers
            top_performers = []
            for translator_id, stats in translator_stats.items():
                if stats["count"] >= 3:  # Minimum translations for ranking
                    avg_translator_quality = np.mean(stats["quality"]) if stats["quality"] else 0
                    top_performers.append({
                        "translator_id": translator_id,
                        "translation_count": stats["count"],
                        "avg_quality": avg_translator_quality,
                        "productivity_score": stats["count"] * avg_translator_quality
                    })
            
            top_performers.sort(key=lambda x: x["productivity_score"], reverse=True)
            
            return {
                "period": time_period,
                "start_date": start_date.isoformat(),
                "end_date": end_date.isoformat(),
                "total_translations": total_translations,
                "completed_translations": completed_translations,
                "completion_rate": completed_translations / max(total_translations, 1),
                "average_quality": avg_quality,
                "top_performers": top_performers[:10],
                "workflow_efficiency": {
                    "avg_time_to_complete": 0,  # Would calculate from actual data
                    "bottlenecks": [],
                    "optimization_opportunities": len(self.optimization_rules)
                }
            }
            
        except Exception as e:
            logger.error(f"Error generating productivity dashboard: {e}")
            return {}


# Global workflow optimizer instance
workflow_optimizer = TranslationWorkflowOptimizer()