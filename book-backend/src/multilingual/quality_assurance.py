"""Translation Quality Assurance System."""

import logging
import re
import asyncio
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from enum import Enum
import json
import statistics

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func

from app.models.multilingual import (
    ContentTranslation, TranslationMemory, TerminologyGlossary,
    TranslationStatus, Language
)
from app.models.database import User

logger = logging.getLogger(__name__)


class QualityMetric(str, Enum):
    """Quality assessment metrics."""
    ACCURACY = "accuracy"
    FLUENCY = "fluency"
    CONSISTENCY = "consistency"
    COMPLETENESS = "completeness"
    TERMINOLOGY = "terminology"
    FORMATTING = "formatting"
    CULTURAL_APPROPRIATENESS = "cultural_appropriateness"


class QualityLevel(str, Enum):
    """Quality levels."""
    EXCELLENT = "excellent"  # 90-100%
    GOOD = "good"           # 70-89%
    FAIR = "fair"           # 50-69%
    POOR = "poor"           # 0-49%


class QualityIssue:
    """Quality issue representation."""
    
    def __init__(
        self,
        issue_type: str,
        severity: str,
        description: str,
        location: Optional[str] = None,
        suggestion: Optional[str] = None,
        auto_fixable: bool = False
    ):
        self.issue_type = issue_type
        self.severity = severity  # critical, major, minor, info
        self.description = description
        self.location = location
        self.suggestion = suggestion
        self.auto_fixable = auto_fixable
        self.timestamp = datetime.utcnow()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "issue_type": self.issue_type,
            "severity": self.severity,
            "description": self.description,
            "location": self.location,
            "suggestion": self.suggestion,
            "auto_fixable": self.auto_fixable,
            "timestamp": self.timestamp.isoformat()
        }


class TranslationQualityAssurance:
    """Translation Quality Assurance System."""
    
    def __init__(self):
        self.quality_thresholds = {
            QualityLevel.EXCELLENT: 0.9,
            QualityLevel.GOOD: 0.7,
            QualityLevel.FAIR: 0.5,
            QualityLevel.POOR: 0.0
        }
        
        self.metric_weights = {
            QualityMetric.ACCURACY: 0.25,
            QualityMetric.FLUENCY: 0.20,
            QualityMetric.CONSISTENCY: 0.15,
            QualityMetric.COMPLETENESS: 0.15,
            QualityMetric.TERMINOLOGY: 0.15,
            QualityMetric.FORMATTING: 0.05,
            QualityMetric.CULTURAL_APPROPRIATENESS: 0.05
        }
        
        # Language-specific patterns and rules
        self.language_rules = {
            "ur": {
                "rtl_chars": r'[\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]',
                "punctuation": r'[۔؍؎؏؞؟٪٫٬]',
                "numbers": r'[۰-۹]',
                "common_errors": [
                    (r'\bthe\b', 'Untranslated English article'),
                    (r'\band\b', 'Untranslated English conjunction'),
                    (r'\bof\b', 'Untranslated English preposition')
                ]
            },
            "ar": {
                "rtl_chars": r'[\u0600-\u06FF\u0750-\u077F\u08A0-\u08FF\uFB50-\uFDFF\uFE70-\uFEFF]',
                "punctuation": r'[؍؎؏؞؟٪٫٬]',
                "numbers": r'[٠-٩]'
            }
        }
    
    async def assess_translation_quality(
        self,
        translation_id: int,
        db: Session,
        detailed_analysis: bool = True
    ) -> Dict[str, Any]:
        """Assess overall translation quality."""
        try:
            translation = db.query(ContentTranslation).filter(
                ContentTranslation.id == translation_id
            ).first()
            
            if not translation:
                raise ValueError(f"Translation {translation_id} not found")
            
            # Get source content (would need to be implemented)
            source_content = await self._get_source_content(translation, db)
            
            if not source_content:
                logger.warning(f"No source content found for translation {translation_id}")
                return {"error": "Source content not available"}
            
            # Perform quality assessments
            quality_scores = {}
            issues = []
            
            # Accuracy assessment
            accuracy_result = await self._assess_accuracy(
                source_content, translation.content or "", 
                translation.language_code, db
            )
            quality_scores[QualityMetric.ACCURACY] = accuracy_result["score"]
            issues.extend(accuracy_result["issues"])
            
            # Fluency assessment
            fluency_result = await self._assess_fluency(
                translation.content or "", translation.language_code
            )
            quality_scores[QualityMetric.FLUENCY] = fluency_result["score"]
            issues.extend(fluency_result["issues"])
            
            # Consistency assessment
            consistency_result = await self._assess_consistency(
                translation, db
            )
            quality_scores[QualityMetric.CONSISTENCY] = consistency_result["score"]
            issues.extend(consistency_result["issues"])
            
            # Completeness assessment
            completeness_result = await self._assess_completeness(
                source_content, translation.content or ""
            )
            quality_scores[QualityMetric.COMPLETENESS] = completeness_result["score"]
            issues.extend(completeness_result["issues"])
            
            # Terminology assessment
            terminology_result = await self._assess_terminology(
                translation.content or "", translation.language_code, db
            )
            quality_scores[QualityMetric.TERMINOLOGY] = terminology_result["score"]
            issues.extend(terminology_result["issues"])
            
            # Formatting assessment
            formatting_result = await self._assess_formatting(
                source_content, translation.content or "", translation.language_code
            )
            quality_scores[QualityMetric.FORMATTING] = formatting_result["score"]
            issues.extend(formatting_result["issues"])
            
            # Cultural appropriateness assessment
            cultural_result = await self._assess_cultural_appropriateness(
                translation.content or "", translation.language_code
            )
            quality_scores[QualityMetric.CULTURAL_APPROPRIATENESS] = cultural_result["score"]
            issues.extend(cultural_result["issues"])
            
            # Calculate overall score
            overall_score = sum(
                score * self.metric_weights[metric]
                for metric, score in quality_scores.items()
            )
            
            # Determine quality level
            quality_level = self._determine_quality_level(overall_score)
            
            # Generate recommendations
            recommendations = await self._generate_recommendations(
                quality_scores, issues, translation.language_code
            )
            
            # Update translation quality score
            translation.quality_score = overall_score
            translation.updated_at = datetime.utcnow()
            
            # Update metadata with quality assessment
            metadata = translation.metadata or {}
            metadata.update({
                "quality_assessment": {
                    "overall_score": overall_score,
                    "quality_level": quality_level.value,
                    "metric_scores": {k.value: v for k, v in quality_scores.items()},
                    "assessment_date": datetime.utcnow().isoformat(),
                    "issues_count": len(issues),
                    "critical_issues": len([i for i in issues if i.severity == "critical"]),
                    "recommendations_count": len(recommendations)
                }
            })
            translation.metadata = metadata
            
            db.commit()
            
            result = {
                "translation_id": translation_id,
                "overall_score": overall_score,
                "quality_level": quality_level.value,
                "metric_scores": quality_scores,
                "recommendations": recommendations
            }
            
            if detailed_analysis:
                result.update({
                    "issues": [issue.to_dict() for issue in issues],
                    "detailed_metrics": {
                        "accuracy": accuracy_result,
                        "fluency": fluency_result,
                        "consistency": consistency_result,
                        "completeness": completeness_result,
                        "terminology": terminology_result,
                        "formatting": formatting_result,
                        "cultural": cultural_result
                    }
                })
            
            return result
            
        except Exception as e:
            logger.error(f"Error assessing translation quality: {e}")
            return {"error": str(e)}
    
    async def _assess_accuracy(
        self,
        source_text: str,
        translated_text: str,
        target_language: str,
        db: Session
    ) -> Dict[str, Any]:
        """Assess translation accuracy."""
        issues = []
        score = 0.8  # Base score
        
        try:
            # Check for untranslated content
            if source_text.strip() == translated_text.strip():
                issues.append(QualityIssue(
                    "untranslated_content",
                    "critical",
                    "Content appears to be untranslated",
                    auto_fixable=False
                ))
                score -= 0.5
            
            # Check for missing content
            if not translated_text.strip():
                issues.append(QualityIssue(
                    "missing_translation",
                    "critical",
                    "Translation is empty",
                    auto_fixable=False
                ))
                score -= 0.8
            
            # Check for language-specific issues
            if target_language in self.language_rules:
                rules = self.language_rules[target_language]
                
                # Check for proper RTL characters
                if "rtl_chars" in rules:
                    rtl_pattern = rules["rtl_chars"]
                    if not re.search(rtl_pattern, translated_text):
                        issues.append(QualityIssue(
                            "missing_rtl_chars",
                            "major",
                            f"No {target_language.upper()} characters found in translation",
                            auto_fixable=False
                        ))
                        score -= 0.3
                
                # Check for common translation errors
                if "common_errors" in rules:
                    for pattern, description in rules["common_errors"]:
                        if re.search(pattern, translated_text, re.IGNORECASE):
                            issues.append(QualityIssue(
                                "common_error",
                                "minor",
                                description,
                                location=pattern,
                                auto_fixable=True
                            ))
                            score -= 0.1
            
            # Check for placeholder preservation
            source_placeholders = re.findall(r'\{[^}]+\}', source_text)
            translated_placeholders = re.findall(r'\{[^}]+\}', translated_text)
            
            if set(source_placeholders) != set(translated_placeholders):
                issues.append(QualityIssue(
                    "placeholder_mismatch",
                    "major",
                    "Placeholders don't match between source and translation",
                    suggestion="Ensure all placeholders are preserved",
                    auto_fixable=True
                ))
                score -= 0.2
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "source_length": len(source_text),
                    "translated_length": len(translated_text),
                    "length_ratio": len(translated_text) / max(len(source_text), 1)
                }
            }
            
        except Exception as e:
            logger.error(f"Error in accuracy assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_fluency(
        self,
        translated_text: str,
        target_language: str
    ) -> Dict[str, Any]:
        """Assess translation fluency."""
        issues = []
        score = 0.8  # Base score
        
        try:
            # Check for basic fluency issues
            
            # Repeated words
            words = translated_text.lower().split()
            word_counts = {}
            for word in words:
                word_counts[word] = word_counts.get(word, 0) + 1
            
            repeated_words = [word for word, count in word_counts.items() if count > 3 and len(word) > 3]
            if repeated_words:
                issues.append(QualityIssue(
                    "repeated_words",
                    "minor",
                    f"Repeated words detected: {', '.join(repeated_words[:3])}",
                    auto_fixable=False
                ))
                score -= 0.1
            
            # Sentence structure
            sentences = re.split(r'[.!?۔]', translated_text)
            avg_sentence_length = sum(len(s.split()) for s in sentences) / max(len(sentences), 1)
            
            if avg_sentence_length > 30:
                issues.append(QualityIssue(
                    "long_sentences",
                    "minor",
                    "Sentences are too long, may affect readability",
                    suggestion="Consider breaking long sentences",
                    auto_fixable=False
                ))
                score -= 0.1
            
            # Language-specific fluency checks
            if target_language == "ur":
                # Check for proper Urdu sentence structure
                if not re.search(r'[۔؟!]$', translated_text.strip()):
                    issues.append(QualityIssue(
                        "missing_urdu_punctuation",
                        "minor",
                        "Missing proper Urdu punctuation at the end",
                        suggestion="Add proper Urdu punctuation (۔ ؟ !)",
                        auto_fixable=True
                    ))
                    score -= 0.05
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "word_count": len(words),
                    "sentence_count": len(sentences),
                    "avg_sentence_length": avg_sentence_length,
                    "repeated_words_count": len(repeated_words)
                }
            }
            
        except Exception as e:
            logger.error(f"Error in fluency assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_consistency(
        self,
        translation: ContentTranslation,
        db: Session
    ) -> Dict[str, Any]:
        """Assess translation consistency."""
        issues = []
        score = 0.9  # Base score
        
        try:
            # Check consistency with translation memory
            similar_translations = db.query(ContentTranslation).filter(
                and_(
                    ContentTranslation.language_code == translation.language_code,
                    ContentTranslation.content_type == translation.content_type,
                    ContentTranslation.id != translation.id,
                    ContentTranslation.translation_status.in_([
                        TranslationStatus.APPROVED,
                        TranslationStatus.PUBLISHED
                    ])
                )
            ).limit(10).all()
            
            # Check terminology consistency
            if translation.content:
                # Get approved terminology for this language
                terminology = db.query(TerminologyGlossary).filter(
                    and_(
                        TerminologyGlossary.target_language == translation.language_code,
                        TerminologyGlossary.approved == True
                    )
                ).all()
                
                inconsistent_terms = []
                for term in terminology:
                    if term.term.lower() in translation.content.lower():
                        # Check if the approved translation is used
                        if term.translation not in translation.content:
                            inconsistent_terms.append((term.term, term.translation))
                
                if inconsistent_terms:
                    issues.append(QualityIssue(
                        "terminology_inconsistency",
                        "major",
                        f"Inconsistent terminology usage: {len(inconsistent_terms)} terms",
                        suggestion="Use approved terminology translations",
                        auto_fixable=True
                    ))
                    score -= 0.2
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "similar_translations_checked": len(similar_translations),
                    "terminology_checked": len(terminology) if 'terminology' in locals() else 0,
                    "inconsistent_terms": len(inconsistent_terms) if 'inconsistent_terms' in locals() else 0
                }
            }
            
        except Exception as e:
            logger.error(f"Error in consistency assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_completeness(
        self,
        source_text: str,
        translated_text: str
    ) -> Dict[str, Any]:
        """Assess translation completeness."""
        issues = []
        score = 1.0  # Start with perfect score
        
        try:
            # Check if translation is complete
            if not translated_text.strip():
                issues.append(QualityIssue(
                    "incomplete_translation",
                    "critical",
                    "Translation is empty",
                    auto_fixable=False
                ))
                return {"score": 0.0, "issues": issues}
            
            # Check length ratio (rough completeness indicator)
            length_ratio = len(translated_text) / max(len(source_text), 1)
            
            if length_ratio < 0.3:
                issues.append(QualityIssue(
                    "too_short",
                    "major",
                    "Translation appears too short compared to source",
                    auto_fixable=False
                ))
                score -= 0.4
            elif length_ratio > 3.0:
                issues.append(QualityIssue(
                    "too_long",
                    "minor",
                    "Translation appears much longer than source",
                    auto_fixable=False
                ))
                score -= 0.1
            
            # Check for incomplete sentences
            if translated_text.strip() and not re.search(r'[.!?۔؟]$', translated_text.strip()):
                issues.append(QualityIssue(
                    "incomplete_sentence",
                    "minor",
                    "Translation appears to end abruptly",
                    suggestion="Complete the sentence with proper punctuation",
                    auto_fixable=True
                ))
                score -= 0.1
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "length_ratio": length_ratio,
                    "source_length": len(source_text),
                    "translated_length": len(translated_text)
                }
            }
            
        except Exception as e:
            logger.error(f"Error in completeness assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_terminology(
        self,
        translated_text: str,
        target_language: str,
        db: Session
    ) -> Dict[str, Any]:
        """Assess terminology usage."""
        issues = []
        score = 0.9  # Base score
        
        try:
            # Get approved terminology
            terminology = db.query(TerminologyGlossary).filter(
                and_(
                    TerminologyGlossary.target_language == target_language,
                    TerminologyGlossary.approved == True
                )
            ).all()
            
            correct_usage = 0
            total_terms = 0
            
            for term in terminology:
                if term.term.lower() in translated_text.lower():
                    total_terms += 1
                    if term.translation in translated_text:
                        correct_usage += 1
                    else:
                        issues.append(QualityIssue(
                            "incorrect_terminology",
                            "major",
                            f"Term '{term.term}' should be translated as '{term.translation}'",
                            location=term.term,
                            suggestion=f"Use '{term.translation}' instead",
                            auto_fixable=True
                        ))
            
            if total_terms > 0:
                terminology_accuracy = correct_usage / total_terms
                score = terminology_accuracy
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "total_terms_found": total_terms,
                    "correct_usage": correct_usage,
                    "terminology_accuracy": correct_usage / max(total_terms, 1)
                }
            }
            
        except Exception as e:
            logger.error(f"Error in terminology assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_formatting(
        self,
        source_text: str,
        translated_text: str,
        target_language: str
    ) -> Dict[str, Any]:
        """Assess formatting preservation."""
        issues = []
        score = 0.9  # Base score
        
        try:
            # Check for preserved formatting elements
            formatting_elements = [
                (r'\*\*([^*]+)\*\*', 'bold'),
                (r'\*([^*]+)\*', 'italic'),
                (r'`([^`]+)`', 'code'),
                (r'\[([^\]]+)\]\([^)]+\)', 'link'),
                (r'#{1,6}\s', 'heading')
            ]
            
            for pattern, element_type in formatting_elements:
                source_matches = re.findall(pattern, source_text)
                translated_matches = re.findall(pattern, translated_text)
                
                if len(source_matches) != len(translated_matches):
                    issues.append(QualityIssue(
                        "formatting_mismatch",
                        "minor",
                        f"Formatting mismatch for {element_type} elements",
                        suggestion=f"Preserve {element_type} formatting",
                        auto_fixable=True
                    ))
                    score -= 0.1
            
            # Check line breaks preservation
            source_lines = source_text.count('\n')
            translated_lines = translated_text.count('\n')
            
            if abs(source_lines - translated_lines) > 2:
                issues.append(QualityIssue(
                    "line_break_mismatch",
                    "minor",
                    "Line break structure differs significantly",
                    suggestion="Preserve paragraph structure",
                    auto_fixable=True
                ))
                score -= 0.05
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "source_lines": source_lines,
                    "translated_lines": translated_lines,
                    "formatting_elements_checked": len(formatting_elements)
                }
            }
            
        except Exception as e:
            logger.error(f"Error in formatting assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    async def _assess_cultural_appropriateness(
        self,
        translated_text: str,
        target_language: str
    ) -> Dict[str, Any]:
        """Assess cultural appropriateness."""
        issues = []
        score = 0.9  # Base score
        
        try:
            # Language-specific cultural checks
            if target_language == "ur":
                # Check for culturally inappropriate content
                inappropriate_patterns = [
                    (r'\balcohol\b', 'alcohol reference'),
                    (r'\bpork\b', 'pork reference'),
                    (r'\bgambling\b', 'gambling reference')
                ]
                
                for pattern, issue_type in inappropriate_patterns:
                    if re.search(pattern, translated_text, re.IGNORECASE):
                        issues.append(QualityIssue(
                            "cultural_sensitivity",
                            "minor",
                            f"Potentially sensitive content: {issue_type}",
                            suggestion="Consider cultural context",
                            auto_fixable=False
                        ))
                        score -= 0.1
                
                # Check for proper honorifics usage
                if re.search(r'\b(sir|madam)\b', translated_text, re.IGNORECASE):
                    issues.append(QualityIssue(
                        "honorifics",
                        "info",
                        "Consider using appropriate Urdu honorifics",
                        suggestion="Use 'جناب' or 'صاحب' instead of 'sir'",
                        auto_fixable=True
                    ))
            
            return {
                "score": max(0.0, min(1.0, score)),
                "issues": issues,
                "details": {
                    "cultural_checks_performed": True,
                    "target_language": target_language
                }
            }
            
        except Exception as e:
            logger.error(f"Error in cultural assessment: {e}")
            return {"score": 0.0, "issues": [], "error": str(e)}
    
    def _determine_quality_level(self, score: float) -> QualityLevel:
        """Determine quality level based on score."""
        if score >= self.quality_thresholds[QualityLevel.EXCELLENT]:
            return QualityLevel.EXCELLENT
        elif score >= self.quality_thresholds[QualityLevel.GOOD]:
            return QualityLevel.GOOD
        elif score >= self.quality_thresholds[QualityLevel.FAIR]:
            return QualityLevel.FAIR
        else:
            return QualityLevel.POOR
    
    async def _generate_recommendations(
        self,
        quality_scores: Dict[QualityMetric, float],
        issues: List[QualityIssue],
        target_language: str
    ) -> List[str]:
        """Generate improvement recommendations."""
        recommendations = []
        
        # Score-based recommendations
        for metric, score in quality_scores.items():
            if score < 0.7:
                if metric == QualityMetric.ACCURACY:
                    recommendations.append("Review translation accuracy against source content")
                elif metric == QualityMetric.FLUENCY:
                    recommendations.append("Improve text fluency and natural language flow")
                elif metric == QualityMetric.CONSISTENCY:
                    recommendations.append("Ensure consistency with approved terminology")
                elif metric == QualityMetric.COMPLETENESS:
                    recommendations.append("Complete missing parts of the translation")
                elif metric == QualityMetric.TERMINOLOGY:
                    recommendations.append("Use approved technical terminology")
                elif metric == QualityMetric.FORMATTING:
                    recommendations.append("Preserve source formatting and structure")
                elif metric == QualityMetric.CULTURAL_APPROPRIATENESS:
                    recommendations.append("Review cultural appropriateness for target audience")
        
        # Issue-based recommendations
        critical_issues = [i for i in issues if i.severity == "critical"]
        if critical_issues:
            recommendations.append("Address critical issues before publication")
        
        major_issues = [i for i in issues if i.severity == "major"]
        if len(major_issues) > 3:
            recommendations.append("Review and fix major quality issues")
        
        # Auto-fixable issues
        auto_fixable = [i for i in issues if i.auto_fixable]
        if auto_fixable:
            recommendations.append(f"Apply automatic fixes for {len(auto_fixable)} issues")
        
        # Language-specific recommendations
        if target_language == "ur":
            recommendations.append("Verify proper Urdu grammar and sentence structure")
            recommendations.append("Ensure correct RTL text direction and formatting")
        
        return recommendations
    
    async def _get_source_content(
        self,
        translation: ContentTranslation,
        db: Session
    ) -> Optional[str]:
        """Get source content for comparison."""
        try:
            # This would implement logic to get the original content
            # For now, return a placeholder
            return f"Source content for {translation.content_type}:{translation.content_id}"
            
        except Exception as e:
            logger.error(f"Error getting source content: {e}")
            return None
    
    async def get_quality_statistics(
        self,
        language_code: Optional[str] = None,
        content_type: Optional[str] = None,
        date_range: Optional[Tuple[datetime, datetime]] = None,
        db: Session = None
    ) -> Dict[str, Any]:
        """Get quality statistics."""
        try:
            query = db.query(ContentTranslation).filter(
                ContentTranslation.quality_score.isnot(None)
            )
            
            if language_code:
                query = query.filter(ContentTranslation.language_code == language_code)
            
            if content_type:
                query = query.filter(ContentTranslation.content_type == content_type)
            
            if date_range:
                start_date, end_date = date_range
                query = query.filter(
                    and_(
                        ContentTranslation.updated_at >= start_date,
                        ContentTranslation.updated_at <= end_date
                    )
                )
            
            translations = query.all()
            
            if not translations:
                return {"message": "No translations found"}
            
            scores = [t.quality_score for t in translations if t.quality_score is not None]
            
            # Calculate statistics
            stats = {
                "total_translations": len(translations),
                "average_quality": statistics.mean(scores) if scores else 0,
                "median_quality": statistics.median(scores) if scores else 0,
                "min_quality": min(scores) if scores else 0,
                "max_quality": max(scores) if scores else 0,
                "quality_distribution": {
                    QualityLevel.EXCELLENT.value: len([s for s in scores if s >= 0.9]),
                    QualityLevel.GOOD.value: len([s for s in scores if 0.7 <= s < 0.9]),
                    QualityLevel.FAIR.value: len([s for s in scores if 0.5 <= s < 0.7]),
                    QualityLevel.POOR.value: len([s for s in scores if s < 0.5])
                }
            }
            
            return stats
            
        except Exception as e:
            logger.error(f"Error getting quality statistics: {e}")
            return {"error": str(e)}


# Global quality assurance instance
quality_assurance = TranslationQualityAssurance()