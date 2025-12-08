"""User Acceptance Testing framework for multilingual system."""

import logging
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
from enum import Enum
from dataclasses import dataclass, asdict
import json
import uuid

from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func
from pydantic import BaseModel

logger = logging.getLogger(__name__)


class TestScenarioType(str, Enum):
    """Types of UAT test scenarios."""
    LANGUAGE_SWITCHING = "language_switching"
    CONTENT_TRANSLATION = "content_translation"
    RTL_LAYOUT = "rtl_layout"
    CULTURAL_APPROPRIATENESS = "cultural_appropriateness"
    COMMUNITY_FEATURES = "community_features"
    SEARCH_FUNCTIONALITY = "search_functionality"
    PERFORMANCE = "performance"
    ACCESSIBILITY = "accessibility"


class TestStatus(str, Enum):
    """Test execution status."""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    BLOCKED = "blocked"


class FeedbackRating(int, Enum):
    """Feedback rating scale."""
    VERY_POOR = 1
    POOR = 2
    FAIR = 3
    GOOD = 4
    EXCELLENT = 5


@dataclass
class TestScenario:
    """Represents a UAT test scenario."""
    id: str
    title: str
    description: str
    scenario_type: TestScenarioType
    steps: List[str]
    expected_outcome: str
    acceptance_criteria: List[str]
    priority: str  # high, medium, low
    estimated_duration: int  # minutes
    prerequisites: List[str]


@dataclass
class TestExecution:
    """Represents a test execution by a user."""
    id: str
    scenario_id: str
    tester_id: int
    status: TestStatus
    started_at: datetime
    completed_at: Optional[datetime]
    actual_outcome: Optional[str]
    issues_found: List[str]
    feedback_rating: Optional[FeedbackRating]
    feedback_comments: Optional[str]
    screenshots: List[str]
    metadata: Dict[str, Any]


class UATestingFramework:
    """User Acceptance Testing framework for multilingual features."""
    
    def __init__(self):
        self.test_scenarios = self._initialize_test_scenarios()
        self.test_executions = []
        self.tester_profiles = {}
    
    def _initialize_test_scenarios(self) -> List[TestScenario]:
        """Initialize predefined test scenarios."""
        scenarios = [
            # Language Switching Scenarios
            TestScenario(
                id="LS001",
                title="Basic Language Switching",
                description="Test switching from English to Urdu and back",
                scenario_type=TestScenarioType.LANGUAGE_SWITCHING,
                steps=[
                    "Navigate to the homepage",
                    "Click on language selector",
                    "Select Urdu from dropdown",
                    "Verify interface changes to Urdu",
                    "Switch back to English",
                    "Verify interface returns to English"
                ],
                expected_outcome="Language switches smoothly without errors",
                acceptance_criteria=[
                    "Language switch completes within 1 second",
                    "All UI elements are translated correctly",
                    "No layout breaks occur",
                    "User preferences are saved"
                ],
                priority="high",
                estimated_duration=5,
                prerequisites=["User account created", "Browser supports Urdu fonts"]
            ),   
         
            TestScenario(
                id="LS002",
                title="Language Persistence Across Sessions",
                description="Test that language preference persists after logout/login",
                scenario_type=TestScenarioType.LANGUAGE_SWITCHING,
                steps=[
                    "Login to account",
                    "Switch language to Urdu",
                    "Logout from account",
                    "Login again",
                    "Verify language is still Urdu"
                ],
                expected_outcome="Language preference persists across sessions",
                acceptance_criteria=[
                    "Language preference is saved to user profile",
                    "Interface loads in Urdu after re-login",
                    "No additional language selection required"
                ],
                priority="medium",
                estimated_duration=3,
                prerequisites=["User account with login credentials"]
            ),
            
            # Content Translation Scenarios
            TestScenario(
                id="CT001",
                title="Chapter Content Translation",
                description="Test reading chapter content in Urdu",
                scenario_type=TestScenarioType.CONTENT_TRANSLATION,
                steps=[
                    "Navigate to a chapter page",
                    "Switch language to Urdu",
                    "Read the translated content",
                    "Verify translation quality and accuracy",
                    "Check for any untranslated text"
                ],
                expected_outcome="Chapter content is fully translated and readable",
                acceptance_criteria=[
                    "All text content is translated to Urdu",
                    "Translation maintains original meaning",
                    "Technical terms are appropriately translated",
                    "No English text remains visible"
                ],
                priority="high",
                estimated_duration=10,
                prerequisites=["Chapter with Urdu translation available"]
            ),
            
            TestScenario(
                id="CT002",
                title="Side-by-Side Content Comparison",
                description="Test bilingual content viewing feature",
                scenario_type=TestScenarioType.CONTENT_TRANSLATION,
                steps=[
                    "Open a chapter page",
                    "Enable bilingual view mode",
                    "Verify English and Urdu content side by side",
                    "Test synchronized scrolling",
                    "Check paragraph alignment"
                ],
                expected_outcome="Bilingual view works correctly with proper alignment",
                acceptance_criteria=[
                    "Both languages display simultaneously",
                    "Scrolling is synchronized between versions",
                    "Paragraphs are properly aligned",
                    "Layout is responsive on different screen sizes"
                ],
                priority="medium",
                estimated_duration=8,
                prerequisites=["Content available in both languages"]
            ),
            
            # RTL Layout Scenarios
            TestScenario(
                id="RTL001",
                title="RTL Layout Rendering",
                description="Test Right-to-Left layout for Urdu content",
                scenario_type=TestScenarioType.RTL_LAYOUT,
                steps=[
                    "Switch to Urdu language",
                    "Navigate through different pages",
                    "Verify text alignment is right-to-left",
                    "Check navigation menu alignment",
                    "Test form input alignment"
                ],
                expected_outcome="All UI elements properly align for RTL reading",
                acceptance_criteria=[
                    "Text flows from right to left",
                    "Navigation elements are mirrored",
                    "Form inputs align correctly",
                    "Icons and buttons are positioned appropriately"
                ],
                priority="high",
                estimated_duration=7,
                prerequisites=["Urdu language selected"]
            ),
            
            # Cultural Appropriateness Scenarios
            TestScenario(
                id="CA001",
                title="Cultural Context Validation",
                description="Test cultural appropriateness of examples and references",
                scenario_type=TestScenarioType.CULTURAL_APPROPRIATENESS,
                steps=[
                    "Review translated content for cultural examples",
                    "Check currency and date formats",
                    "Verify local references (cities, companies)",
                    "Assess cultural sensitivity of content",
                    "Identify any inappropriate cultural references"
                ],
                expected_outcome="Content is culturally appropriate for Urdu speakers",
                acceptance_criteria=[
                    "Examples use local context (Pakistani cities, companies)",
                    "Currency is displayed in PKR where appropriate",
                    "Date formats follow local conventions",
                    "No culturally insensitive content present"
                ],
                priority="high",
                estimated_duration=15,
                prerequisites=["Native Urdu speaker with cultural knowledge"]
            ),
            
            # Community Features Scenarios
            TestScenario(
                id="CF001",
                title="Translation Contribution",
                description="Test submitting translation contributions",
                scenario_type=TestScenarioType.COMMUNITY_FEATURES,
                steps=[
                    "Navigate to community translation page",
                    "Select content to translate",
                    "Submit a translation contribution",
                    "Add notes for reviewers",
                    "Verify submission confirmation"
                ],
                expected_outcome="Translation contribution is submitted successfully",
                acceptance_criteria=[
                    "Translation form accepts Urdu text input",
                    "Submission process completes without errors",
                    "Confirmation message is displayed",
                    "Contribution appears in user profile"
                ],
                priority="medium",
                estimated_duration=12,
                prerequisites=["User account with contributor privileges"]
            ),
            
            # Search Functionality Scenarios
            TestScenario(
                id="SF001",
                title="Multilingual Search",
                description="Test search functionality in Urdu",
                scenario_type=TestScenarioType.SEARCH_FUNCTIONALITY,
                steps=[
                    "Switch to Urdu language",
                    "Enter search query in Urdu",
                    "Review search results",
                    "Test search suggestions",
                    "Verify result relevance"
                ],
                expected_outcome="Search works effectively in Urdu language",
                acceptance_criteria=[
                    "Search accepts Urdu text input",
                    "Results are relevant to query",
                    "Search suggestions appear in Urdu",
                    "Results load within 500ms"
                ],
                priority="medium",
                estimated_duration=8,
                prerequisites=["Searchable Urdu content available"]
            ),
            
            # Performance Scenarios
            TestScenario(
                id="PF001",
                title="Translation Loading Performance",
                description="Test performance of translation loading",
                scenario_type=TestScenarioType.PERFORMANCE,
                steps=[
                    "Measure time to switch languages",
                    "Test translation loading on slow connection",
                    "Monitor font loading time",
                    "Check memory usage during language switch",
                    "Test with multiple tabs open"
                ],
                expected_outcome="Performance meets specified requirements",
                acceptance_criteria=[
                    "Language switch completes within 1 second",
                    "Translation loads within 2 seconds",
                    "Font loading completes within 1 second",
                    "No significant memory leaks detected"
                ],
                priority="high",
                estimated_duration=10,
                prerequisites=["Performance monitoring tools available"]
            ),
            
            # Accessibility Scenarios
            TestScenario(
                id="AC001",
                title="Screen Reader Compatibility",
                description="Test accessibility with screen readers",
                scenario_type=TestScenarioType.ACCESSIBILITY,
                steps=[
                    "Enable screen reader software",
                    "Navigate through Urdu content",
                    "Test language announcements",
                    "Verify proper heading structure",
                    "Check ARIA labels and descriptions"
                ],
                expected_outcome="Content is accessible via screen readers",
                acceptance_criteria=[
                    "Screen reader announces language changes",
                    "Urdu text is read correctly",
                    "Navigation structure is clear",
                    "All interactive elements are accessible"
                ],
                priority="medium",
                estimated_duration=15,
                prerequisites=["Screen reader software installed"]
            )
        ]
        
        return scenarios
    
    def get_test_scenarios(
        self,
        scenario_type: Optional[TestScenarioType] = None,
        priority: Optional[str] = None
    ) -> List[TestScenario]:
        """Get test scenarios with optional filtering."""
        scenarios = self.test_scenarios
        
        if scenario_type:
            scenarios = [s for s in scenarios if s.scenario_type == scenario_type]
        
        if priority:
            scenarios = [s for s in scenarios if s.priority == priority]
        
        return scenarios
    
    def create_test_execution(
        self,
        scenario_id: str,
        tester_id: int,
        metadata: Optional[Dict[str, Any]] = None
    ) -> TestExecution:
        """Create a new test execution."""
        execution = TestExecution(
            id=str(uuid.uuid4()),
            scenario_id=scenario_id,
            tester_id=tester_id,
            status=TestStatus.PENDING,
            started_at=datetime.utcnow(),
            completed_at=None,
            actual_outcome=None,
            issues_found=[],
            feedback_rating=None,
            feedback_comments=None,
            screenshots=[],
            metadata=metadata or {}
        )
        
        self.test_executions.append(execution)
        return execution
    
    def start_test_execution(self, execution_id: str) -> bool:
        """Start a test execution."""
        for execution in self.test_executions:
            if execution.id == execution_id:
                execution.status = TestStatus.IN_PROGRESS
                execution.started_at = datetime.utcnow()
                return True
        return False
    
    def complete_test_execution(
        self,
        execution_id: str,
        actual_outcome: str,
        issues_found: List[str],
        feedback_rating: FeedbackRating,
        feedback_comments: str,
        screenshots: Optional[List[str]] = None
    ) -> bool:
        """Complete a test execution with results."""
        for execution in self.test_executions:
            if execution.id == execution_id:
                execution.status = TestStatus.COMPLETED
                execution.completed_at = datetime.utcnow()
                execution.actual_outcome = actual_outcome
                execution.issues_found = issues_found
                execution.feedback_rating = feedback_rating
                execution.feedback_comments = feedback_comments
                execution.screenshots = screenshots or []
                return True
        return False
    
    def get_test_results_summary(self) -> Dict[str, Any]:
        """Get summary of test results."""
        total_tests = len(self.test_executions)
        completed_tests = len([e for e in self.test_executions if e.status == TestStatus.COMPLETED])
        failed_tests = len([e for e in self.test_executions if e.status == TestStatus.FAILED])
        
        # Calculate average rating
        ratings = [e.feedback_rating.value for e in self.test_executions 
                  if e.feedback_rating is not None]
        avg_rating = sum(ratings) / len(ratings) if ratings else 0
        
        # Group by scenario type
        scenario_results = {}
        for execution in self.test_executions:
            scenario = next((s for s in self.test_scenarios if s.id == execution.scenario_id), None)
            if scenario:
                scenario_type = scenario.scenario_type.value
                if scenario_type not in scenario_results:
                    scenario_results[scenario_type] = {
                        "total": 0,
                        "completed": 0,
                        "failed": 0,
                        "avg_rating": 0
                    }
                
                scenario_results[scenario_type]["total"] += 1
                if execution.status == TestStatus.COMPLETED:
                    scenario_results[scenario_type]["completed"] += 1
                elif execution.status == TestStatus.FAILED:
                    scenario_results[scenario_type]["failed"] += 1
        
        # Calculate average ratings by scenario type
        for scenario_type in scenario_results:
            type_ratings = [
                e.feedback_rating.value for e in self.test_executions
                if e.feedback_rating is not None and
                any(s.id == e.scenario_id and s.scenario_type.value == scenario_type 
                    for s in self.test_scenarios)
            ]
            scenario_results[scenario_type]["avg_rating"] = (
                sum(type_ratings) / len(type_ratings) if type_ratings else 0
            )
        
        return {
            "total_tests": total_tests,
            "completed_tests": completed_tests,
            "failed_tests": failed_tests,
            "success_rate": (completed_tests / max(total_tests, 1)) * 100,
            "average_rating": avg_rating,
            "scenario_results": scenario_results,
            "generated_at": datetime.utcnow().isoformat()
        }
    
    def get_issues_report(self) -> Dict[str, Any]:
        """Get report of all issues found during testing."""
        all_issues = []
        
        for execution in self.test_executions:
            if execution.issues_found:
                scenario = next((s for s in self.test_scenarios if s.id == execution.scenario_id), None)
                for issue in execution.issues_found:
                    all_issues.append({
                        "execution_id": execution.id,
                        "scenario_id": execution.scenario_id,
                        "scenario_title": scenario.title if scenario else "Unknown",
                        "scenario_type": scenario.scenario_type.value if scenario else "unknown",
                        "tester_id": execution.tester_id,
                        "issue_description": issue,
                        "reported_at": execution.completed_at.isoformat() if execution.completed_at else None,
                        "priority": scenario.priority if scenario else "unknown"
                    })
        
        # Group issues by type and priority
        issues_by_type = {}
        issues_by_priority = {"high": 0, "medium": 0, "low": 0}
        
        for issue in all_issues:
            issue_type = issue["scenario_type"]
            if issue_type not in issues_by_type:
                issues_by_type[issue_type] = []
            issues_by_type[issue_type].append(issue)
            
            priority = issue["priority"]
            if priority in issues_by_priority:
                issues_by_priority[priority] += 1
        
        return {
            "total_issues": len(all_issues),
            "issues_by_type": issues_by_type,
            "issues_by_priority": issues_by_priority,
            "all_issues": all_issues,
            "generated_at": datetime.utcnow().isoformat()
        }
    
    def register_tester(
        self,
        tester_id: int,
        profile: Dict[str, Any]
    ) -> None:
        """Register a tester with their profile information."""
        self.tester_profiles[tester_id] = {
            **profile,
            "registered_at": datetime.utcnow().isoformat()
        }
    
    def get_tester_assignments(self, tester_id: int) -> List[Dict[str, Any]]:
        """Get test assignments for a specific tester."""
        assignments = []
        
        for execution in self.test_executions:
            if execution.tester_id == tester_id:
                scenario = next((s for s in self.test_scenarios if s.id == execution.scenario_id), None)
                if scenario:
                    assignments.append({
                        "execution_id": execution.id,
                        "scenario": asdict(scenario),
                        "status": execution.status.value,
                        "started_at": execution.started_at.isoformat(),
                        "completed_at": execution.completed_at.isoformat() if execution.completed_at else None
                    })
        
        return assignments


# Global UAT framework instance
uat_framework = UATestingFramework()