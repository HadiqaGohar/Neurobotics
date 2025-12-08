"""User Acceptance Testing API routes."""

import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Query, BackgroundTasks
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field

from app.core.dependencies import get_db, get_current_user
from app.models.database import User
from .uat_framework import (
    uat_framework, TestScenarioType, TestStatus, FeedbackRating
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/multilingual/uat", tags=["multilingual-uat"])


# Pydantic models for request/response
class TesterRegistrationRequest(BaseModel):
    native_language: str = Field(..., description="Tester's native language")
    language_proficiency: Dict[str, str] = Field(..., description="Language proficiency levels")
    technical_background: str = Field(..., description="Technical background level")
    device_info: Dict[str, str] = Field(..., description="Device and browser information")
    location: str = Field(..., description="Geographic location")
    availability: str = Field(..., description="Testing availability")


class TestExecutionRequest(BaseModel):
    scenario_id: str = Field(..., description="Test scenario ID")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata")


class TestCompletionRequest(BaseModel):
    actual_outcome: str = Field(..., description="Actual test outcome")
    issues_found: List[str] = Field(default_factory=list, description="Issues discovered")
    feedback_rating: int = Field(..., ge=1, le=5, description="Feedback rating (1-5)")
    feedback_comments: str = Field(..., description="Detailed feedback comments")
    screenshots: Optional[List[str]] = Field(None, description="Screenshot URLs")


@router.post("/register-tester")
async def register_tester(
    request: TesterRegistrationRequest,
    current_user: User = Depends(get_current_user)
):
    """Register as a UAT tester."""
    try:
        # Register tester profile
        uat_framework.register_tester(
            tester_id=current_user.id,
            profile={
                "username": current_user.username,
                "full_name": current_user.full_name,
                "email": current_user.email,
                "native_language": request.native_language,
                "language_proficiency": request.language_proficiency,
                "technical_background": request.technical_background,
                "device_info": request.device_info,
                "location": request.location,
                "availability": request.availability
            }
        )
        
        return {
            "success": True,
            "message": "Successfully registered as UAT tester",
            "tester_id": current_user.id
        }
        
    except Exception as e:
        logger.error(f"Error registering tester: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to register tester"
        )


@router.get("/scenarios")
async def get_test_scenarios(
    scenario_type: Optional[str] = Query(None, description="Filter by scenario type"),
    priority: Optional[str] = Query(None, regex="^(high|medium|low)$", description="Filter by priority"),
    current_user: User = Depends(get_current_user)
):
    """Get available test scenarios."""
    try:
        # Parse scenario type
        scenario_type_enum = None
        if scenario_type:
            try:
                scenario_type_enum = TestScenarioType(scenario_type)
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Invalid scenario type: {scenario_type}"
                )
        
        scenarios = uat_framework.get_test_scenarios(
            scenario_type=scenario_type_enum,
            priority=priority
        )
        
        # Convert to dict format
        scenario_dicts = []
        for scenario in scenarios:
            scenario_dict = {
                "id": scenario.id,
                "title": scenario.title,
                "description": scenario.description,
                "scenario_type": scenario.scenario_type.value,
                "steps": scenario.steps,
                "expected_outcome": scenario.expected_outcome,
                "acceptance_criteria": scenario.acceptance_criteria,
                "priority": scenario.priority,
                "estimated_duration": scenario.estimated_duration,
                "prerequisites": scenario.prerequisites
            }
            scenario_dicts.append(scenario_dict)
        
        return {
            "scenarios": scenario_dicts,
            "total_count": len(scenario_dicts),
            "filters": {
                "scenario_type": scenario_type,
                "priority": priority
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting test scenarios: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get test scenarios"
        )


@router.post("/executions")
async def create_test_execution(
    request: TestExecutionRequest,
    current_user: User = Depends(get_current_user)
):
    """Create a new test execution."""
    try:
        # Check if scenario exists
        scenarios = uat_framework.get_test_scenarios()
        scenario = next((s for s in scenarios if s.id == request.scenario_id), None)
        
        if not scenario:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Test scenario not found"
            )
        
        # Check if tester is registered
        if current_user.id not in uat_framework.tester_profiles:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Please register as a tester first"
            )
        
        # Create test execution
        execution = uat_framework.create_test_execution(
            scenario_id=request.scenario_id,
            tester_id=current_user.id,
            metadata=request.metadata
        )
        
        return {
            "success": True,
            "execution_id": execution.id,
            "scenario_id": execution.scenario_id,
            "status": execution.status.value,
            "message": "Test execution created successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error creating test execution: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create test execution"
        )


@router.put("/executions/{execution_id}/start")
async def start_test_execution(
    execution_id: str,
    current_user: User = Depends(get_current_user)
):
    """Start a test execution."""
    try:
        # Find execution and verify ownership
        execution = next(
            (e for e in uat_framework.test_executions if e.id == execution_id),
            None
        )
        
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Test execution not found"
            )
        
        if execution.tester_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to start this test execution"
            )
        
        # Start execution
        success = uat_framework.start_test_execution(execution_id)
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to start test execution"
            )
        
        return {
            "success": True,
            "execution_id": execution_id,
            "status": "in_progress",
            "message": "Test execution started successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error starting test execution: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to start test execution"
        )


@router.put("/executions/{execution_id}/complete")
async def complete_test_execution(
    execution_id: str,
    request: TestCompletionRequest,
    current_user: User = Depends(get_current_user)
):
    """Complete a test execution with results."""
    try:
        # Find execution and verify ownership
        execution = next(
            (e for e in uat_framework.test_executions if e.id == execution_id),
            None
        )
        
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Test execution not found"
            )
        
        if execution.tester_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to complete this test execution"
            )
        
        # Validate feedback rating
        try:
            feedback_rating = FeedbackRating(request.feedback_rating)
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid feedback rating. Must be between 1 and 5"
            )
        
        # Complete execution
        success = uat_framework.complete_test_execution(
            execution_id=execution_id,
            actual_outcome=request.actual_outcome,
            issues_found=request.issues_found,
            feedback_rating=feedback_rating,
            feedback_comments=request.feedback_comments,
            screenshots=request.screenshots
        )
        
        if not success:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to complete test execution"
            )
        
        return {
            "success": True,
            "execution_id": execution_id,
            "status": "completed",
            "message": "Test execution completed successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error completing test execution: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to complete test execution"
        )


@router.get("/my-assignments")
async def get_my_test_assignments(
    current_user: User = Depends(get_current_user)
):
    """Get test assignments for current user."""
    try:
        assignments = uat_framework.get_tester_assignments(current_user.id)
        
        return {
            "assignments": assignments,
            "total_count": len(assignments),
            "tester_id": current_user.id
        }
        
    except Exception as e:
        logger.error(f"Error getting test assignments: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get test assignments"
        )


@router.get("/results/summary")
async def get_test_results_summary(
    current_user: User = Depends(get_current_user)
):
    """Get summary of test results (admin only)."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        summary = uat_framework.get_test_results_summary()
        
        return summary
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting test results summary: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get test results summary"
        )


@router.get("/results/issues")
async def get_issues_report(
    current_user: User = Depends(get_current_user)
):
    """Get report of all issues found during testing (admin only)."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        issues_report = uat_framework.get_issues_report()
        
        return issues_report
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting issues report: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get issues report"
        )


@router.get("/executions")
async def get_test_executions(
    status_filter: Optional[str] = Query(None, description="Filter by status"),
    scenario_type: Optional[str] = Query(None, description="Filter by scenario type"),
    tester_id: Optional[int] = Query(None, description="Filter by tester ID"),
    limit: int = Query(50, ge=1, le=1000, description="Limit results"),
    current_user: User = Depends(get_current_user)
):
    """Get test executions with optional filtering (admin only)."""
    try:
        # Check if user has admin privileges
        if not getattr(current_user, 'is_admin', False):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Admin privileges required"
            )
        
        executions = uat_framework.test_executions
        
        # Apply filters
        if status_filter:
            try:
                status_enum = TestStatus(status_filter)
                executions = [e for e in executions if e.status == status_enum]
            except ValueError:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Invalid status filter: {status_filter}"
                )
        
        if tester_id:
            executions = [e for e in executions if e.tester_id == tester_id]
        
        if scenario_type:
            # Filter by scenario type
            scenarios = uat_framework.get_test_scenarios()
            scenario_ids = [s.id for s in scenarios if s.scenario_type.value == scenario_type]
            executions = [e for e in executions if e.scenario_id in scenario_ids]
        
        # Sort by start time (most recent first)
        executions.sort(key=lambda x: x.started_at, reverse=True)
        
        # Limit results
        executions = executions[:limit]
        
        # Convert to response format
        execution_dicts = []
        for execution in executions:
            scenario = next(
                (s for s in uat_framework.test_scenarios if s.id == execution.scenario_id),
                None
            )
            
            execution_dict = {
                "id": execution.id,
                "scenario_id": execution.scenario_id,
                "scenario_title": scenario.title if scenario else "Unknown",
                "scenario_type": scenario.scenario_type.value if scenario else "unknown",
                "tester_id": execution.tester_id,
                "status": execution.status.value,
                "started_at": execution.started_at.isoformat(),
                "completed_at": execution.completed_at.isoformat() if execution.completed_at else None,
                "feedback_rating": execution.feedback_rating.value if execution.feedback_rating else None,
                "issues_count": len(execution.issues_found),
                "has_screenshots": len(execution.screenshots) > 0
            }
            execution_dicts.append(execution_dict)
        
        return {
            "executions": execution_dicts,
            "total_count": len(execution_dicts),
            "filters": {
                "status": status_filter,
                "scenario_type": scenario_type,
                "tester_id": tester_id
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting test executions: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get test executions"
        )


@router.get("/executions/{execution_id}")
async def get_test_execution_details(
    execution_id: str,
    current_user: User = Depends(get_current_user)
):
    """Get detailed information about a test execution."""
    try:
        # Find execution
        execution = next(
            (e for e in uat_framework.test_executions if e.id == execution_id),
            None
        )
        
        if not execution:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Test execution not found"
            )
        
        # Check permissions (admin or execution owner)
        if (execution.tester_id != current_user.id and 
            not getattr(current_user, 'is_admin', False)):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Not authorized to view this test execution"
            )
        
        # Get scenario details
        scenario = next(
            (s for s in uat_framework.test_scenarios if s.id == execution.scenario_id),
            None
        )
        
        # Get tester profile
        tester_profile = uat_framework.tester_profiles.get(execution.tester_id, {})
        
        execution_details = {
            "id": execution.id,
            "scenario": {
                "id": scenario.id,
                "title": scenario.title,
                "description": scenario.description,
                "scenario_type": scenario.scenario_type.value,
                "steps": scenario.steps,
                "expected_outcome": scenario.expected_outcome,
                "acceptance_criteria": scenario.acceptance_criteria,
                "priority": scenario.priority,
                "estimated_duration": scenario.estimated_duration,
                "prerequisites": scenario.prerequisites
            } if scenario else None,
            "tester": {
                "id": execution.tester_id,
                "username": tester_profile.get("username", "Unknown"),
                "location": tester_profile.get("location", "Unknown"),
                "native_language": tester_profile.get("native_language", "Unknown")
            },
            "status": execution.status.value,
            "started_at": execution.started_at.isoformat(),
            "completed_at": execution.completed_at.isoformat() if execution.completed_at else None,
            "actual_outcome": execution.actual_outcome,
            "issues_found": execution.issues_found,
            "feedback_rating": execution.feedback_rating.value if execution.feedback_rating else None,
            "feedback_comments": execution.feedback_comments,
            "screenshots": execution.screenshots,
            "metadata": execution.metadata
        }
        
        return execution_details
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting test execution details: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get test execution details"
        )