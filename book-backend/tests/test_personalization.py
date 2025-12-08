"""
Comprehensive test suite for personalization system.
"""

import pytest
import asyncio
from unittest.mock import Mock, patch
from datetime import datetime

from src.personalization.service import PersonalizationService
from src.personalization.models import UserPreferences, SoftwareBackground, HardwareBackground
from src.personalization.content_adapter import ContentAdaptationEngine
from src.personalization.rules_engine import PersonalizationRulesEngine
from src.personalization.cache_manager import PersonalizationCacheManager


class TestPersonalizationService:
    """Test cases for personalization service."""
    
    @pytest.fixture
    def mock_preferences(self):
        """Mock user preferences for testing."""
        return UserPreferences(
            user_id="test_user_1",
            software_background=SoftwareBackground(
                categories=["web_development"],
                experience_level="intermediate",
                preferred_languages=["python", "javascript"],
                frameworks=["react", "django"]
            ),
            hardware_background=HardwareBackground(
                categories=["iot_devices"],
                experience_level="beginner",
                platforms=["arduino", "raspberry_pi"],
                components=["sensors", "actuators"]
            ),
            content_complexity="moderate",
            explanation_depth="standard",
            example_style="practical"
        )
    
    @pytest.fixture
    def mock_content(self):
        """Mock content for testing."""
        return {
            "title": "Test Chapter",
            "text": "This is test content for personalization.",
            "learning_objectives": ["Understand concepts", "Apply knowledge"],
            "key_concepts": ["Concept 1", "Concept 2"]
        }
    
    def test_content_adaptation_engine(self, mock_content, mock_preferences):
        """Test content adaptation engine."""
        engine = ContentAdaptationEngine()
        
        adapted_content = engine.adapt_content(
            original_content=mock_content,
            preferences=mock_preferences
        )
        
        assert "adaptation_metadata" in adapted_content
        assert "code_examples" in adapted_content
        assert len(adapted_content["code_examples"]) > 0
        
        # Check if Python examples are included (user preference)
        python_examples = [ex for ex in adapted_content["code_examples"] if ex["language"] == "python"]
        assert len(python_examples) > 0
    
    def test_rules_engine_execution(self, mock_content, mock_preferences):
        """Test personalization rules engine."""
        engine = PersonalizationRulesEngine()
        
        personalized_content, rule_results = engine.execute_rules(
            content=mock_content,
            user_preferences=mock_preferences
        )
        
        assert "rule_execution_metadata" in personalized_content
        assert len(rule_results) > 0
        
        # Check that some rules were executed
        executed_rules = [r for r in rule_results if r.executed]
        assert len(executed_rules) > 0
    
    def test_cache_manager_operations(self):
        """Test cache manager functionality."""
        cache_manager = PersonalizationCacheManager()
        
        # Test set and get
        test_key = "test_key"
        test_value = {"test": "data"}
        
        assert cache_manager.set(test_key, test_value)
        retrieved_value = cache_manager.get(test_key)
        
        assert retrieved_value == test_value
        
        # Test cache stats
        stats = cache_manager.get_stats()
        assert "cache_stats" in stats
        assert stats["cache_stats"]["total_requests"] > 0
    
    @pytest.mark.asyncio
    async def test_personalization_workflow(self, mock_content, mock_preferences):
        """Test complete personalization workflow."""
        service = PersonalizationService()
        
        # Mock database session
        mock_db = Mock()
        
        with patch.object(service, 'get_user_preferences', return_value=mock_preferences):
            with patch.object(service, '_get_base_content', return_value=mock_content):
                # Test content personalization
                request = Mock()
                request.user_id = "test_user_1"
                request.chapter_id = "chapter_1"
                request.section_id = None
                request.force_regenerate = False
                
                response = await service.personalize_content(request, mock_db)
                
                assert response.content_id is not None
                assert response.personalized_content is not None
                assert response.personalization_applied is not None


class TestPerformanceAndScalability:
    """Test cases for performance and scalability."""
    
    def test_concurrent_personalization_requests(self):
        """Test handling of concurrent personalization requests."""
        cache_manager = PersonalizationCacheManager(max_size=1000)
        
        # Simulate concurrent cache operations
        async def cache_operation(i):
            key = f"test_key_{i}"
            value = {"data": f"test_data_{i}"}
            cache_manager.set(key, value)
            return cache_manager.get(key)
        
        async def run_concurrent_operations():
            tasks = [cache_operation(i) for i in range(100)]
            results = await asyncio.gather(*tasks)
            return results
        
        # Run the test
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        results = loop.run_until_complete(run_concurrent_operations())
        
        assert len(results) == 100
        assert all(result is not None for result in results)
    
    def test_cache_eviction_performance(self):
        """Test cache eviction performance under load."""
        cache_manager = PersonalizationCacheManager(max_size=100)
        
        # Fill cache beyond capacity
        for i in range(150):
            key = f"test_key_{i}"
            value = {"data": f"test_data_{i}"}
            cache_manager.set(key, value)
        
        # Verify cache size is maintained
        assert len(cache_manager.cache) <= cache_manager.max_size
        
        # Verify eviction statistics
        stats = cache_manager.get_stats()
        assert stats["cache_stats"]["evictions"] > 0


class TestSecurityAndPrivacy:
    """Test cases for security and privacy measures."""
    
    def test_user_data_isolation(self, mock_preferences):
        """Test that user data is properly isolated."""
        cache_manager = PersonalizationCacheManager()
        
        # Set preferences for two different users
        user1_key = "user_prefs:user1"
        user2_key = "user_prefs:user2"
        
        user1_prefs = mock_preferences.dict()
        user2_prefs = mock_preferences.dict()
        user2_prefs["user_id"] = "user2"
        
        cache_manager.set(user1_key, user1_prefs)
        cache_manager.set(user2_key, user2_prefs)
        
        # Verify isolation
        retrieved_user1 = cache_manager.get(user1_key)
        retrieved_user2 = cache_manager.get(user2_key)
        
        assert retrieved_user1["user_id"] != retrieved_user2["user_id"]
    
    def test_preference_validation(self):
        """Test preference data validation."""
        # Test with invalid data
        with pytest.raises(Exception):
            UserPreferences(
                user_id="",  # Invalid empty user ID
                software_background=SoftwareBackground(),
                hardware_background=HardwareBackground()
            )


class TestIntegrationScenarios:
    """Integration test scenarios."""
    
    @pytest.mark.asyncio
    async def test_end_to_end_personalization_journey(self, mock_preferences, mock_content):
        """Test complete user personalization journey."""
        service = PersonalizationService()
        mock_db = Mock()
        
        # Mock the database operations
        with patch.object(service, 'get_user_preferences', return_value=mock_preferences):
            with patch.object(service, 'save_user_preferences', return_value=True):
                with patch.object(service, '_get_base_content', return_value=mock_content):
                    
                    # Step 1: Save user preferences
                    save_result = await service.save_user_preferences(mock_preferences, mock_db)
                    assert save_result is True
                    
                    # Step 2: Get personalized content
                    request = Mock()
                    request.user_id = mock_preferences.user_id
                    request.chapter_id = "chapter_1"
                    request.section_id = None
                    request.force_regenerate = False
                    
                    response = await service.personalize_content(request, mock_db)
                    
                    # Verify response
                    assert response.content_id is not None
                    assert response.personalized_content is not None
                    assert "code_examples" in response.personalized_content
                    assert "hardware_examples" in response.personalized_content
                    
                    # Step 3: Verify caching works
                    response2 = await service.personalize_content(request, mock_db)
                    assert response2.cache_hit is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])