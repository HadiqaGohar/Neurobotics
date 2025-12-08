"""Tests for multilingual functionality."""

import pytest
from datetime import datetime
from sqlalchemy.orm import Session

from app.models.multilingual import (
    Language, ContentTranslation, TranslationMemory, TerminologyGlossary,
    TranslationStatus, TranslationMethod, TextDirection
)
from src.multilingual.service import multilingual_service
from src.multilingual.workflow import translation_workflow, Priority


class TestMultilingualModels:
    """Test multilingual database models."""
    
    def test_language_model(self, db: Session):
        """Test Language model creation."""
        language = Language(
            code="ur",
            name="Urdu",
            native_name="اردو",
            direction=TextDirection.RTL,
            is_active=True,
            is_default=False
        )
        
        db.add(language)
        db.commit()
        db.refresh(language)
        
        assert language.id is not None
        assert language.code == "ur"
        assert language.direction == TextDirection.RTL
        assert language.is_active is True
    
    def test_content_translation_model(self, db: Session):
        """Test ContentTranslation model creation."""
        # Create language first
        language = Language(
            code="ur",
            name="Urdu",
            native_name="اردو",
            direction=TextDirection.RTL,
            is_active=True
        )
        db.add(language)
        db.commit()
        
        translation = ContentTranslation(
            content_type="chapter",
            content_id="ch001",
            language_code="ur",
            title="پروگرامنگ کا تعارف",
            content="یہ پروگرامنگ کا بنیادی تعارف ہے۔",
            translation_method=TranslationMethod.MANUAL,
            translation_status=TranslationStatus.PENDING
        )
        
        db.add(translation)
        db.commit()
        db.refresh(translation)
        
        assert translation.id is not None
        assert translation.language_code == "ur"
        assert translation.translation_status == TranslationStatus.PENDING
    
    def test_translation_memory_model(self, db: Session):
        """Test TranslationMemory model creation."""
        # Create languages
        en_lang = Language(code="en", name="English", native_name="English", direction=TextDirection.LTR)
        ur_lang = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add_all([en_lang, ur_lang])
        db.commit()
        
        memory = TranslationMemory(
            source_text="Hello World",
            target_text="ہیلو ورلڈ",
            source_language="en",
            target_language="ur",
            context="greeting",
            domain="general",
            quality_score=0.95
        )
        
        db.add(memory)
        db.commit()
        db.refresh(memory)
        
        assert memory.id is not None
        assert memory.quality_score == 0.95
        assert memory.usage_count == 0


class TestMultilingualService:
    """Test multilingual service functionality."""
    
    @pytest.mark.asyncio
    async def test_get_supported_languages(self, db: Session):
        """Test getting supported languages."""
        # Create test languages
        languages = [
            Language(code="en", name="English", native_name="English", direction=TextDirection.LTR, is_active=True),
            Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL, is_active=True),
            Language(code="ar", name="Arabic", native_name="العربية", direction=TextDirection.RTL, is_active=False)
        ]
        
        db.add_all(languages)
        db.commit()
        
        result = await multilingual_service.get_supported_languages(db)
        
        assert len(result) == 2  # Only active languages
        assert any(lang["code"] == "en" for lang in result)
        assert any(lang["code"] == "ur" for lang in result)
        assert not any(lang["code"] == "ar" for lang in result)
    
    @pytest.mark.asyncio
    async def test_detect_user_language(self):
        """Test user language detection."""
        # Test with Accept-Language header
        result = await multilingual_service.detect_user_language(
            accept_language="ur,en;q=0.9,ar;q=0.8"
        )
        
        assert result in ["ur", "en"]  # Should return supported language
    
    @pytest.mark.asyncio
    async def test_translation_memory(self, db: Session):
        """Test translation memory functionality."""
        # Create languages
        en_lang = Language(code="en", name="English", native_name="English", direction=TextDirection.LTR)
        ur_lang = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add_all([en_lang, ur_lang])
        db.commit()
        
        # Add translation to memory
        success = await multilingual_service.add_translation_memory(
            source_text="Programming",
            target_text="پروگرامنگ",
            source_language="en",
            target_language="ur",
            context="technical",
            domain="programming",
            quality_score=0.9,
            db=db
        )
        
        assert success is True
        
        # Retrieve from memory
        result = await multilingual_service.get_translation_memory(
            source_text="Programming",
            source_language="en",
            target_language="ur",
            db=db
        )
        
        assert result is not None
        assert result["target_text"] == "پروگرامنگ"
        assert result["quality_score"] == 0.9


class TestTranslationWorkflow:
    """Test translation workflow functionality."""
    
    @pytest.mark.asyncio
    async def test_create_translation_job(self, db: Session):
        """Test creating a translation job."""
        # Create language
        language = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add(language)
        db.commit()
        
        job_id = await translation_workflow.create_translation_job(
            content_type="chapter",
            content_id="ch001",
            source_language="en",
            target_language="ur",
            title="Introduction to Programming",
            content="This is an introduction to programming concepts.",
            priority=Priority.HIGH,
            db=db
        )
        
        assert job_id is not None
        
        # Verify job was created
        job = db.query(ContentTranslation).filter(ContentTranslation.id == job_id).first()
        assert job is not None
        assert job.content_type == "chapter"
        assert job.language_code == "ur"
        assert job.translation_status == TranslationStatus.PENDING
    
    @pytest.mark.asyncio
    async def test_workflow_statistics(self, db: Session):
        """Test getting workflow statistics."""
        # Create test data
        language = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add(language)
        db.commit()
        
        # Create some translation jobs
        for i in range(3):
            translation = ContentTranslation(
                content_type="chapter",
                content_id=f"ch{i:03d}",
                language_code="ur",
                translation_method=TranslationMethod.AI,
                translation_status=TranslationStatus.PENDING if i < 2 else TranslationStatus.PUBLISHED,
                quality_score=0.8 + (i * 0.1)
            )
            db.add(translation)
        
        db.commit()
        
        stats = await translation_workflow.get_workflow_statistics(db)
        
        assert stats["total_jobs"] == 3
        assert stats["by_status"]["pending"] == 2
        assert stats["by_status"]["published"] == 1
        assert "quality_metrics" in stats


class TestRTLSupport:
    """Test RTL language support."""
    
    def test_rtl_text_validation(self):
        """Test RTL text validation."""
        from book_frontend.src.utils.rtlUtils import validateRTLText, isRTLLanguage
        
        # Test RTL language detection
        assert isRTLLanguage("ur") is True
        assert isRTLLanguage("ar") is True
        assert isRTLLanguage("en") is False
        
        # Test RTL text validation
        urdu_text = "یہ اردو متن ہے"
        english_text = "This is English text"
        
        assert validateRTLText(urdu_text, "ur") is True
        assert validateRTLText(english_text, "en") is True
    
    def test_bidi_text_direction(self):
        """Test bidirectional text direction detection."""
        from book_frontend.src.utils.rtlUtils import getBidiTextDirection
        
        # Pure RTL text
        rtl_text = "یہ اردو متن ہے"
        assert getBidiTextDirection(rtl_text) == "rtl"
        
        # Pure LTR text
        ltr_text = "This is English text"
        assert getBidiTextDirection(ltr_text) == "ltr"
        
        # Mixed text
        mixed_text = "This is English اور یہ اردو ہے"
        assert getBidiTextDirection(mixed_text) == "mixed"


class TestTranslationQuality:
    """Test translation quality assurance."""
    
    @pytest.mark.asyncio
    async def test_quality_scoring(self, db: Session):
        """Test translation quality scoring."""
        # Create languages
        en_lang = Language(code="en", name="English", native_name="English", direction=TextDirection.LTR)
        ur_lang = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add_all([en_lang, ur_lang])
        db.commit()
        
        # Create translation with quality score
        translation = ContentTranslation(
            content_type="chapter",
            content_id="ch001",
            language_code="ur",
            title="پروگرامنگ کا تعارف",
            content="یہ پروگرامنگ کا بنیادی تعارف ہے۔",
            translation_method=TranslationMethod.MANUAL,
            translation_status=TranslationStatus.APPROVED,
            quality_score=0.95
        )
        
        db.add(translation)
        db.commit()
        
        # Test quality threshold
        assert translation.quality_score >= 0.8  # High quality threshold
    
    def test_terminology_consistency(self, db: Session):
        """Test terminology consistency."""
        # Create languages
        en_lang = Language(code="en", name="English", native_name="English", direction=TextDirection.LTR)
        ur_lang = Language(code="ur", name="Urdu", native_name="اردو", direction=TextDirection.RTL)
        db.add_all([en_lang, ur_lang])
        db.commit()
        
        # Create terminology entries
        terms = [
            TerminologyGlossary(
                term="variable",
                translation="متغیر",
                source_language="en",
                target_language="ur",
                domain="programming",
                approved=True
            ),
            TerminologyGlossary(
                term="function",
                translation="فنکشن",
                source_language="en",
                target_language="ur",
                domain="programming",
                approved=True
            )
        ]
        
        db.add_all(terms)
        db.commit()
        
        # Verify terminology exists
        variable_term = db.query(TerminologyGlossary).filter(
            TerminologyGlossary.term == "variable"
        ).first()
        
        assert variable_term is not None
        assert variable_term.translation == "متغیر"
        assert variable_term.approved is True


@pytest.fixture
def db():
    """Database fixture for testing."""
    # This would be implemented with your test database setup
    # For now, returning a mock
    class MockDB:
        def add(self, obj):
            pass
        
        def add_all(self, objs):
            pass
        
        def commit(self):
            pass
        
        def refresh(self, obj):
            obj.id = 1
        
        def query(self, model):
            return MockQuery()
    
    class MockQuery:
        def filter(self, *args):
            return self
        
        def first(self):
            return None
        
        def all(self):
            return []
        
        def count(self):
            return 0
    
    return MockDB()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])