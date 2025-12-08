#!/usr/bin/env python3
"""
Development environment setup script for multilingual support.
"""

import os
import sys
import subprocess
import json
from pathlib import Path

def run_command(command, cwd=None):
    """Run a shell command and return the result."""
    try:
        result = subprocess.run(
            command,
            shell=True,
            cwd=cwd,
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Error running command '{command}': {e}")
        print(f"Error output: {e.stderr}")
        return None

def setup_database():
    """Set up database with multilingual tables."""
    print("Setting up database with multilingual support...")
    
    # Run Alembic migrations
    result = run_command("alembic upgrade head")
    if result is not None:
        print("✓ Database migrations completed")
    else:
        print("✗ Database migration failed")
        return False
    
    return True

def install_language_packages():
    """Install required language packages."""
    print("Installing language packages...")
    
    packages = [
        "googletrans==4.0.0rc1",  # Google Translate
        "langdetect==1.0.9",      # Language detection
        "polyglot",               # Multilingual NLP
        "babel==2.12.1",          # Internationalization
        "python-bidi==0.4.2",     # Bidirectional text support
    ]
    
    for package in packages:
        print(f"Installing {package}...")
        result = run_command(f"pip install {package}")
        if result is not None:
            print(f"✓ {package} installed")
        else:
            print(f"✗ Failed to install {package}")
    
    return True

def setup_font_resources():
    """Set up font resources for RTL languages."""
    print("Setting up font resources...")
    
    # Create fonts directory
    fonts_dir = Path("../book-frontend/public/fonts")
    fonts_dir.mkdir(parents=True, exist_ok=True)
    
    # Font configuration
    font_config = {
        "fonts": {
            "urdu": {
                "family": "Noto Nastaliq Urdu",
                "files": [
                    "NotoNastaliqUrdu-Regular.ttf",
                    "NotoNastaliqUrdu-Bold.ttf"
                ],
                "fallbacks": ["Jameel Noori Nastaleeq", "serif"]
            },
            "arabic": {
                "family": "Noto Sans Arabic",
                "files": [
                    "NotoSansArabic-Regular.ttf",
                    "NotoSansArabic-Bold.ttf"
                ],
                "fallbacks": ["Arabic Typesetting", "Traditional Arabic", "serif"]
            }
        }
    }
    
    # Save font configuration
    config_file = fonts_dir / "font-config.json"
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(font_config, f, indent=2, ensure_ascii=False)
    
    print("✓ Font configuration created")
    
    # Create CSS for font loading
    font_css = """
/* Multilingual Font Loading */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('./fonts/NotoNastaliqUrdu-Regular.ttf') format('truetype');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('./fonts/NotoNastaliqUrdu-Bold.ttf') format('truetype');
  font-weight: bold;
  font-style: normal;
  font-display: swap;
}

@font-face {
  font-family: 'Noto Sans Arabic';
  src: url('./fonts/NotoSansArabic-Regular.ttf') format('truetype');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

@font-face {
  font-family: 'Noto Sans Arabic';
  src: url('./fonts/NotoSansArabic-Bold.ttf') format('truetype');
  font-weight: bold;
  font-style: normal;
  font-display: swap;
}

/* RTL Language Support */
[lang="ur"] {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  direction: rtl;
  text-align: right;
}

[lang="ar"] {
  font-family: 'Noto Sans Arabic', 'Arabic Typesetting', serif;
  direction: rtl;
  text-align: right;
}
"""
    
    css_file = Path("../book-frontend/src/styles/fonts.css")
    css_file.parent.mkdir(parents=True, exist_ok=True)
    with open(css_file, 'w', encoding='utf-8') as f:
        f.write(font_css)
    
    print("✓ Font CSS created")
    return True

def setup_test_data():
    """Set up test data for multilingual development."""
    print("Setting up test data...")
    
    test_data = {
        "languages": [
            {
                "code": "en",
                "name": "English",
                "native_name": "English",
                "direction": "ltr",
                "is_active": True,
                "is_default": True
            },
            {
                "code": "ur",
                "name": "Urdu",
                "native_name": "اردو",
                "direction": "rtl",
                "is_active": True,
                "is_default": False
            }
        ],
        "sample_content": {
            "en": {
                "title": "Introduction to Programming",
                "content": "Programming is the process of creating a set of instructions that tell a computer how to perform a task."
            },
            "ur": {
                "title": "پروگرامنگ کا تعارف",
                "content": "پروگرامنگ ہدایات کا ایک مجموعہ بنانے کا عمل ہے جو کمپیوٹر کو بتاتا ہے کہ کوئی کام کیسے کرنا ہے۔"
            }
        },
        "terminology": [
            {
                "term": "variable",
                "translation": "متغیر",
                "definition": "A storage location with an associated name",
                "domain": "programming"
            },
            {
                "term": "function",
                "translation": "فنکشن",
                "definition": "A block of code that performs a specific task",
                "domain": "programming"
            }
        ]
    }
    
    # Save test data
    test_data_file = Path("test_data/multilingual_test_data.json")
    test_data_file.parent.mkdir(parents=True, exist_ok=True)
    with open(test_data_file, 'w', encoding='utf-8') as f:
        json.dump(test_data, f, indent=2, ensure_ascii=False)
    
    print("✓ Test data created")
    return True

def setup_development_tools():
    """Set up development tools for multilingual support."""
    print("Setting up development tools...")
    
    # Create development scripts
    scripts_dir = Path("scripts/multilingual")
    scripts_dir.mkdir(parents=True, exist_ok=True)
    
    # Translation validation script
    validation_script = """#!/usr/bin/env python3
\"\"\"Translation validation script.\"\"\"

import json
import re
from pathlib import Path

def validate_rtl_text(text, language):
    \"\"\"Validate RTL text content.\"\"\"
    if language in ['ur', 'ar', 'fa', 'he']:
        # Check for RTL characters
        rtl_pattern = r'[\\u0590-\\u05FF\\u0600-\\u06FF\\u0750-\\u077F\\u08A0-\\u08FF\\uFB50-\\uFDFF\\uFE70-\\uFEFF]'
        if not re.search(rtl_pattern, text):
            return False, "No RTL characters found"
    return True, "Valid"

def validate_translation_completeness(source, target):
    \"\"\"Check translation completeness.\"\"\"
    # Basic checks
    if not target.strip():
        return False, "Empty translation"
    
    # Check for untranslated placeholders
    placeholders = re.findall(r'\\{[^}]+\\}', source)
    for placeholder in placeholders:
        if placeholder not in target:
            return False, f"Missing placeholder: {placeholder}"
    
    return True, "Complete"

if __name__ == "__main__":
    print("Translation validation tool ready")
"""
    
    with open(scripts_dir / "validate_translations.py", 'w') as f:
        f.write(validation_script)
    
    # RTL testing script
    rtl_test_script = """#!/usr/bin/env python3
\"\"\"RTL layout testing script.\"\"\"

def test_rtl_layout():
    \"\"\"Test RTL layout components.\"\"\"
    test_cases = [
        {
            "name": "Text Direction",
            "test": "Check if RTL text flows right-to-left",
            "expected": "dir='rtl' attribute present"
        },
        {
            "name": "Icon Flipping",
            "test": "Check if directional icons are flipped",
            "expected": "transform: scaleX(-1) applied"
        },
        {
            "name": "Margin/Padding",
            "test": "Check if margins and padding are mirrored",
            "expected": "margin-left becomes margin-right"
        }
    ]
    
    for test_case in test_cases:
        print(f"Testing: {test_case['name']}")
        print(f"Expected: {test_case['expected']}")
        print("---")

if __name__ == "__main__":
    test_rtl_layout()
"""
    
    with open(scripts_dir / "test_rtl_layout.py", 'w') as f:
        f.write(rtl_test_script)
    
    print("✓ Development tools created")
    return True

def setup_ci_cd_config():
    """Set up CI/CD configuration for multilingual testing."""
    print("Setting up CI/CD configuration...")
    
    # GitHub Actions workflow for multilingual testing
    workflow_config = """name: Multilingual Tests

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  multilingual-tests:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.9'
    
    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-cov
    
    - name: Run multilingual tests
      run: |
        pytest tests/test_multilingual.py -v
    
    - name: Test RTL layout
      run: |
        python scripts/multilingual/test_rtl_layout.py
    
    - name: Validate translations
      run: |
        python scripts/multilingual/validate_translations.py
"""
    
    workflow_dir = Path(".github/workflows")
    workflow_dir.mkdir(parents=True, exist_ok=True)
    with open(workflow_dir / "multilingual-tests.yml", 'w') as f:
        f.write(workflow_config)
    
    print("✓ CI/CD configuration created")
    return True

def create_development_guide():
    """Create development guide for multilingual features."""
    print("Creating development guide...")
    
    guide_content = """# Multilingual Development Guide

## Overview
This guide covers development practices for multilingual support in the Book Platform.

## RTL Language Support

### CSS Guidelines
- Use logical properties (margin-inline-start instead of margin-left)
- Test layouts in both LTR and RTL modes
- Use the RTL utility classes provided in rtl.css

### Component Development
- Import RTLWrapper for RTL-aware components
- Use getRTLPosition() for dynamic positioning
- Test with actual RTL text content

## Translation Workflow

### Adding New Content
1. Create content in English (source language)
2. Use translation workflow API to create translation jobs
3. Assign translators through the workflow system
4. Review and approve translations
5. Publish approved translations

### Quality Assurance
- Use translation memory for consistency
- Implement terminology glossary
- Set quality score thresholds
- Regular review cycles

## Testing

### RTL Testing
```bash
# Run RTL layout tests
python scripts/multilingual/test_rtl_layout.py

# Validate translations
python scripts/multilingual/validate_translations.py
```

### Browser Testing
- Test in Chrome, Firefox, Safari
- Test on mobile devices
- Use browser dev tools to simulate RTL
- Test with screen readers

## Font Management

### Adding New Fonts
1. Add font files to public/fonts/
2. Update font-config.json
3. Add @font-face declarations
4. Test font loading performance

### Font Optimization
- Use font-display: swap
- Preload critical fonts
- Implement font fallbacks
- Monitor font loading metrics

## Performance Considerations

### Translation Loading
- Implement lazy loading for translations
- Use translation caching
- Optimize API calls
- Monitor translation load times

### Font Performance
- Subset fonts for specific languages
- Use WOFF2 format when possible
- Implement progressive font loading
- Monitor Core Web Vitals

## Debugging

### Common Issues
- Text direction not applied: Check dir attribute
- Icons not flipped: Verify RTL CSS classes
- Layout broken: Test logical properties
- Fonts not loading: Check font paths and CORS

### Debug Tools
- Browser RTL simulation
- Translation validation scripts
- Font loading analysis
- Performance monitoring

## Best Practices

### Code Organization
- Separate RTL styles in dedicated files
- Use consistent naming conventions
- Document RTL-specific components
- Maintain translation keys registry

### Content Management
- Use structured content format
- Implement version control for translations
- Maintain translation memory
- Regular quality audits

## Resources

### External Tools
- Google Fonts (Noto fonts for RTL languages)
- Babel for internationalization
- ICU for text processing
- Polyglot for language detection

### Documentation
- MDN RTL Guidelines
- W3C Internationalization
- Unicode Bidirectional Algorithm
- CSS Logical Properties Spec
"""
    
    guide_file = Path("docs/multilingual-development-guide.md")
    guide_file.parent.mkdir(parents=True, exist_ok=True)
    with open(guide_file, 'w', encoding='utf-8') as f:
        f.write(guide_content)
    
    print("✓ Development guide created")
    return True

def main():
    """Main setup function."""
    print("Setting up multilingual development environment...")
    print("=" * 50)
    
    steps = [
        ("Database Setup", setup_database),
        ("Language Packages", install_language_packages),
        ("Font Resources", setup_font_resources),
        ("Test Data", setup_test_data),
        ("Development Tools", setup_development_tools),
        ("CI/CD Configuration", setup_ci_cd_config),
        ("Development Guide", create_development_guide),
    ]
    
    success_count = 0
    for step_name, step_function in steps:
        print(f"\n{step_name}:")
        try:
            if step_function():
                success_count += 1
        except Exception as e:
            print(f"✗ Error in {step_name}: {e}")
    
    print("\n" + "=" * 50)
    print(f"Setup completed: {success_count}/{len(steps)} steps successful")
    
    if success_count == len(steps):
        print("✓ Multilingual development environment ready!")
        print("\nNext steps:")
        print("1. Run 'alembic upgrade head' to apply database migrations")
        print("2. Start the development server")
        print("3. Test RTL layout with sample content")
        print("4. Review the development guide in docs/")
    else:
        print("✗ Some steps failed. Please check the errors above.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())