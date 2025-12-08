-- Initialize personalization database tables
-- This script creates the necessary tables for the personalization service

-- Create user_preferences table
CREATE TABLE IF NOT EXISTS user_preferences (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) UNIQUE NOT NULL,
    software_background JSONB NOT NULL DEFAULT '{}',
    hardware_background JSONB NOT NULL DEFAULT '{}',
    content_complexity VARCHAR(50) NOT NULL DEFAULT 'moderate',
    explanation_depth VARCHAR(50) NOT NULL DEFAULT 'standard',
    example_style VARCHAR(50) NOT NULL DEFAULT 'practical',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

-- Create indexes for user_preferences
CREATE INDEX IF NOT EXISTS idx_user_preferences_user_id ON user_preferences(user_id);
CREATE INDEX IF NOT EXISTS idx_user_preferences_updated_at ON user_preferences(updated_at);

-- Create content_variants table
CREATE TABLE IF NOT EXISTS content_variants (
    id SERIAL PRIMARY KEY,
    content_id VARCHAR(255) UNIQUE NOT NULL,
    chapter_id VARCHAR(255) NOT NULL,
    section_id VARCHAR(255),
    variant_type VARCHAR(50) NOT NULL,
    target_audience JSONB NOT NULL DEFAULT '{}',
    content JSONB NOT NULL DEFAULT '{}',
    metadata JSONB NOT NULL DEFAULT '{}',
    is_ai_generated BOOLEAN NOT NULL DEFAULT FALSE,
    quality_score INTEGER,
    usage_count INTEGER NOT NULL DEFAULT 0,
    user_preferences_id INTEGER REFERENCES user_preferences(id),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

-- Create indexes for content_variants
CREATE INDEX IF NOT EXISTS idx_content_variants_content_id ON content_variants(content_id);
CREATE INDEX IF NOT EXISTS idx_content_variants_chapter_id ON content_variants(chapter_id);
CREATE INDEX IF NOT EXISTS idx_content_variants_section_id ON content_variants(section_id);
CREATE INDEX IF NOT EXISTS idx_content_variants_variant_type ON content_variants(variant_type);
CREATE INDEX IF NOT EXISTS idx_content_variants_is_ai_generated ON content_variants(is_ai_generated);

-- Create personalization_logs table
CREATE TABLE IF NOT EXISTS personalization_logs (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL,
    chapter_id VARCHAR(255) NOT NULL,
    section_id VARCHAR(255),
    requested_complexity VARCHAR(50),
    applied_personalization JSONB NOT NULL DEFAULT '{}',
    cache_hit BOOLEAN NOT NULL DEFAULT FALSE,
    generation_time_ms INTEGER NOT NULL DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

-- Create indexes for personalization_logs
CREATE INDEX IF NOT EXISTS idx_personalization_logs_user_id ON personalization_logs(user_id);
CREATE INDEX IF NOT EXISTS idx_personalization_logs_chapter_id ON personalization_logs(chapter_id);
CREATE INDEX IF NOT EXISTS idx_personalization_logs_created_at ON personalization_logs(created_at);
CREATE INDEX IF NOT EXISTS idx_personalization_logs_cache_hit ON personalization_logs(cache_hit);

-- Create trigger to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Apply trigger to user_preferences table
DROP TRIGGER IF EXISTS update_user_preferences_updated_at ON user_preferences;
CREATE TRIGGER update_user_preferences_updated_at
    BEFORE UPDATE ON user_preferences
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Apply trigger to content_variants table
DROP TRIGGER IF EXISTS update_content_variants_updated_at ON content_variants;
CREATE TRIGGER update_content_variants_updated_at
    BEFORE UPDATE ON content_variants
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Insert some sample data for testing (optional)
-- This can be removed in production

-- Sample user preferences
INSERT INTO user_preferences (user_id, software_background, hardware_background, content_complexity) 
VALUES 
    ('sample_user_1', 
     '{"categories": ["web_development"], "experience_level": "intermediate", "preferred_languages": ["javascript", "python"], "frameworks": ["react", "django"]}',
     '{"categories": ["iot_devices"], "experience_level": "beginner", "platforms": ["arduino", "raspberry_pi"], "components": ["sensors"]}',
     'moderate'
    ),
    ('sample_user_2',
     '{"categories": ["data_science", "machine_learning"], "experience_level": "advanced", "preferred_languages": ["python", "r"], "frameworks": ["tensorflow", "pandas"]}',
     '{"categories": ["embedded_systems"], "experience_level": "expert", "platforms": ["stm32", "arm"], "components": ["microcontrollers"]}',
     'detailed'
    )
ON CONFLICT (user_id) DO NOTHING;

-- Sample content variants
INSERT INTO content_variants (content_id, chapter_id, section_id, variant_type, target_audience, content, is_ai_generated)
VALUES 
    ('ch1_intro_beginner', 'chapter_1', 'introduction', 'beginner',
     '{"experience_level": "beginner", "software_categories": ["web_development"], "hardware_categories": ["iot_devices"]}',
     '{"text": "Welcome to the world of AI and robotics! This chapter will introduce you to the basic concepts in simple terms.", "complexity_level": "beginner", "code_examples": [{"language": "python", "code": "print(\"Hello, AI World!\")", "explanation": "This simple Python command prints a greeting message."}]}',
     false
    ),
    ('ch1_intro_advanced', 'chapter_1', 'introduction', 'advanced',
     '{"experience_level": "advanced", "software_categories": ["machine_learning"], "hardware_categories": ["embedded_systems"]}',
     '{"text": "This chapter explores advanced AI architectures and their implementation in embedded systems.", "complexity_level": "advanced", "code_examples": [{"language": "python", "code": "import tensorflow as tf\nmodel = tf.keras.Sequential([tf.keras.layers.Dense(128, activation=\"relu\")])", "explanation": "Advanced TensorFlow model initialization for embedded deployment."}]}',
     false
    )
ON CONFLICT (content_id) DO NOTHING;

-- Create a view for personalization analytics
CREATE OR REPLACE VIEW personalization_analytics AS
SELECT 
    DATE_TRUNC('day', created_at) as date,
    COUNT(*) as total_requests,
    COUNT(*) FILTER (WHERE cache_hit = true) as cache_hits,
    AVG(generation_time_ms) as avg_generation_time,
    COUNT(DISTINCT user_id) as unique_users
FROM personalization_logs
GROUP BY DATE_TRUNC('day', created_at)
ORDER BY date DESC;

-- Grant necessary permissions (adjust as needed for your setup)
-- GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO your_app_user;
-- GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO your_app_user;