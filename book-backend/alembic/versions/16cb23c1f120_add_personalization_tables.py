"""Add personalization tables

Revision ID: 16cb23c1f120
Revises: book_backend_001
Create Date: 2025-12-07 22:05:12.465772

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '16cb23c1f120'
down_revision: Union[str, Sequence[str], None] = 'book_backend_001'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # Create user_preferences table
    op.create_table(
        'user_preferences',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('software_background', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('hardware_background', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('content_complexity', sa.String(length=50), nullable=False, server_default='moderate'),
        sa.Column('explanation_depth', sa.String(length=50), nullable=False, server_default='standard'),
        sa.Column('example_style', sa.String(length=50), nullable=False, server_default='practical'),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('user_id')
    )
    
    # Create indexes for user_preferences
    op.create_index('ix_user_preferences_id', 'user_preferences', ['id'])
    op.create_index('ix_user_preferences_user_id', 'user_preferences', ['user_id'])
    
    # Create content_variants table
    op.create_table(
        'content_variants',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('content_id', sa.String(length=255), nullable=False),
        sa.Column('chapter_id', sa.String(length=255), nullable=False),
        sa.Column('section_id', sa.String(length=255), nullable=True),
        sa.Column('variant_type', sa.String(length=50), nullable=False),
        sa.Column('target_audience', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('content', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('content_metadata', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('is_ai_generated', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('quality_score', sa.Integer(), nullable=True),
        sa.Column('usage_count', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('user_preferences_id', sa.Integer(), nullable=True),
        sa.ForeignKeyConstraint(['user_preferences_id'], ['user_preferences.id'], ),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('content_id')
    )
    
    # Create indexes for content_variants
    op.create_index('ix_content_variants_id', 'content_variants', ['id'])
    op.create_index('ix_content_variants_content_id', 'content_variants', ['content_id'])
    op.create_index('ix_content_variants_chapter_id', 'content_variants', ['chapter_id'])
    op.create_index('ix_content_variants_section_id', 'content_variants', ['section_id'])
    
    # Create personalization_logs table
    op.create_table(
        'personalization_logs',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.String(length=255), nullable=False),
        sa.Column('section_id', sa.String(length=255), nullable=True),
        sa.Column('requested_complexity', sa.String(length=50), nullable=True),
        sa.Column('applied_personalization', sa.JSON(), nullable=False, server_default='{}'),
        sa.Column('cache_hit', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('generation_time_ms', sa.Integer(), nullable=False, server_default='0'),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    
    # Create indexes for personalization_logs
    op.create_index('ix_personalization_logs_id', 'personalization_logs', ['id'])
    op.create_index('ix_personalization_logs_user_id', 'personalization_logs', ['user_id'])
    op.create_index('ix_personalization_logs_chapter_id', 'personalization_logs', ['chapter_id'])
    op.create_index('ix_personalization_logs_section_id', 'personalization_logs', ['section_id'])


def downgrade() -> None:
    """Downgrade schema."""
    # Drop indexes and tables in reverse order
    op.drop_index('ix_personalization_logs_section_id', table_name='personalization_logs')
    op.drop_index('ix_personalization_logs_chapter_id', table_name='personalization_logs')
    op.drop_index('ix_personalization_logs_user_id', table_name='personalization_logs')
    op.drop_index('ix_personalization_logs_id', table_name='personalization_logs')
    op.drop_table('personalization_logs')
    
    op.drop_index('ix_content_variants_section_id', table_name='content_variants')
    op.drop_index('ix_content_variants_chapter_id', table_name='content_variants')
    op.drop_index('ix_content_variants_content_id', table_name='content_variants')
    op.drop_index('ix_content_variants_id', table_name='content_variants')
    op.drop_table('content_variants')
    
    op.drop_index('ix_user_preferences_user_id', table_name='user_preferences')
    op.drop_index('ix_user_preferences_id', table_name='user_preferences')
    op.drop_table('user_preferences')
