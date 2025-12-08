"""
Database service for SQLAlchemy/SQLModel engine and session management
"""

from sqlalchemy import create_engine, MetaData
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import StaticPool
from app.core.config import get_settings
import logging

logger = logging.getLogger(__name__)

# Get settings
settings = get_settings()

# Create SQLAlchemy engine
engine = create_engine(
    settings.database_url,
    poolclass=StaticPool,
    connect_args={
        "check_same_thread": False,
        "sslmode": "require"
    } if "sqlite" not in settings.database_url else {"check_same_thread": False},
    echo=settings.debug
)

# Create SessionLocal class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create Base class for models
Base = declarative_base()

# Metadata for table creation
metadata = MetaData()


def get_db() -> Session:
    """
    Database session dependency for FastAPI
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def create_tables():
    """Create all tables"""
    try:
        Base.metadata.create_all(bind=engine)
        logger.info("Database tables created successfully")
    except Exception as e:
        logger.error(f"Error creating database tables: {e}")
        raise


def drop_tables():
    """Drop all tables (use with caution)"""
    try:
        Base.metadata.drop_all(bind=engine)
        logger.info("Database tables dropped successfully")
    except Exception as e:
        logger.error(f"Error dropping database tables: {e}")
        raise


class DatabaseService:
    """Database service class for advanced operations"""
    
    def __init__(self):
        self.engine = engine
        self.SessionLocal = SessionLocal
    
    def get_session(self) -> Session:
        """Get a new database session"""
        return self.SessionLocal()
    
    def execute_raw_sql(self, sql: str, params: dict = None):
        """Execute raw SQL query"""
        with self.engine.connect() as connection:
            result = connection.execute(sql, params or {})
            return result.fetchall()
    
    def check_connection(self) -> bool:
        """Check if database connection is working"""
        try:
            with self.engine.connect() as connection:
                connection.execute("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Database connection check failed: {e}")
            return False
    
    def get_table_info(self, table_name: str) -> dict:
        """Get information about a specific table"""
        try:
            with self.engine.connect() as connection:
                # Get column information
                result = connection.execute(f"""
                    SELECT column_name, data_type, is_nullable
                    FROM information_schema.columns
                    WHERE table_name = '{table_name}'
                    ORDER BY ordinal_position
                """)
                columns = result.fetchall()
                
                # Get row count
                count_result = connection.execute(f"SELECT COUNT(*) FROM {table_name}")
                row_count = count_result.fetchone()[0]
                
                return {
                    "table_name": table_name,
                    "columns": [
                        {
                            "name": col[0],
                            "type": col[1],
                            "nullable": col[2] == "YES"
                        }
                        for col in columns
                    ],
                    "row_count": row_count
                }
        except Exception as e:
            logger.error(f"Error getting table info for {table_name}: {e}")
            return {"error": str(e)}


# Global database service instance
db_service = DatabaseService()