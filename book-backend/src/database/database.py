import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv

load_dotenv()

NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

if not NEON_DATABASE_URL:
    # For development/testing, use SQLite if Neon URL is not set
    NEON_DATABASE_URL = "sqlite:///./test.db"
    print("Warning: NEON_DATABASE_URL not set, using SQLite for development")

# Configure engine with appropriate settings
if NEON_DATABASE_URL.startswith("sqlite"):
    engine = create_engine(NEON_DATABASE_URL, connect_args={"check_same_thread": False})
else:
    engine = create_engine(NEON_DATABASE_URL, pool_pre_ping=True)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
