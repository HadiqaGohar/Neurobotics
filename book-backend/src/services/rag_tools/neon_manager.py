import os
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy import Column, Integer, Text, DateTime, func, select
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy.dialects.postgresql import ARRAY # For vector type, if direct support isn't there
from pgvector.sqlalchemy import Vector # For pgvector integration
from typing import List


# Define the base for declarative models
Base = declarative_base()

class BookChunk(Base):
    __tablename__ = 'book_chunks'

    id = Column(Integer, primary_key=True, autoincrement=True)
    qdrant_id = Column(Text, unique=True, nullable=False)
    module = Column(Text, nullable=False)
    chapter = Column(Text, nullable=False)
    text = Column(Text, nullable=False)
    embedding = Column(Vector(1536), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __repr__(self):
        return f"<BookChunk(id={self.id}, module='{self.module}', chapter='{self.chapter}')>"

class NeonManager:
    def __init__(self, db_url: str = os.getenv("NEON_DB_URL")):
        if not db_url:
            raise ValueError("NEON_DB_URL environment variable must be set.")
        self.db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
        self.engine = create_async_engine(self.db_url, echo=False)
        self.AsyncSessionLocal = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=self.engine,
            class_=AsyncSession
        )

    async def _ensure_vector_extension(self):
        async with self.engine.begin() as conn:
            await conn.execute(Text("CREATE EXTENSION IF NOT EXISTS vector;"))
            print("Ensured 'vector' extension exists.")

    async def _ensure_table_exists(self):
        async with self.engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
            print("Ensured 'book_chunks' table exists.")

    async def initialize(self):
        await self._ensure_vector_extension()
        await self._ensure_table_exists()

    async def insert_chunk(self,
                           qdrant_id: str,
                           module: str,
                           chapter: str,
                           text_content: str,
                           embedding_vector: List[float]):
        async with self.AsyncSessionLocal() as session:
            new_chunk = BookChunk(
                qdrant_id=qdrant_id,
                module=module,
                chapter=chapter,
                text=text_content,
                embedding=embedding_vector
            )
            session.add(new_chunk)
            await session.commit()
            await session.refresh(new_chunk)
            print(f"Inserted chunk with qdrant_id: {qdrant_id}")
            return new_chunk

    async def insert_chunks(self, chunks_data: List[Dict[str, Any]]):
        async with self.AsyncSessionLocal() as session:
            new_chunks = []
            for data in chunks_data:
                new_chunk = BookChunk(
                    qdrant_id=data["qdrant_id"],
                    module=data["module"],
                    chapter=data["chapter"],
                    text=data["text"],
                    embedding=data["embedding"]
                )
                session.add(new_chunk)
                new_chunks.append(new_chunk)
            await session.commit()
            for chunk in new_chunks:
                await session.refresh(chunk)
            print(f"Inserted {len(new_chunks)} chunks into Neon.")
            return new_chunks
            
    async def get_chunk_by_qdrant_id(self, qdrant_id: str):
        async with self.AsyncSessionLocal() as session:
            chunk = await session.execute(
                select(BookChunk).filter_by(qdrant_id=qdrant_id)
            )
            return chunk.scalar_one_or_none()


if __name__ == '__main__':
    # Set dummy environment variable for testing
    os.environ["NEON_DB_URL"] = "postgresql://user:password@host:port/database" # Replace with your Neon connection string

    async def main():
        try:
            manager = NeonManager()
            await manager.initialize()

            # Dummy data for testing
            dummy_chunk_data = {
                "qdrant_id": str(uuid.uuid4()),
                "module": "Module X",
                "chapter": "Chapter Y",
                "text": "This is a test text for Neon database insertion.",
                "embedding": [0.01] * 1536
            }
            
            print("\nTesting single chunk insertion...")
            inserted_chunk = await manager.insert_chunk(**dummy_chunk_data)
            print(f"Single inserted chunk: {inserted_chunk}")

            print("\nTesting batch chunk insertion...")
            batch_data = [
                {
                    "qdrant_id": str(uuid.uuid4()),
                    "module": "Module X",
                    "chapter": "Chapter Z",
                    "text": "Another test text for batch insertion.",
                    "embedding": [0.02] * 1536
                },
                {
                    "qdrant_id": str(uuid.uuid4()),
                    "module": "Module Y",
                    "chapter": "Chapter A",
                    "text": "Third test text for batch insertion.",
                    "embedding": [0.03] * 1536
                }
            ]
            inserted_chunks = await manager.insert_chunks(batch_data)
            for chunk in inserted_chunks:
                print(f"Batch inserted chunk: {chunk}")
            
            # Test retrieval
            retrieved_chunk = await manager.get_chunk_by_qdrant_id(dummy_chunk_data["qdrant_id"])
            print(f"\nRetrieved chunk by qdrant_id: {retrieved_chunk}")


        except ValueError as e:
            print(f"Configuration error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    # Run the async main function
    asyncio.run(main())

    # Clean up dummy environment variable (optional)
    del os.environ["NEON_DB_URL"]
