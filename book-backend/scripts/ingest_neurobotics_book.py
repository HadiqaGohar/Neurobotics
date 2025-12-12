import sys
import os

# Add project root directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
# Script is in 'scripts' folder, so navigate up one level to reach book-backend folder
book_backend_path = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.insert(0, book_backend_path)

print(f"Added to sys.path: {book_backend_path}") # Debugging ke liye

import asyncio
import uuid
import logging
from typing import List

from qdrant_client import models
from sqlalchemy.orm import Session as DBSession

# Corrected imports based on user feedback: assuming 'src' is directly importable from the path added.
# If 'book_backend_path' is added to sys.path, then imports like 'book_backend.src...' should work if book_backend is a package.
# However, user stated 'from src.database...' is correct, implying 'src' is the root.
# Let's try to directly import from src after adding book-backend to sys.path.
# If this fails, we might need to add 'book-backend/src' to sys.path instead of 'book-backend'.
# For now, attempting to correct the import statement itself as suggested by the user.

from src.database.database import get_db
from src.database import crud
from src.database.models import Document, DocumentChunk
from src.services.embedding_service import EmbeddingService
from src.qdrant.client import QdrantManager

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Constants
NEUROBOTICS_BOOK_PATH = "NEUROBOTICS_BOOK.txt"
QDRANT_COLLECTION_NAME = "book_content"
EMBEDDING_DIMENSION = 384  # all-MiniLM-L6-v2 produces 384-dimensional embeddings

async def ingest_neurobotics_book():
    logger.info(f"Starting ingestion process for {NEUROBOTICS_BOOK_PATH}")

    # 1. Load NEUROBOTICS_BOOK.txt
    try:
        with open(NEUROBOTICS_BOOK_PATH, 'r', encoding='utf-8') as f:
            book_content = f.read()
        logger.info(f"Successfully loaded {NEUROBOTICS_BOOK_PATH}")
    except FileNotFoundError:
        logger.error(f"Error: {NEUROBOTICS_BOOK_PATH} not found.")
        return
    except Exception as e:
        logger.error(f"Error reading {NEUROBOTICS_BOOK_PATH}: {e}")
        return

    # 2. Initialize EmbeddingService
    embedding_service = EmbeddingService()
    logger.info("EmbeddingService initialized.")

    # 3. Chunk the document
    chunks: List[str] = await embedding_service.chunk_document(book_content)
    logger.info(f"Document chunked into {len(chunks)} pieces.")

    # 4. Initialize Qdrant Manager
    qdrant_manager = QdrantManager()
    qdrant_client = qdrant_manager.get_client()
    logger.info("QdrantManager initialized.")

    # 5. Create Qdrant collection if it doesn't exist or re-create
    try:
        # Note: recreate_collection is deprecated, but used here as per original script.
        # For production, consider collection_exists and create_collection.
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=EMBEDDING_DIMENSION, distance=models.Distance.COSINE),
        )
        logger.info(f"Qdrant collection '{QDRANT_COLLECTION_NAME}' created/recreated.")
    except Exception as e:
        logger.error(f"Error creating/recreating Qdrant collection: {e}")
        return

    # 6. Connect to Database
    db_generator = get_db()
    db: DBSession = next(db_generator)
    try:
        # 7. Create Document entry
        document_title = "NEUROBOTICS_BOOK"
        document_source_uri = f"file:///{NEUROBOTICS_BOOK_PATH}"
        db_document = crud.create_document(db, title=document_title, source_uri=document_source_uri, total_chunks=len(chunks))
        logger.info(f"Document '{document_title}' recorded in DB with ID: {db_document.id}")

        # Prepare data for Qdrant upsert
        qdrant_ids = []
        qdrant_vectors = []
        qdrant_payloads = []
        
        # Prepare data for DB
        db_chunks_to_create = []

        for i, chunk_content in enumerate(chunks):
            # Generate unique ID for Qdrant and DB
            embedding_id = str(uuid.uuid4())

            # Generate embedding for the chunk
            # The embedding_service.generate_embeddings method expects a list of strings.
            chunk_embedding = await embedding_service.generate_embeddings([chunk_content])
            
            if not chunk_embedding or not chunk_embedding[0]:
                logger.warning(f"Skipping chunk {i} due to failed embedding generation.")
                continue

            qdrant_ids.append(embedding_id)
            qdrant_vectors.append(chunk_embedding[0])
            qdrant_payloads.append({"embedding_id": embedding_id, "document_id": str(db_document.id), "chunk_index": i})
            
            db_chunks_to_create.append({
                "document_id": db_document.id,
                "chunk_index": i,
                "content": chunk_content,
                "embedding_id": embedding_id,
                "extra_data": {"source_uri": document_source_uri}
            })
            
            if (i + 1) % 100 == 0:  # Upsert in batches of 100
                logger.info(f"Upserting batch of {len(qdrant_ids)} vectors to Qdrant...")
                qdrant_manager.upsert_vectors(
                    collection_name=QDRANT_COLLECTION_NAME,
                    ids=qdrant_ids,
                    vectors=qdrant_vectors,
                    payloads=qdrant_payloads
                )
                logger.info(f"Adding batch of {len(db_chunks_to_create)} chunks to Postgres...")
                for chunk_data in db_chunks_to_create:
                    crud.create_document_chunk(db, **chunk_data)
                db.commit() # Commit after each batch
                
                # Clear batches
                qdrant_ids = []
                qdrant_vectors = []
                qdrant_payloads = []
                db_chunks_to_create = []

        # Upsert any remaining vectors
        if qdrant_ids:
            logger.info(f"Upserting final batch of {len(qdrant_ids)} vectors to Qdrant...")
            qdrant_manager.upsert_vectors(
                collection_name=QDRANT_COLLECTION_NAME,
                ids=qdrant_ids,
                vectors=qdrant_vectors,
                payloads=qdrant_payloads
            )
        if db_chunks_to_create:
            logger.info(f"Adding final batch of {len(db_chunks_to_create)} chunks to Postgres...")
            for chunk_data in db_chunks_to_create:
                crud.create_document_chunk(db, **chunk_data)
            db.commit() # Final commit

        logger.info("Ingestion process completed successfully!")

    except Exception as e:
        logger.error(f"An error occurred during ingestion: {e}", exc_info=True)
        db.rollback() # Rollback on error
    finally:
        try:
            next(db_generator) # Close the session
        except StopIteration:
            pass

if __name__ == "__main__":
    # Ensure environment variables for Qdrant are set
    # For local development, you might set them in a .env file and load it
    # from dotenv import load_dotenv
    # load_dotenv()

    # Run the ingestion
    asyncio.run(ingest_neurobotics_book())