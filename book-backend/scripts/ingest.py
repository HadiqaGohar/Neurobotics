import os
import asyncio
from dotenv import load_dotenv
from typing import List, Dict, Any

# Assuming these are in book-backend/src/services/rag_tools/
from src.services.rag_tools.text_loader import TextLoader
from src.services.rag_tools.chunk_text import TextChunker
from src.services.rag_tools.embedding_generator import EmbeddingGenerator
from src.services.rag_tools.qdrant_manager import QdrantManager
from src.services.rag_tools.neon_manager import NeonManager

# Load environment variables
load_dotenv()

async def ingest_book_content(modules_base_path: str = "modules"):
    print("Starting book content ingestion process...")

    # Initialize tools
    try:
        text_loader = TextLoader(base_path=modules_base_path)
        text_chunker = TextChunker()
        embedding_generator = EmbeddingGenerator()
        qdrant_manager = QdrantManager()
        neon_manager = NeonManager()
        await neon_manager.initialize() # Ensure Neon table and extension are ready
    except ValueError as e:
        print(f"Tool initialization error: {e}")
        return

    all_files_to_process = []
    # Step 1: Load all modules and identify files
    print(f"Scanning for modules in: {modules_base_path}")
    for root, _, files in os.walk(modules_base_path):
        for file in files:
            if file.lower().endswith(('.txt', '.md', '.pdf')):
                all_files_to_process.append(os.path.join(root, file))
    
    if not all_files_to_process:
        print("No book content files found in the specified modules path. Exiting.")
        return

    print(f"Found {len(all_files_to_process)} files to process.")
    
    processed_chunks_count = 0
    for file_path in all_files_to_process:
        print(f"\nProcessing file: {file_path}")
        try:
            # Step 1 (cont.): Read text
            loaded_data = text_loader.load_text(file_path)
            raw_text = loaded_data["text"]
            module_name = loaded_data["module"]
            chapter_name = loaded_data["chapter"]

            if not raw_text.strip():
                print(f"Skipping empty file: {file_path}")
                continue

            # Step 2: Chunk text
            chunks = text_chunker.chunk_text(raw_text)
            if not chunks:
                print(f"No chunks generated for file: {file_path}. Skipping.")
                continue
            print(f"Generated {len(chunks)} chunks.")

            # Step 3: Generate embeddings
            embeddings = embedding_generator.generate_embeddings(chunks)
            if not embeddings or len(embeddings) != len(chunks):
                print(f"Failed to generate embeddings for all chunks in {file_path}. Skipping.")
                continue
            print(f"Generated {len(embeddings)} embeddings.")

            # Prepare metadata for Qdrant
            metadata_list = [{"module": module_name, "chapter": chapter_name} for _ in chunks]

            # Step 4: Push to Qdrant
            qdrant_ids = qdrant_manager.upsert_chunks(chunks, embeddings, metadata_list)
            print(f"Pushed {len(qdrant_ids)} chunks to Qdrant.")

            # Step 5: Push to Neon
            neon_data = []
            for i, q_id in enumerate(qdrant_ids):
                neon_data.append({
                    "qdrant_id": q_id,
                    "module": module_name,
                    "chapter": chapter_name,
                    "text": chunks[i],
                    "embedding": embeddings[i]
                })
            await neon_manager.insert_chunks(neon_data)
            print(f"Pushed {len(neon_data)} chunks to Neon.")
            processed_chunks_count += len(chunks)

        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            # Continue to next file even if one fails

    print(f"\nIngestion process completed. Total chunks processed: {processed_chunks_count}")

if __name__ == '__main__':
    # You might need to adjust the path to the 'modules' directory
    # relative to where this script is executed.
    # For example, if 'ingest.py' is in 'book-backend/scripts' and 'modules' is at the project root:
    # modules_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../modules'))
    
    # For now, assuming 'modules' is a sibling of 'book-backend'
    # Current working directory: /home/hadiqa/Documents/SpecifyPlus/Hackthon/Book
    # modules_base_path = "modules" 
    
    # If the script is run from book-backend/scripts, then relative path would be:
    modules_base_path = "../../modules"

    asyncio.run(ingest_book_content(modules_base_path))