#!/usr/bin/env python3
"""
CLI script for ingesting content into the RAG system
Usage:
    python scripts/ingest_content.py --book /path/to/book.pdf
    python scripts/ingest_content.py --docs /path/to/docs/
    python scripts/ingest_content.py --docusaurus-docs
"""


import asyncio
import argparse
import sys
import os
from pathlib import Path

# Add the root project directory to the Python path
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


from src.services.ingestion_service import BookIngestionService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def get_qdrant_client() -> QdrantClient:
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variable not set")
    return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)


async def main():
    parser = argparse.ArgumentParser(description="Ingest content into RAG system")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--book", help="Path to book file (PDF, DOCX, EPUB, TXT, MD)")
    group.add_argument("--docs", help="Path to documentation directory")
    group.add_argument("--docusaurus-docs", action="store_true", 
                      help="Ingest Docusaurus docs from ../book-frontend/docs")
    
    args = parser.parse_args()
    
    try:
        # Initialize services
        print("Initializing services...")
        qdrant_client = get_qdrant_client()
        embedding_service = EmbeddingService()
        qdrant_service = QdrantService(qdrant_client)
        ingestion_service = BookIngestionService(embedding_service, qdrant_service)
        
        if args.book:
            print(f"Ingesting book: {args.book}")
            result = await ingestion_service.ingest_book_content(args.book)
            
        elif args.docs:
            print(f"Ingesting documentation from: {args.docs}")
            result = await ingestion_service.ingest_markdown_docs(args.docs)
            
        elif args.docusaurus_docs:
            docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "book-frontend", "docs")
            docs_path = os.path.abspath(docs_path)
            print(f"Ingesting Docusaurus docs from: {docs_path}")
            result = await ingestion_service.ingest_markdown_docs(docs_path)
        
        # Print results
        print(f"\nIngestion Results:")
        print(f"Status: {result['status']}")
        print(f"Message: {result['message']}")
        print(f"Documents indexed: {result['documents_indexed']}")
        if 'chunks_created' in result:
            print(f"Chunks created: {result['chunks_created']}")
        
        if result['status'] == 'error':
            sys.exit(1)
        else:
            print("\nIngestion completed successfully!")
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())