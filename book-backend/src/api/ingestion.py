from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional

from src.services.ingestion_service import BookIngestionService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
from qdrant_client import QdrantClient
import os

router = APIRouter()

def get_qdrant_client() -> QdrantClient:
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variable not set")
    return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

def get_embedding_service() -> EmbeddingService:
    return EmbeddingService()

def get_qdrant_service(qdrant_client: QdrantClient = Depends(get_qdrant_client)) -> QdrantService:
    return QdrantService(qdrant_client)

def get_ingestion_service(
    embedding_service: EmbeddingService = Depends(get_embedding_service),
    qdrant_service: QdrantService = Depends(get_qdrant_service)
) -> BookIngestionService:
    return BookIngestionService(embedding_service, qdrant_service)


class IngestBookRequest(BaseModel):
    book_path: str

class IngestDocsRequest(BaseModel):
    docs_path: str

class IngestResponse(BaseModel):
    status: str
    message: str
    documents_indexed: int
    chunks_created: Optional[int] = None


@router.post("/ingest/book", response_model=IngestResponse)
async def ingest_book_endpoint(
    request: IngestBookRequest,
    ingestion_service: BookIngestionService = Depends(get_ingestion_service)
):
    """
    Ingest a single book file (PDF, DOCX, EPUB, TXT, MD)
    """
    try:
        result = await ingestion_service.ingest_book_content(request.book_path)
        
        if result["status"] == "error":
            raise HTTPException(status_code=400, detail=result["message"])
        
        return IngestResponse(
            status=result["status"],
            message=result["message"],
            documents_indexed=result["documents_indexed"],
            chunks_created=result.get("chunks_created")
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


@router.post("/ingest/markdown", response_model=IngestResponse)
async def ingest_markdown_docs_endpoint(
    request: IngestDocsRequest,
    ingestion_service: BookIngestionService = Depends(get_ingestion_service)
):
    """
    Ingest Docusaurus markdown documentation from a directory
    """
    try:
        result = await ingestion_service.ingest_markdown_docs(request.docs_path)
        
        if result["status"] == "error":
            raise HTTPException(status_code=400, detail=result["message"])
        
        return IngestResponse(
            status=result["status"],
            message=result["message"],
            documents_indexed=result["documents_indexed"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Markdown ingestion failed: {str(e)}")


@router.post("/ingest/docusaurus-docs")
async def ingest_docusaurus_docs_endpoint(
    ingestion_service: BookIngestionService = Depends(get_ingestion_service)
):
    """
    Convenience endpoint to ingest the Docusaurus docs from the standard location
    """
    docs_path = "../book-frontend/docs"  # Relative to book-backend directory
    
    try:
        result = await ingestion_service.ingest_markdown_docs(docs_path)
        
        if result["status"] == "error":
            raise HTTPException(status_code=400, detail=result["message"])
        
        return IngestResponse(
            status=result["status"],
            message=result["message"],
            documents_indexed=result["documents_indexed"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Docusaurus docs ingestion failed: {str(e)}")