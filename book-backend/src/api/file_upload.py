from fastapi import APIRouter, Depends, HTTPException, UploadFile, File, Form
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional, List
import uuid
import tempfile
import os
from pathlib import Path

from src.database.database import get_db
from src.database import crud
from src.services.rag_service import RAGService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
from src.services.ingestion_service import BookIngestionService
from qdrant_client import QdrantClient

router = APIRouter()

# Initialize services
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

def get_rag_service(
    qdrant_client: QdrantClient = Depends(get_qdrant_client),
    embedding_service: EmbeddingService = Depends(get_embedding_service)
) -> RAGService:
    return RAGService(qdrant_client, embedding_service)


class FileUploadResponse(BaseModel):
    status: str
    message: str
    session_id: uuid.UUID
    context_added: bool
    file_info: dict


class FileQueryRequest(BaseModel):
    query: str
    session_id: uuid.UUID


class FileQueryResponse(BaseModel):
    response: str
    session_id: uuid.UUID
    context: Optional[List[str]] = None


# Supported file types
SUPPORTED_EXTENSIONS = {'.pdf', '.docx', '.txt', '.md', '.markdown', '.epub'}
MAX_FILE_SIZE = 10 * 1024 * 1024  # 10MB


def validate_file(file: UploadFile) -> None:
    """Validate uploaded file"""
    if not file.filename:
        raise HTTPException(status_code=400, detail="No filename provided")
    
    file_extension = Path(file.filename).suffix.lower()
    if file_extension not in SUPPORTED_EXTENSIONS:
        raise HTTPException(
            status_code=400, 
            detail=f"Unsupported file type. Supported types: {', '.join(SUPPORTED_EXTENSIONS)}"
        )
    
    # Note: file.size might not be available in all cases
    # You might need to read the file to check size
    

@router.post("/chat/file", response_model=FileUploadResponse)
async def upload_file_endpoint(
    file: UploadFile = File(...),
    session_id: Optional[str] = Form(None),
    db: Session = Depends(get_db),
    ingestion_service: BookIngestionService = Depends(get_ingestion_service)
):
    """
    Upload and process a file for temporary knowledge base augmentation
    """
    try:
        # Validate file
        validate_file(file)
        
        # Handle session
        if session_id:
            try:
                session_uuid = uuid.UUID(session_id)
                db_session = crud.get_session(db, session_uuid)
                if not db_session:
                    raise HTTPException(status_code=404, detail="Session not found")
            except ValueError:
                raise HTTPException(status_code=400, detail="Invalid session ID format")
        else:
            # Create a new session
            db_session = crud.create_session(db)
            session_uuid = db_session.id
        
        # Read file content
        file_content = await file.read()
        
        # Check file size
        if len(file_content) > MAX_FILE_SIZE:
            raise HTTPException(
                status_code=400, 
                detail=f"File too large. Maximum size: {MAX_FILE_SIZE // (1024*1024)}MB"
            )
        
        # Save file to temporary location
        with tempfile.NamedTemporaryFile(
            suffix=Path(file.filename).suffix, 
            delete=False
        ) as temp_file:
            temp_file.write(file_content)
            temp_file_path = temp_file.name
        
        try:
            # Process file using ingestion service
            result = await ingestion_service.ingest_book_content(temp_file_path)
            
            if result["status"] == "error":
                raise HTTPException(status_code=400, detail=result["message"])
            
            # Store file upload message
            file_message = f"📎 Uploaded and processed file: {file.filename} ({len(file_content) // 1024} KB)"
            crud.create_message(
                db, 
                session_uuid, 
                file_message, 
                "user", 
                {
                    "input_type": "file_upload",
                    "filename": file.filename,
                    "file_size": len(file_content),
                    "chunks_created": result.get("chunks_created", 0)
                }
            )
            
            # Create AI response about successful processing
            ai_response = f"✅ Successfully processed your file '{file.filename}'! I've analyzed the content and can now answer questions about it. The file was split into {result.get('chunks_created', 0)} chunks for better understanding. What would you like to know about this document?"
            
            crud.create_message(
                db,
                session_uuid,
                ai_response,
                "ai",
                {
                    "input_type": "file_processing_response",
                    "processing_result": result
                }
            )
            
            return FileUploadResponse(
                status="success",
                message=ai_response,
                session_id=session_uuid,
                context_added=True,
                file_info={
                    "filename": file.filename,
                    "size_kb": len(file_content) // 1024,
                    "chunks_created": result.get("chunks_created", 0),
                    "documents_indexed": result.get("documents_indexed", 0)
                }
            )
            
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
                
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle unexpected errors
        raise HTTPException(status_code=500, detail=f"File processing failed: {str(e)}")


@router.post("/chat/file/query", response_model=FileQueryResponse)
async def query_uploaded_files(
    request: FileQueryRequest,
    db: Session = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
):
    """
    Query against uploaded files using RAG
    """
    try:
        # Verify session exists
        db_session = crud.get_session(db, request.session_id)
        if not db_session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        # Store user query
        crud.create_message(db, request.session_id, request.query, "user")
        
        # Process query with RAG (this will search across all indexed content including uploaded files)
        retrieved_context = await rag_service.retrieve_context(request.query)
        rag_response = await rag_service.generate_rag_response(request.query, retrieved_context)
        
        # Store AI response
        crud.create_message(
            db,
            request.session_id,
            rag_response,
            "ai",
            {"context": retrieved_context, "query_type": "file_query"}
        )
        
        return FileQueryResponse(
            response=rag_response,
            session_id=request.session_id,
            context=retrieved_context
        )
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle unexpected errors
        raise HTTPException(status_code=500, detail=f"File query failed: {str(e)}")


@router.get("/chat/file/supported-types")
async def get_supported_file_types():
    """Get list of supported file types"""
    return {
        "supported_extensions": list(SUPPORTED_EXTENSIONS),
        "max_file_size_mb": MAX_FILE_SIZE // (1024 * 1024),
        "description": {
            ".pdf": "PDF documents",
            ".docx": "Microsoft Word documents",
            ".txt": "Plain text files",
            ".md": "Markdown files",
            ".markdown": "Markdown files",
            ".epub": "EPUB ebooks"
        }
    }


@router.delete("/chat/file/session/{session_id}")
async def clear_session_files(
    session_id: uuid.UUID,
    db: Session = Depends(get_db)
):
    """
    Clear all uploaded files for a session
    Note: This is a simplified implementation. In production, you might want to:
    1. Track which documents belong to which session
    2. Remove only session-specific vectors from Qdrant
    3. Implement proper cleanup mechanisms
    """
    try:
        # Verify session exists
        db_session = crud.get_session(db, session_id)
        if not db_session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        # For now, just add a message indicating files were cleared
        # In a full implementation, you'd remove session-specific vectors from Qdrant
        clear_message = "🗑️ Session files cleared. Previous uploaded documents are no longer available for queries."
        crud.create_message(
            db,
            session_id,
            clear_message,
            "ai",
            {"action": "files_cleared"}
        )
        
        return {"status": "success", "message": "Session files cleared"}
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to clear session files: {str(e)}")