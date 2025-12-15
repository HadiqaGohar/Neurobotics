
import os
import uuid
from typing import List, Optional
from pathlib import Path
import PyPDF2
import docx
import ebooklib
from ebooklib import epub
from sqlalchemy.orm import Session

from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
from src.database import crud
from src.database.database import get_db


class BookIngestionService:
    def __init__(self, embedding_service: EmbeddingService, qdrant_service: QdrantService):
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        self.collection_name = "book_content"

    def extract_text_from_pdf(self, file_path: str) -> str:
        """Extract text from PDF file"""
        text = ""
        try:
            with open(file_path, 'rb') as file:
                pdf_reader = PyPDF2.PdfReader(file)
                for page in pdf_reader.pages:
                    text += page.extract_text() + "\n"
        except Exception as e:
            print(f"Error reading PDF {file_path}: {e}")
        return text

    def extract_text_from_docx(self, file_path: str) -> str:
        """Extract text from DOCX file"""
        text = ""
        try:
            doc = docx.Document(file_path)
            for paragraph in doc.paragraphs:
                text += paragraph.text + "\n"
        except Exception as e:
            print(f"Error reading DOCX {file_path}: {e}")
        return text

    def extract_text_from_epub(self, file_path: str) -> str:
        """Extract text from EPUB file"""
        text = ""
        try:
            book = epub.read_epub(file_path)
            for item in book.get_items():
                if item.get_type() == ebooklib.ITEM_DOCUMENT:
                    content = item.get_content().decode('utf-8')
                    # Basic HTML tag removal (you might want to use BeautifulSoup for better parsing)
                    import re
                    clean_text = re.sub('<[^<]+?>', '', content)
                    text += clean_text + "\n"
        except Exception as e:
            print(f"Error reading EPUB {file_path}: {e}")
        return text

    def extract_text_from_txt(self, file_path: str) -> str:
        """Extract text from TXT file"""
        text = ""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                text = file.read()
        except Exception as e:
            print(f"Error reading TXT {file_path}: {e}")
        return text

    def extract_text_from_markdown(self, file_path: str) -> str:
        """Extract text from Markdown file"""
        text = ""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
                # Basic markdown cleanup - remove headers, links, etc.
                import re
                # Remove markdown headers
                content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
                # Remove markdown links
                content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)
                # Remove markdown bold/italic
                content = re.sub(r'\*\*([^\*]+)\*\*', r'\1', content)
                content = re.sub(r'\*([^\*]+)\*', r'\1', content)
                text = content
        except Exception as e:
            print(f"Error reading Markdown {file_path}: {e}")
        return text

    def extract_text_from_file(self, file_path: str) -> str:
        """Extract text from various file formats"""
        file_extension = Path(file_path).suffix.lower()
        
        if file_extension == '.pdf':
            return self.extract_text_from_pdf(file_path)
        elif file_extension == '.docx':
            return self.extract_text_from_docx(file_path)
        elif file_extension == '.epub':
            return self.extract_text_from_epub(file_path)
        elif file_extension == '.txt':
            return self.extract_text_from_txt(file_path)
        elif file_extension in ['.md', '.markdown']:
            return self.extract_text_from_markdown(file_path)
        else:
            print(f"Unsupported file format: {file_extension}")
            return ""

    async def ingest_book_content(self, book_path: str) -> dict:
        """
        Ingest book content from a file path
        Returns: dict with status and documents_indexed count
        """
        if not os.path.exists(book_path):
            return {"status": "error", "message": "File not found", "documents_indexed": 0}

        # Extract text from the book
        text_content = self.extract_text_from_file(book_path)
        if not text_content.strip():
            return {"status": "error", "message": "No text content extracted", "documents_indexed": 0}

        # Get database session
        db_generator = get_db()
        db = next(db_generator)
        
        try:
            # Create document record
            book_title = Path(book_path).stem
            document = crud.create_document(
                db=db,
                title=book_title,
                source_uri=book_path
            )

            # Chunk the document
            chunks = await self.embedding_service.chunk_document(text_content)
            
            # Generate embeddings for all chunks
            embeddings = await self.embedding_service.generate_embeddings(chunks)
            
            if not embeddings or len(embeddings) != len(chunks):
                return {"status": "error", "message": "Failed to generate embeddings", "documents_indexed": 0}

            # Store chunks and embeddings
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                embedding_id = str(uuid.uuid4())
                
                # Store chunk metadata in Postgres
                crud.create_document_chunk(
                    db=db,
                    document_id=document.id,
                    chunk_index=i,
                    content=chunk,
                    embedding_id=embedding_id
                )
                
                # Store embedding in Qdrant
                await self.qdrant_service.add_vector(
                    collection_name=self.collection_name,
                    vector=embedding,
                    payload={"embedding_id": embedding_id, "document_id": str(document.id), "chunk_index": i}
                )

            # Update document with total chunks
            document.total_chunks = len(chunks)
            db.commit()

            return {
                "status": "success", 
                "message": f"Successfully ingested {book_title}",
                "documents_indexed": 1,
                "chunks_created": len(chunks)
            }

        except Exception as e:
            db.rollback()
            return {"status": "error", "message": f"Ingestion failed: {str(e)}", "documents_indexed": 0}
        finally:
            try:
                next(db_generator)
            except StopIteration:
                pass

    async def ingest_markdown_docs(self, docs_path: str) -> dict:
        """
        Ingest Docusaurus markdown documentation from a directory
        Returns: dict with status and documents_indexed count
        """
        if not os.path.exists(docs_path):
            return {"status": "error", "message": "Directory not found", "documents_indexed": 0}

        markdown_files = []
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                if file.endswith(('.md', '.markdown', '.mdx')):
                    markdown_files.append(os.path.join(root, file))

        if not markdown_files:
            return {"status": "error", "message": "No markdown files found", "documents_indexed": 0}

        db_generator = get_db()
        db = next(db_generator)
        documents_indexed = 0

        try:
            for file_path in markdown_files:
                try:
                    # Extract text content
                    text_content = self.extract_text_from_markdown(file_path)
                    if not text_content.strip():
                        continue

                    # Create document record
                    relative_path = os.path.relpath(file_path, docs_path)
                    document_title = Path(file_path).stem
                    
                    document = crud.create_document(
                        db=db,
                        title=document_title,
                        source_uri=relative_path
                    )

                    # Chunk the document
                    chunks = await self.embedding_service.chunk_document(text_content)
                    
                    # Generate embeddings
                    embeddings = await self.embedding_service.generate_embeddings(chunks)
                    
                    if not embeddings or len(embeddings) != len(chunks):
                        print(f"Failed to generate embeddings for {file_path}")
                        continue

                    # Store chunks and embeddings
                    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                        embedding_id = str(uuid.uuid4())
                        
                        # Store chunk metadata in Postgres
                        crud.create_document_chunk(
                            db=db,
                            document_id=document.id,
                            chunk_index=i,
                            content=chunk,
                            embedding_id=embedding_id
                        )
                        
                        # Store embedding in Qdrant
                        await self.qdrant_service.add_vector(
                            collection_name=self.collection_name,
                            vector=embedding,
                            payload={"embedding_id": embedding_id, "document_id": str(document.id), "chunk_index": i}
                        )

                    # Update document with total chunks
                    document.total_chunks = len(chunks)
                    documents_indexed += 1

                except Exception as e:
                    print(f"Error processing {file_path}: {e}")
                    continue

            db.commit()
            return {
                "status": "success",
                "message": f"Successfully ingested {documents_indexed} markdown documents",
                "documents_indexed": documents_indexed
            }

        except Exception as e:
            db.rollback()
            return {"status": "error", "message": f"Batch ingestion failed: {str(e)}", "documents_indexed": 0}
        finally:
            try:
                next(db_generator)
            except StopIteration:
                pass