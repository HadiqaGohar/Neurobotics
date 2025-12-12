import os
import re
from typing import List, Dict
import openai
from qdrant_client import QdrantClient, models
from PyPDF2 import PdfReader


class TextLoader:
    def __init__(self):
        if PdfReader is None:
            print("Warning: PyPDF2 is not installed. PDF files cannot be processed.")

    def _load_txt_md(self, file_path: str) -> str:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()

    def _load_pdf(self, file_path: str) -> str:
        if PdfReader is None:
            raise ImportError("PyPDF2 is not installed. Please install it to process PDF files.")
        
        text = ""
        with open(file_path, 'rb') as f:
            reader = PdfReader(f)
            for page in reader.pages:
                text += page.extract_text()
        return text

    def load_file(self, file_path: str) -> Dict[str, str]:
        _ , ext = os.path.splitext(file_path)
        content = ""

        if ext == '.txt' or ext == '.md':
            content = self._load_txt_md(file_path)
        elif ext == '.pdf':
            content = self._load_pdf(file_path)
        else:
            raise ValueError(f"Unsupported file type: {ext}")

        # Extract module and chapter from file path
        # Assuming path format like /modules/<module_name>/<chapter_name>.<ext>
        parts = file_path.split(os.sep)
        module_name = "unknown_module"
        chapter_name = os.path.basename(file_path).split('.')[0]

        if "modules" in parts:
            module_index = parts.index("modules")
            if module_index + 1 < len(parts):
                module_name = parts[module_index + 1]

        return {
            "module": module_name,
            "chapter": chapter_name,
            "text": content
        }

class ChunkText:
    def __init__(self, chunk_size: int = 1500, overlap: int = 200):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk(self, text: str) -> List[str]:
        # Simple character-based chunking
        # TODO: Improve chunking to respect paragraph boundaries, etc.
        chunks = []
        start = 0
        while start < len(text):
            end = start + self.chunk_size
            chunk = text[start:end]
            chunks.append(chunk)
            start += (self.chunk_size - self.overlap)
            if start >= len(text):
                break
        return chunks

# Placeholder for other tools
class MakeEmbeddings:
    def __init__(self, model: str = "gemini-embedding-001", dimension: int = 1536):
        self.client = openai.OpenAI(
            api_key=os.getenv("GEMINI_API_KEY"), # Use GEMINI_API_KEY from environment
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        self.model = model
        self.dimension = dimension
        # Note: Gemini embedding models typically have dimension 768 or 1024, not 1536.
        # However, following the requirement from CHATBOT_README.md (dimension: 1536).
        # We will proceed with the requested dimension.

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        print(f"Generating embeddings for {len(texts)} texts using Gemini model '{self.model}' via OpenAI client...")
        # OpenAI client's embeddings.create expects a list of strings
        response = self.client.embeddings.create(
            input=texts,
            model=self.model
        )
        # Extract embeddings from the response
        embeddings = [data.embedding for data in response.data]
        return embeddings

class QdrantUpsert:
    def __init__(self, collection_name: str = "book_chunks", vector_size: int = 1536, distance: str = "Cosine"):
        self.collection_name = collection_name
        self.vector_size = vector_size
        self.distance = models.Distance.COSINE if distance.lower() == "cosine" else models.Distance.DOT
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        
        # Ensure collection exists or create it
        try:
            self.client.get_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' already exists.")
        except Exception: # Qdrant client raises an exception if collection not found
            print(f"Collection '{self.collection_name}' not found. Creating new collection.")
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=self.vector_size, distance=self.distance),
            )
            print(f"Collection '{self.collection_name}' created with vector size {self.vector_size} and distance {distance}.")

    def upsert(self, chunks: List[str], embeddings: List[List[float]], metadata: List[Dict]) -> List[str]:
        print(f"Upserting {len(chunks)} chunks to Qdrant collection {self.collection_name}...")
        
        points = []
        qdrant_ids = []
        for i, (chunk_text, embedding, meta) in enumerate(zip(chunks, embeddings, metadata)):
            point_id = str(uuid.uuid4()) # Generate a unique ID for each point
            qdrant_ids.append(point_id)
            payload = {
                "text": chunk_text,
                "module": meta.get("module"),
                "chapter": meta.get("chapter"),
                **meta # Include any other metadata passed
            }
            points.append(models.PointStruct(id=point_id, vector=embedding, payload=payload))

        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points,
        )
        print(f"Successfully upserted {len(chunks)} points to Qdrant.")
        return qdrant_ids

class NeonInsert:
    def __init__(self):
        # TODO: Initialize Neon Postgres client
        # Handle NEON_DB_URL from environment
        pass

    def insert(self, data: List[Dict]) -> None:
        # TODO: Implement Neon insertion logic
        print(f"Inserting {len(data)} records into Neon Postgres...")
        pass

class SearchQdrant:
    def __init__(self, collection_name: str = "book_chunks", top_k: int = 5):
        self.collection_name = collection_name
        self.top_k = top_k
        # TODO: Initialize Qdrant client and Embedding service

    def search(self, query: str) -> List[Dict]:
        # TODO: Implement query embedding and Qdrant search
        print(f"Searching Qdrant for query: {query} with top_k={self.top_k}...")
        # Dummy implementation
        return [{"qdrant_id": "dummy_id", "text": "dummy text", "score": 0.9}]

class RagAnswer:
    def __init__(self):
        # TODO: Initialize Embedding service, SearchQdrant, Neon client, and Gemini model
        pass

    def get_answer(self, user_query: str, lang_pref: str = "English") -> Dict:
        # TODO: Implement full RAG workflow
        print(f"Generating RAG answer for query: {user_query} in {lang_pref}...")
        # Dummy implementation
        return {
            "answer": "This is a dummy answer based on retrieved context.",
            "sources": [{"doc_id": "dummy_doc", "chunk_id": "dummy_chunk"}],
            "prompt_sent": "Dummy prompt"
        }
