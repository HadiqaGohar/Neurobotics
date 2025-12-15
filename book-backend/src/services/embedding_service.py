from sentence_transformers import SentenceTransformer
from typing import List
import re
import asyncio



class EmbeddingService:
    def __init__(self):
        self.model = SentenceTransformer('all-MiniLM-L6-v2')

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts with error handling and validation"""
        if not texts:
            return []
        
        # Validate input
        if len(texts) > 100:  # Limit batch size
            raise ValueError("Too many texts in batch (max 100)")
        
        # Filter and validate texts
        valid_texts = []
        for text in texts:
            if isinstance(text, str) and text.strip():
                # Limit text length
                if len(text) > 10000:
                    text = text[:10000]
                valid_texts.append(text.strip())
        
        if not valid_texts:
            return []
        
        try:
            # Run the encoding in a thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            embeddings = await loop.run_in_executor(
                None, 
                lambda: self.model.encode(valid_texts, convert_to_numpy=True).tolist()
            )
            
            # Validate output
            if not embeddings or not all(isinstance(emb, list) for emb in embeddings):
                raise ValueError("Invalid embeddings generated")
            
            return embeddings
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            # Return empty embeddings instead of raising
            return [[] for _ in valid_texts]

    async def chunk_document(self, text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
        """
        Chunk a document into smaller pieces with overlap
        Uses sentence-aware chunking when possible
        """
        if not text or not text.strip():
            return []
        
        text = text.strip()
        
        # If text is shorter than chunk_size, return as single chunk
        if len(text) <= chunk_size:
            return [text]
        
        # Try sentence-based chunking first
        chunks = self._chunk_text_by_sentences(text, chunk_size, overlap)
        
        # If sentence-based chunking doesn't work well, fall back to word-based
        if not chunks or any(len(chunk) > chunk_size * 1.5 for chunk in chunks):
            chunks = self._chunk_text_by_words(text, chunk_size, overlap)
        
        # Filter out empty chunks
        chunks = [chunk.strip() for chunk in chunks if chunk.strip()]
        
        return chunks

    def _chunk_text_by_sentences(self, text: str, chunk_size: int, overlap: int) -> List[str]:
        """Chunk text by sentences, respecting chunk size limits"""
        # Split into sentences using regex
        sentences = re.split(r'(?<=[.!?])\s+', text)
        
        chunks = []
        current_chunk = ""
        
        for sentence in sentences:
            # If adding this sentence would exceed chunk_size, start a new chunk
            if current_chunk and len(current_chunk) + len(sentence) + 1 > chunk_size:
                chunks.append(current_chunk.strip())
                
                # Start new chunk with overlap from previous chunk
                if overlap > 0 and chunks:
                    overlap_text = self._get_overlap_text(current_chunk, overlap)
                    current_chunk = overlap_text + " " + sentence if overlap_text else sentence
                else:
                    current_chunk = sentence
            else:
                current_chunk = current_chunk + " " + sentence if current_chunk else sentence
        
        # Add the last chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        return chunks

    def _chunk_text_by_words(self, text: str, chunk_size: int, overlap: int) -> List[str]:
        """Chunk text by words when sentence-based chunking isn'"t suitable"""
        words = text.split()
        chunks = []
        
        for i in range(0, len(words), chunk_size - overlap):
            chunk_words = words[i:i + chunk_size]
            chunk = " ".join(chunk_words)
            chunks.append(chunk)
        
        return chunks

    def _get_overlap_text(self, text: str, overlap_chars: int) -> str:
        """Get the last overlap_chars characters from text, trying to break at word boundaries"""
        if len(text) <= overlap_chars:
            return text
        
        # Try to break at word boundary
        overlap_text = text[-overlap_chars:]
        space_index = overlap_text.find(' ')
        
        if space_index > 0:
            return overlap_text[space_index + 1:]
        
        return overlap_text
