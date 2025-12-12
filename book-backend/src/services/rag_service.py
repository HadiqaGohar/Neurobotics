from typing import List, Optional
from qdrant_client import QdrantClient
from src.services.embedding_service import EmbeddingService
from src.database.database import get_db
from src.database import crud
from src.security.security_utils import SecurityValidator
import os
import logging
# from openai_agents.agents.chat_agent import ChatAgent
from agents import Agent, Runner


logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_client: QdrantClient, embedding_service: EmbeddingService):
        self.qdrant_client = qdrant_client
        self.embedding_service = embedding_service
        self.collection_name = "book_content" # Ensure this matches the collection created in T012
        
        # Validate API key exists
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            logger.warning("GEMINI_API_KEY not found, RAG responses may fail")
            
        self.chat_agent = Agent(model_name="gemini-2.5-flash", api_key=api_key)

    async def retrieve_context(self, query: str, top_k: int = 5) -> List[str]:
        """Retrieve relevant context for a query with security validation."""
        try:
            # Security validation
            if not SecurityValidator.validate_query(query):
                logger.warning(f"Potentially malicious query detected: {query[:100]}...")
                return []
            
            # Sanitize query
            sanitized_query = SecurityValidator.sanitize_input(query)
            if not sanitized_query.strip():
                return []
            
            # Validate top_k parameter
            top_k = max(1, min(top_k, 20))  # Limit between 1 and 20
            
            # 1. Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embeddings([sanitized_query])
            if not query_embedding:
                logger.warning("Failed to generate embeddings for query")
                return []

            # 2. Search Qdrant for similar vectors
            try:
                search_result = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding[0], # Assuming generate_embeddings returns List[List[float]]
                    limit=top_k
                )
            except Exception as e:
                logger.error(f"Qdrant search failed: {e}")
                return []

            # 3. Extract embedding_id from Qdrant results
            embedding_ids = []
            for hit in search_result:
                if hit.payload and 'embedding_id' in hit.payload:
                    embedding_ids.append(str(hit.payload['embedding_id']))
            
            if not embedding_ids:
                logger.info("No relevant context found for query")
                return []

            # 4. Retrieve chunk content from Postgres using embedding_ids
            db_generator = get_db()
            db = next(db_generator)
            try:
                chunks_content = crud.get_document_chunk_content_by_embedding_ids(db, embedding_ids)
                return chunks_content[:top_k]  # Ensure we don't return more than requested
            except Exception as e:
                logger.error(f"Database retrieval failed: {e}")
                return []
            finally:
                try:
                    next(db_generator)
                except StopIteration:
                    pass
                    
        except Exception as e:
            logger.error(f"Context retrieval failed: {e}")
            return []

    async def generate_rag_response(self, query: str, context: List[str]) -> str:
        """Generate RAG response with security validation and error handling."""
        try:
            # Security validation
            sanitized_query = SecurityValidator.sanitize_input(query)
            if not sanitized_query.strip():
                return "I'm sorry, but I cannot process that query."
            
            # Sanitize context
            sanitized_context = []
            for ctx in context:
                sanitized_ctx = SecurityValidator.sanitize_input(ctx)
                if sanitized_ctx.strip():
                    sanitized_context.append(sanitized_ctx)
            
            # Limit context size to prevent token overflow
            max_context_length = 8000  # Adjust based on model limits
            context_str = "\n".join(sanitized_context)
            if len(context_str) > max_context_length:
                context_str = context_str[:max_context_length] + "..."
            
            # Build secure prompt
            prompt = f"""You are a helpful AI assistant. Use the following pieces of context to answer the question at the end.
If you don't know the answer, just say that you don't know, don't try to make up an answer.
Do not include any code, scripts, or potentially harmful content in your response.

Context:
{context_str}

Question: {sanitized_query}

Helpful Answer:"""

            # Generate response using ChatAgent (Gemini integration)
            if not self.chat_agent:
                return "I'm sorry, but the AI service is currently unavailable."
            
            response = await self.chat_agent.send_message(prompt)
            
            if not response or not hasattr(response, 'content'):
                logger.error("Invalid response from chat agent")
                return "I'm sorry, but I couldn't generate a response at this time."
            
            # Sanitize the response
            sanitized_response = SecurityValidator.sanitize_input(response.content)
            
            # Ensure response is not empty
            if not sanitized_response.strip():
                return "I'm sorry, but I couldn't generate a meaningful response."
            
            return sanitized_response
            
        except Exception as e:
            logger.error(f"RAG response generation failed: {e}")
            return "I'm sorry, but I encountered an error while generating a response."


