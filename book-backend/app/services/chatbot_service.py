"""RAG Chatbot service using OpenAI Agents SDK"""

from typing import Optional, List, Dict, Any, Tuple
from datetime import datetime
from sqlalchemy.orm import Session
import logging
import json
import asyncio
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig

from app.models.database import ChatSession, ChatMessage, Book, Chapter, User
from app.services.content_service import ContentService
from app.services.qdrant_service import qdrant_service
from app.core.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class ChatbotService:
    """Service for RAG-powered chatbot using OpenAI Agents SDK"""
    
    def __init__(self, db: Session):
        self.db = db
        self.content_service = ContentService(db)
        self.agent = None
        self.runner = None
        self._initialize_openai_agent()
    
    def _initialize_openai_agent(self):
        """Initialize OpenAI Agent using proper Agents SDK"""
        try:
            if not settings.openai_api_key:
                logger.warning("OpenAI API key not configured")
                return
            
            # Initialize OpenAI model
            model = OpenAIChatCompletionsModel(
                'gemini-2.0-flash',
                api_key=settings.openai_api_key
            )
            
            # Create the agent with RAG capabilities
            self.agent = Agent(
                model=model,
                system_prompt="""You are an intelligent book content assistant with RAG capabilities.

Your role is to help users understand and learn from book content by:
1. Answering questions based on provided book context
2. Explaining concepts in a clear and engaging way  
3. Adapting your responses to the user's experience level and persona
4. Providing practical examples and applications
5. Citing sources when referencing specific book content
6. Admitting when information is not available in the provided context

When you need to search for book content, use the search_book_content function.
Always be helpful, accurate, and educational in your responses.""",
                functions=[self._search_book_content_function]
            )
            
            # Initialize runner
            self.runner = Runner(self.agent)
            
            logger.info("OpenAI Agent initialized successfully using Agents SDK")
                
        except Exception as e:
            logger.error(f"Error initializing OpenAI Agent: {e}")
            self.agent = None
            self.runner = None
    
    def _search_book_content_function(
        self, 
        query: str, 
        book_ids: Optional[List[int]] = None, 
        limit: int = 5
    ) -> str:
        """Function for agent to search book content"""
        try:
            relevant_chunks = self.search_relevant_content(
                query=query,
                book_ids=book_ids,
                limit=limit,
                similarity_threshold=0.7
            )
            
            if not relevant_chunks:
                return "No relevant content found for this query."
            
            # Format results for the agent
            formatted_results = []
            for chunk in relevant_chunks:
                source_info = f"Book: {chunk['source']['book_title']}"
                if chunk['source']['type'] == 'chapter':
                    source_info += f" | Chapter {chunk['source']['chapter_number']}: {chunk['source']['chapter_title']}"
                
                formatted_results.append(f"""
Source: {source_info}
Relevance Score: {chunk['similarity_score']:.3f}
Content: {chunk['content']}
---""")
            
            return "\n".join(formatted_results)
            
        except Exception as e:
            logger.error(f"Error in search function: {e}")
            return f"Error searching content: {str(e)}"
    
    def create_chat_session(
        self, 
        user_id: Optional[int] = None,
        title: Optional[str] = None,
        session_type: str = "general",
        language: str = "en",
        context_book_id: Optional[int] = None
    ) -> ChatSession:
        """Create a new chat session"""
        try:
            session = ChatSession(
                user_id=user_id,
                title=title or f"Chat Session - {datetime.utcnow().strftime('%Y-%m-%d %H:%M')}",
                session_type=session_type,
                language=language,
                context_book_id=context_book_id,
                created_at=datetime.utcnow()
            )
            
            self.db.add(session)
            self.db.commit()
            self.db.refresh(session)
            
            logger.info(f"Created chat session {session.id} for user {user_id}")
            return session
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error creating chat session: {e}")
            raise
    
    def get_chat_session(self, session_id: int) -> Optional[ChatSession]:
        """Get chat session by ID"""
        return self.db.query(ChatSession).filter(ChatSession.id == session_id).first()
    
    def get_user_chat_sessions(
        self, 
        user_id: int, 
        limit: int = 20
    ) -> List[ChatSession]:
        """Get user's chat sessions"""
        return self.db.query(ChatSession).filter(
            ChatSession.user_id == user_id
        ).order_by(ChatSession.updated_at.desc()).limit(limit).all()
    
    def add_message_to_session(
        self,
        session_id: int,
        content: str,
        sender: str,
        message_type: str = "text",
        language: str = "en",
        context_used: Optional[List[str]] = None,
        confidence_score: Optional[float] = None
    ) -> ChatMessage:
        """Add message to chat session"""
        try:
            message = ChatMessage(
                session_id=session_id,
                content=content,
                sender=sender,
                message_type=message_type,
                language=language,
                context_used=context_used,
                confidence_score=confidence_score,
                created_at=datetime.utcnow()
            )
            
            self.db.add(message)
            
            # Update session timestamp
            session = self.get_chat_session(session_id)
            if session:
                session.updated_at = datetime.utcnow()
            
            self.db.commit()
            self.db.refresh(message)
            
            return message
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error adding message to session {session_id}: {e}")
            raise
    
    def get_session_messages(
        self, 
        session_id: int, 
        limit: int = 50
    ) -> List[ChatMessage]:
        """Get messages from chat session"""
        return self.db.query(ChatMessage).filter(
            ChatMessage.session_id == session_id
        ).order_by(ChatMessage.created_at.asc()).limit(limit).all()
    
    def generate_embeddings(self, text: str) -> List[float]:
        """Generate embeddings for text using OpenAI"""
        try:
            # Use AsyncOpenAI for embeddings
            async def _generate_embedding():
                client = AsyncOpenAI(api_key=settings.openai_api_key)
                response = await client.embeddings.create(
                    model="text-embedding-ada-002",
                    input=text
                )
                return response.data[0].embedding
            
            # Run async function
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                embedding = loop.run_until_complete(_generate_embedding())
                return embedding
            finally:
                loop.close()
            
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise
    
    def search_relevant_content(
        self,
        query: str,
        book_ids: Optional[List[int]] = None,
        limit: int = 5,
        similarity_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """Search for relevant content using vector similarity"""
        try:
            # Generate query embedding
            query_embedding = self.generate_embeddings(query)
            
            # Build filter conditions
            filter_conditions = {}
            if book_ids:
                filter_conditions["book_id"] = book_ids[0] if len(book_ids) == 1 else book_ids
            
            # Search in Qdrant
            search_results = qdrant_service.search_similar(
                query_vector=query_embedding,
                limit=limit,
                score_threshold=similarity_threshold,
                filter_conditions=filter_conditions
            )
            
            # Enrich results with database content
            enriched_results = []
            for result in search_results:
                payload = result.get("payload", {})
                
                # Get chapter/book info from database
                if "chapter_id" in payload:
                    chapter = self.content_service.get_chapter(payload["chapter_id"])
                    if chapter:
                        book = self.content_service.get_book(chapter.book_id)
                        enriched_results.append({
                            "content": payload.get("text", ""),
                            "similarity_score": result.get("score", 0),
                            "source": {
                                "type": "chapter",
                                "book_id": chapter.book_id,
                                "book_title": book.title if book else "Unknown",
                                "chapter_id": chapter.id,
                                "chapter_title": chapter.title,
                                "chapter_number": chapter.chapter_number
                            }
                        })
                elif "book_id" in payload:
                    book = self.content_service.get_book(payload["book_id"])
                    if book:
                        enriched_results.append({
                            "content": payload.get("text", ""),
                            "similarity_score": result.get("score", 0),
                            "source": {
                                "type": "book",
                                "book_id": book.id,
                                "book_title": book.title
                            }
                        })
            
            return enriched_results
            
        except Exception as e:
            logger.error(f"Error searching relevant content: {e}")
            return []
    
    async def _run_agent_async(
        self, 
        user_message: str,
        user_preferences: Optional[Dict[str, Any]] = None,
        language: str = "en",
        context_book_ids: Optional[List[int]] = None
    ) -> Tuple[str, float, List[str]]:
        """Run agent asynchronously with user message"""
        try:
            if not self.agent or not self.runner:
                return "Agent not available", 0.0, []
            
            # Build context message for personalization
            context_message = ""
            if user_preferences:
                persona = user_preferences.get("persona", "general")
                experience_level = user_preferences.get("experience_level", "intermediate")
                
                persona_instructions = {
                    "developer": "Focus on implementation details, code examples, and technical best practices.",
                    "researcher": "Provide detailed explanations with theoretical background and references.",
                    "student": "Use simple language with step-by-step explanations and learning tips.",
                    "beginner": "Use basic terminology and provide foundational concepts first.",
                    "intermediate": "Balance detail with clarity, assume some background knowledge.",
                    "advanced": "Provide in-depth analysis and advanced concepts."
                }
                
                context_message = f"""
User Profile: {persona} with {experience_level} experience level.
Please adapt your response style: {persona_instructions.get(persona, 'Provide clear and helpful information.')}
"""
                
                if language == "ur":
                    context_message += "Please respond in Urdu using proper Urdu script and grammar."
                elif language != "en":
                    context_message += f"Please respond in {language} language."
            
            # Add context about specific books if provided
            if context_book_ids:
                context_message += f"\nFocus on content from book IDs: {context_book_ids}"
            
            # Combine context and user message
            full_message = f"{context_message}\n\nUser Question: {user_message}" if context_message else user_message
            
            # Run the agent
            run_config = RunConfig(
                max_iterations=5,  # Allow multiple function calls if needed
                timeout=30.0  # 30 second timeout
            )
            
            result = await self.runner.run(full_message, config=run_config)
            
            # Extract response and sources
            response_text = result.data if hasattr(result, 'data') else str(result)
            
            # Try to extract sources from the response or function calls
            sources_used = []
            if hasattr(result, 'messages'):
                for message in result.messages:
                    if hasattr(message, 'content') and 'Book:' in str(message.content):
                        # Extract book titles from function call results
                        content_str = str(message.content)
                        lines = content_str.split('\n')
                        for line in lines:
                            if line.startswith("Source: Book:"):
                                source = line.replace("Source: ", "")
                                if source not in sources_used:
                                    sources_used.append(source)
            
            confidence = 0.9  # High confidence for successful agent run
            return response_text, confidence, sources_used
            
        except Exception as e:
            logger.error(f"Error running agent: {e}")
            return "I'm experiencing technical difficulties. Please try again later.", 0.0, []
    
    def _search_book_content_function(
        self, 
        query: str, 
        book_ids: Optional[List[int]] = None, 
        limit: int = 5
    ) -> str:
        """Function tool for assistant to search book content"""
        try:
            relevant_chunks = self.search_relevant_content(
                query=query,
                book_ids=book_ids,
                limit=limit,
                similarity_threshold=0.7
            )
            
            if not relevant_chunks:
                return "No relevant content found for this query."
            
            # Format results for the assistant
            formatted_results = []
            for chunk in relevant_chunks:
                source_info = f"Book: {chunk['source']['book_title']}"
                if chunk['source']['type'] == 'chapter':
                    source_info += f" | Chapter {chunk['source']['chapter_number']}: {chunk['source']['chapter_title']}"
                
                formatted_results.append(f"""
Source: {source_info}
Relevance Score: {chunk['similarity_score']:.3f}
Content: {chunk['content']}
---""")
            
            return "\n".join(formatted_results)
            
        except Exception as e:
            logger.error(f"Error in search function: {e}")
            return f"Error searching content: {str(e)}"
    
    def _run_agent_sync(
        self, 
        user_message: str,
        user_preferences: Optional[Dict[str, Any]] = None,
        language: str = "en",
        context_book_ids: Optional[List[int]] = None
    ) -> Tuple[str, float, List[str]]:
        """Synchronous wrapper for running the agent"""
        try:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                # Run the async agent function
                result = loop.run_until_complete(
                    self._run_agent_async(
                        user_message=user_message,
                        user_preferences=user_preferences,
                        language=language,
                        context_book_ids=context_book_ids
                    )
                )
                return result
            finally:
                loop.close()
                
        except Exception as e:
            logger.error(f"Error in sync agent wrapper: {e}")
            return "I'm having trouble processing your request. Please try again.", 0.0, []
    
    def chat_with_rag(
        self,
        session_id: int,
        user_message: str,
        user_id: Optional[int] = None,
        use_rag: bool = True,
        context_book_ids: Optional[List[int]] = None
    ) -> Dict[str, Any]:
        """Main chat function using OpenAI Agents SDK with RAG capabilities"""
        try:
            start_time = datetime.utcnow()
            
            # Get session
            session = self.get_chat_session(session_id)
            if not session:
                raise ValueError(f"Chat session {session_id} not found")
            
            # Add user message to session
            user_msg = self.add_message_to_session(
                session_id=session_id,
                content=user_message,
                sender="user",
                language=session.language
            )
            
            # Get user preferences if available
            user_preferences = {}
            if user_id:
                user = self.db.query(User).filter(User.id == user_id).first()
                if user:
                    user_preferences = {
                        "persona": user.persona,
                        "experience_level": user.experience_level,
                        "preferences": user.preferences or {}
                    }
            
            # Determine context books
            search_book_ids = context_book_ids or []
            if session.context_book_id:
                search_book_ids.append(session.context_book_id)
            
            # Generate response using OpenAI Agent
            if use_rag and self.agent and self.runner:
                # Run agent with RAG capabilities
                response_text, confidence, sources_used = self._run_agent_sync(
                    user_message=user_message,
                    user_preferences=user_preferences,
                    language=session.language,
                    context_book_ids=search_book_ids if search_book_ids else None
                )
                context_used = sources_used
            else:
                # Fallback response without RAG
                response_text = "I'm here to help you with questions about the book content. Please make sure the AI service is properly configured."
                confidence = 0.5
                context_used = []
            
            # Add assistant message to session
            assistant_msg = self.add_message_to_session(
                session_id=session_id,
                content=response_text,
                sender="assistant",
                language=session.language,
                context_used=context_used,
                confidence_score=confidence
            )
            
            processing_time = (datetime.utcnow() - start_time).total_seconds()
            
            return {
                "session_id": session_id,
                "user_message_id": user_msg.id,
                "assistant_message_id": assistant_msg.id,
                "response": response_text,
                "context_used": context_used,
                "confidence_score": confidence,
                "processing_time": processing_time
            }
            
        except Exception as e:
            logger.error(f"Error in chat with RAG: {e}")
            raise
    
    def get_chat_suggestions(
        self,
        session_id: int,
        context_book_id: Optional[int] = None
    ) -> List[str]:
        """Generate chat suggestions based on session context"""
        try:
            suggestions = [
                "What are the main concepts covered in this book?",
                "Can you explain this topic in simpler terms?",
                "What are some practical examples of this concept?",
                "How does this relate to real-world applications?",
                "What should I learn next after this chapter?"
            ]
            
            # Add book-specific suggestions if context is available
            if context_book_id:
                book = self.content_service.get_book(context_book_id)
                if book:
                    suggestions.extend([
                        f"Tell me about the main themes in '{book.title}'",
                        f"What are the key takeaways from '{book.title}'?",
                        f"How is '{book.title}' structured?"
                    ])
            
            return suggestions[:5]  # Return top 5 suggestions
            
        except Exception as e:
            logger.error(f"Error generating chat suggestions: {e}")
            return []
    
    def delete_chat_session(self, session_id: int, user_id: Optional[int] = None) -> bool:
        """Delete chat session and all its messages"""
        try:
            session = self.get_chat_session(session_id)
            if not session:
                return False
            
            # Check ownership if user_id provided
            if user_id and session.user_id != user_id:
                return False
            
            # Delete all messages first
            self.db.query(ChatMessage).filter(
                ChatMessage.session_id == session_id
            ).delete()
            
            # Delete session
            self.db.delete(session)
            self.db.commit()
            
            logger.info(f"Deleted chat session {session_id}")
            return True
            
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error deleting chat session {session_id}: {e}")
            return False