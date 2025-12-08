import os
from dotenv import load_dotenv
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
import asyncio
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import uuid
from datetime import datetime

load_dotenv()

# FastAPI app setup
app = FastAPI(
    title="Neurobotics AI Book Backend with Chatbot API", 
    version="1.0.0"
)

# CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://neurobotics.vercel.app",  # Production URL
        "https://*.vercel.app"  # Any Vercel preview deployments
    ], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# AI Model setup
gemini_api_key = os.getenv("GEMINI_API_KEY")
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY environment variable is not set.")

external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    timeout=30.0,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    openai_client=external_client,
    model="gemini-2.0-flash",
)

config = RunConfig(
    model=model,
    model_provider=external_client,
    tracing_disabled=True,
)

# In-memory storage (replace with database in production)
chat_sessions = {}

# Pydantic models
class ChatMessage(BaseModel):
    id: str
    content: str
    sender: str  # 'user' or 'ai'
    timestamp: datetime
    session_id: str

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    message: ChatMessage
    session_id: str

# API Routes
@app.get("/")
async def root():
    return {"message": "Book Backend with RAG Chatbot API is running"}

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "message": "RAG Chatbot Backend is operational",
        "version": "1.0.0"
    }

@app.post("/chat", response_model=ChatResponse)
async def send_message(request: ChatRequest):
    try:
        # Create or get session
        session_id = request.session_id or str(uuid.uuid4())
        
        if session_id not in chat_sessions:
            chat_sessions[session_id] = []
        
        # Create user message
        user_message = ChatMessage(
            id=str(uuid.uuid4()),
            content=request.message,
            sender="user",
            timestamp=datetime.now(),
            session_id=session_id
        )
        
        chat_sessions[session_id].append(user_message)
        
        # Generate AI response using the existing agent
        ai_response_content = await generate_ai_response(request.message)
        
        ai_message = ChatMessage(
            id=str(uuid.uuid4()),
            content=ai_response_content,
            sender="ai",
            timestamp=datetime.now(),
            session_id=session_id
        )
        
        chat_sessions[session_id].append(ai_message)
        
        return ChatResponse(message=ai_message, session_id=session_id)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/chat/history/{session_id}")
async def get_chat_history(session_id: str):
    if session_id not in chat_sessions:
        return {"messages": []}
    
    return {"messages": chat_sessions[session_id]}

async def generate_ai_response(message: str) -> str:
    """Generate AI response using the existing agent setup"""
    try:
        agent = Agent(
            name="ChatBot Assistant", 
            instructions="You are a helpful AI assistant in a chatbot interface. Provide clear, concise, and helpful responses. Be friendly and conversational.",
            model=model
        )
        result = await Runner.run(agent, message, run_config=config)
        return result.final_output
    except Exception as e:
        print(f"Error generating AI response: {e}")
        return "I apologize, but I'm having trouble processing your request right now. Please try again."

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main_minimal:app", host="0.0.0.0", port=8000, reload=True)