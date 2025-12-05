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
app = FastAPI(title="Book Backend with Chatbot API", version="1.0.0")

# CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
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

class VoiceRequest(BaseModel):
    session_id: str
    audio_data: str  # base64 encoded audio

# API Routes
@app.get("/")
async def root():
    return {"message": "Book Backend with Chatbot API is running"}

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

@app.post("/chat/voice")
async def process_voice(request: VoiceRequest):
    try:
        # Mock voice processing - integrate with speech recognition
        transcribed_text = "This is a mock transcription of the voice input"
        
        # Process as regular chat message
        chat_request = ChatRequest(
            message=transcribed_text,
            session_id=request.session_id
        )
        
        return await send_message(chat_request)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat/ask-ai")
async def ask_ai_help(request: ChatRequest):
    try:
        # Enhanced AI response for help requests
        help_response = await generate_ai_help_response(request.message)
        
        session_id = request.session_id or str(uuid.uuid4())
        
        if session_id not in chat_sessions:
            chat_sessions[session_id] = []
        
        ai_message = ChatMessage(
            id=str(uuid.uuid4()),
            content=help_response,
            sender="ai",
            timestamp=datetime.now(),
            session_id=session_id
        )
        
        chat_sessions[session_id].append(ai_message)
        
        return ChatResponse(message=ai_message, session_id=session_id)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

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

async def generate_ai_help_response(message: str) -> str:
    """Enhanced AI response for help requests"""
    try:
        agent = Agent(
            name="AI Help Assistant", 
            instructions="You are an AI assistant specifically helping users understand topics. When someone asks for help, provide detailed explanations, break down complex concepts, and offer additional context. Be educational and thorough.",
            model=model
        )
        help_prompt = f"Please help explain and provide detailed information about: {message}"
        result = await Runner.run(agent, help_prompt, run_config=config)
        return result.final_output
    except Exception as e:
        print(f"Error generating AI help response: {e}")
        return f"I can help explain that! Regarding '{message}', let me provide some information. However, I'm experiencing some technical difficulties right now. Please try asking again."

async def main():
    """Original main function for testing"""
    agent = Agent(name="Assistant", instructions="You are a helpful assistant.", model=model)
    result = await Runner.run(agent, "What is the capital of Pakistan?", run_config=config)
    print(f"Result: {result.final_output}")

if __name__ == "__main__":
    import uvicorn
    # Run FastAPI server instead of the test function
    uvicorn.run(app, host="0.0.0.0", port=8000)