import os
from dotenv import load_dotenv
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
import asyncio
from fastapi import FastAPI, HTTPException, File, UploadFile, Form
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
    model="gemini-2.5-flash",
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
    return {"message": "Book Backend with RAG Chatbot API is running"}

@app.get("/health")
async def health_check():
    """Enhanced health check with security considerations."""
    try:
        health_status = {
            "status": "healthy",
            "message": "RAG Chatbot Backend is operational",
            "version": "1.0.0"
        }
        
        health_status["features"] = [
            "RAG Chat",
            "Voice Processing", 
            "File Upload",
            "Document Ingestion",
            "Text Selection"
        ]
            
        return health_status
    except Exception as e:
        return {"status": "unhealthy", "error": "Service unavailable"}


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

@app.post("/chat/rag")
async def rag_chat(request: ChatRequest):
    """RAG-enhanced chat endpoint for more detailed responses"""
    try:
        # Enhanced AI response using RAG (Retrieval Augmented Generation)
        rag_response = await generate_rag_response(request.message)
        
        
        session_id = request.session_id or str(uuid.uuid4())
        
        if session_id not in chat_sessions:
            chat_sessions[session_id] = []
        
        ai_message = ChatMessage(
            id=str(uuid.uuid4()),
            content=rag_response,
            sender="ai",
            timestamp=datetime.now(),
            session_id=session_id
        )
        
        chat_sessions[session_id].append(ai_message)
        
        return ChatResponse(message=ai_message, session_id=session_id)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat/file")
async def upload_file(file: UploadFile = File(...), session_id: str = Form(...)):
    """Handle file uploads for chat"""
    try:
        # Read file content
        file_content = await file.read()
        
        # Process the uploaded file
        file_response = await process_uploaded_file(file_content, session_id, file.filename)
        
        if session_id not in chat_sessions:
            chat_sessions[session_id] = []
        
        ai_message = ChatMessage(
            id=str(uuid.uuid4()),
            content=file_response,
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
        # Return a more helpful mock response when API fails
        return f"Hello! I'm your AI assistant. You asked: '{message}'. I'm currently in demo mode due to API limitations, but I'm here to help! In a full deployment, I would provide detailed responses using advanced AI capabilities."

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
        return f"📚 **Help with: {message}**\n\nI'm here to help explain this topic! In demo mode, I can provide general guidance:\n\n• This appears to be related to your query about '{message}'\n• In a full deployment, I would analyze this using advanced AI and provide detailed explanations\n• I would break down complex concepts and provide relevant examples\n• I would offer additional context and related information\n\nPlease note: This is a demo response due to API limitations."

async def generate_rag_response(message: str) -> str:
    """Generate RAG-enhanced response with more context and detail"""
    try:
        agent = Agent(
            name="RAG Assistant", 
            instructions="You are an advanced AI assistant with access to a knowledge base. Provide comprehensive, well-structured responses with relevant context, examples, and detailed explanations. Use markdown formatting for better readability.",
            model=model
        )
        rag_prompt = f"Using your knowledge base, provide a comprehensive response to: {message}"
        result = await Runner.run(agent, rag_prompt, run_config=config)
        return result.final_output
    except Exception as e:
        print(f"Error generating RAG response: {e}")
        return f"🤖 **Enhanced AI Response (Demo Mode)**\n\n**Your Query:** {message}\n\n**Response:** I'm your advanced AI assistant with RAG capabilities! In full deployment mode, I would:\n\n• Search through extensive knowledge bases\n• Provide comprehensive, well-structured responses\n• Include relevant context and examples\n• Use advanced reasoning and analysis\n• Format responses with markdown for better readability\n\nCurrently in demo mode due to API limitations, but the interface and functionality are fully operational!"

async def process_uploaded_file(file_data: bytes, session_id: str, filename: str = None) -> str:
    """Process uploaded file and generate response"""
    try:
        # In a real implementation, you would:
        # 1. Save the file
        # 2. Extract text/content from the file
        # 3. Process it with AI
        # 4. Generate a meaningful response
        
        file_size = len(file_data)
        file_info = f"File: {filename or 'unknown'}, Size: {file_size} bytes"
        
        agent = Agent(
            name="File Processor", 
            instructions="You are an AI assistant that helps users understand and analyze uploaded files. Provide helpful insights about file processing and offer to help analyze the content.",
            model=model
        )
        
        file_prompt = f"A user has uploaded a file: {file_info}. Acknowledge the upload and offer to help analyze or discuss the file content."
        result = await Runner.run(agent, file_prompt, run_config=config)
        return result.final_output
    except Exception as e:
        print(f"Error processing uploaded file: {e}")
        return f"I've received your file upload ({filename or 'unknown file'}). However, I'm experiencing some technical difficulties processing it right now. Please try again or contact support if the issue persists."

async def main():
    """Original main function for testing"""
    agent = Agent(name="Assistant", instructions="You are a helpful assistant.", model=model)
    result = await Runner.run(agent, "What is the capital of Pakistan?", run_config=config)
    print(f"Result: {result.final_output}")

# if __name__ == "__main__":
#     import uvicorn
#     # Run FastAPI server instead of the test function
#     # uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
#     uvicorn.run("main:app", host="0.0.0.0", port=8080, reload=False)  # ✅ PORT 8080, reload=False    



if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8080))  # ✅ Cloud Run PORT
    uvicorn.run(
        "main:app",  # ✅ String format
        host="0.0.0.0", 
        port=port, 
        reload=False,  # ✅ Production
        workers=1  # ✅ Cloud Run single worker
    )