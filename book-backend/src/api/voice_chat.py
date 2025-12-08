from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional
import uuid
import base64
import io
import tempfile
import os
import whisper
import speech_recognition as sr
from pydub import AudioSegment

from src.database.database import get_db
from src.database import crud
from src.services.rag_service import RAGService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
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

def get_rag_service(
    qdrant_client: QdrantClient = Depends(get_qdrant_client),
    embedding_service: EmbeddingService = Depends(get_embedding_service)
) -> RAGService:
    return RAGService(qdrant_client, embedding_service)

# Load Whisper model (you might want to make this configurable)
whisper_model = None

def get_whisper_model():
    global whisper_model
    if whisper_model is None:
        whisper_model = whisper.load_model("base")  # You can use "small", "medium", "large" for better accuracy
    return whisper_model


class VoiceRequest(BaseModel):
    audio_base64: str
    session_id: Optional[uuid.UUID] = None
    language: Optional[str] = None

class VoiceResponse(BaseModel):
    response: str
    session_id: uuid.UUID
    transcript: str
    context: Optional[list] = None


def decode_audio_from_base64(audio_base64: str) -> bytes:
    """Decode base64 audio data"""
    try:
        # Remove data URL prefix if present (e.g., "data:audio/wav;base64,")
        if "," in audio_base64:
            audio_base64 = audio_base64.split(",")[1]
        
        audio_data = base64.b64decode(audio_base64)
        return audio_data
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid base64 audio data: {str(e)}")


def convert_audio_to_wav(audio_data: bytes) -> bytes:
    """Convert audio data to WAV format using pydub"""
    try:
        # Create audio segment from bytes
        audio_segment = AudioSegment.from_file(io.BytesIO(audio_data))
        
        # Convert to WAV format
        wav_buffer = io.BytesIO()
        audio_segment.export(wav_buffer, format="wav")
        wav_buffer.seek(0)
        
        return wav_buffer.getvalue()
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Audio conversion failed: {str(e)}")


def transcribe_audio_whisper(audio_data: bytes, language: Optional[str] = None) -> str:
    """Transcribe audio using OpenAI Whisper"""
    try:
        model = get_whisper_model()
        
        # Save audio to temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_file.write(audio_data)
            temp_file_path = temp_file.name
        
        try:
            # Transcribe using Whisper
            result = model.transcribe(temp_file_path, language=language)
            transcript = result["text"].strip()
            
            if not transcript:
                raise HTTPException(status_code=400, detail="No speech detected in audio")
            
            return transcript
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
                
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Speech-to-text failed: {str(e)}")


def transcribe_audio_speech_recognition(audio_data: bytes) -> str:
    """Alternative transcription using SpeechRecognition library (fallback)"""
    try:
        recognizer = sr.Recognizer()
        
        # Save audio to temporary file
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_file.write(audio_data)
            temp_file_path = temp_file.name
        
        try:
            # Load audio file
            with sr.AudioFile(temp_file_path) as source:
                audio = recognizer.record(source)
            
            # Recognize speech using Google Web Speech API (requires internet)
            transcript = recognizer.recognize_google(audio)
            
            if not transcript:
                raise HTTPException(status_code=400, detail="No speech detected in audio")
            
            return transcript
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
                
    except sr.UnknownValueError:
        raise HTTPException(status_code=400, detail="Could not understand audio")
    except sr.RequestError as e:
        raise HTTPException(status_code=500, detail=f"Speech recognition service error: {str(e)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Speech-to-text failed: {str(e)}")


@router.post("/chat/voice", response_model=VoiceResponse)
async def voice_chat_endpoint(
    request: VoiceRequest,
    db: Session = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
):
    """
    Process voice input: decode audio, transcribe to text, process with RAG, return response
    """
    try:
        # Handle session
        session_id = request.session_id
        if session_id is None:
            # Create a new session if not provided
            db_session = crud.create_session(db)
            session_id = db_session.id
        else:
            db_session = crud.get_session(db, session_id)
            if not db_session:
                raise HTTPException(status_code=404, detail="Session not found")

        # Decode base64 audio
        audio_data = decode_audio_from_base64(request.audio_base64)
        
        # Convert to WAV format if needed
        wav_audio_data = convert_audio_to_wav(audio_data)
        
        # Transcribe audio to text
        try:
            # Try Whisper first (more accurate)
            transcript = transcribe_audio_whisper(wav_audio_data, request.language)
        except Exception as whisper_error:
            print(f"Whisper transcription failed: {whisper_error}")
            try:
                # Fallback to SpeechRecognition
                transcript = transcribe_audio_speech_recognition(wav_audio_data)
            except Exception as sr_error:
                print(f"SpeechRecognition fallback failed: {sr_error}")
                raise HTTPException(
                    status_code=500, 
                    detail="Both Whisper and SpeechRecognition failed. Please try again."
                )
        
        # Store user's voice message (transcript)
        crud.create_message(db, session_id, transcript, "user", {"input_type": "voice"})
        
        # Process transcript with RAG
        retrieved_context = await rag_service.retrieve_context(transcript)
        rag_response = await rag_service.generate_rag_response(transcript, retrieved_context)
        
        # Store AI response
        crud.create_message(
            db, 
            session_id, 
            rag_response, 
            "ai", 
            {"context": retrieved_context, "input_type": "voice_response"}
        )
        
        return VoiceResponse(
            response=rag_response,
            session_id=session_id,
            transcript=transcript,
            context=retrieved_context
        )
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle unexpected errors
        raise HTTPException(status_code=500, detail=f"Voice processing failed: {str(e)}")


@router.get("/chat/voice/health")
async def voice_health_check():
    """Health check endpoint for voice processing"""
    try:
        # Check if Whisper model can be loaded
        model = get_whisper_model()
        return {
            "status": "healthy",
            "whisper_model": "base",
            "services": ["whisper", "speech_recognition"]
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }