"""
Load testing for RAG Chatbot API endpoints using Locust.
"""

import json
import random
from locust import HttpUser, task, between


class RAGChatbotUser(HttpUser):
    """Simulates user interactions with the RAG chatbot system."""
    
    wait_time = between(1, 3)  # Wait 1-3 seconds between requests
    
    def on_start(self):
        """Setup test data and session."""
        self.session_id = None
        self.test_queries = [
            "What is machine learning?",
            "Explain neural networks",
            "How does deep learning work?",
            "What are transformers in AI?",
            "Tell me about natural language processing",
            "What is computer vision?",
            "Explain reinforcement learning",
            "How do recommendation systems work?",
            "What is data science?",
            "Explain artificial intelligence"
        ]
        
    @task(5)
    def chat_rag_endpoint(self):
        """Test the main RAG chat endpoint."""
        query = random.choice(self.test_queries)
        
        payload = {
            "message": query,
            "session_id": self.session_id
        }
        
        with self.client.post(
            "/chat/rag",
            json=payload,
            catch_response=True
        ) as response:
            if response.status_code == 200:
                data = response.json()
                if "response" in data:
                    # Update session_id if returned
                    if "session_id" in data:
                        self.session_id = data["session_id"]
                    response.success()
                else:
                    response.failure("Missing response field")
            else:
                response.failure(f"Status code: {response.status_code}")
    
    @task(2)
    def chat_regular_endpoint(self):
        """Test the regular chat endpoint for comparison."""
        query = random.choice(self.test_queries)
        
        payload = {
            "message": query,
            "session_id": self.session_id
        }
        
        with self.client.post(
            "/chat",
            json=payload,
            catch_response=True
        ) as response:
            if response.status_code == 200:
                data = response.json()
                if "response" in data:
                    response.success()
                else:
                    response.failure("Missing response field")
            else:
                response.failure(f"Status code: {response.status_code}")
    
    @task(1)
    def health_check(self):
        """Test health endpoint."""
        with self.client.get("/health", catch_response=True) as response:
            if response.status_code == 200:
                response.success()
            else:
                response.failure(f"Health check failed: {response.status_code}")


class FileUploadUser(HttpUser):
    """Simulates file upload scenarios."""
    
    wait_time = between(2, 5)
    
    @task
    def upload_text_file(self):
        """Test file upload with text content."""
        # Create a simple text file content
        test_content = """
        This is a test document for load testing.
        It contains multiple paragraphs to simulate real document uploads.
        
        Machine learning is a subset of artificial intelligence that focuses on
        algorithms that can learn and make decisions from data.
        
        Deep learning uses neural networks with multiple layers to process
        complex patterns in data.
        """
        
        files = {
            'file': ('test_document.txt', test_content, 'text/plain')
        }
        
        with self.client.post(
            "/chat/file",
            files=files,
            catch_response=True
        ) as response:
            if response.status_code == 200:
                data = response.json()
                if "message" in data:
                    response.success()
                else:
                    response.failure("Missing response message")
            else:
                response.failure(f"File upload failed: {response.status_code}")


class VoiceChatUser(HttpUser):
    """Simulates voice chat scenarios (mock audio data)."""
    
    wait_time = between(3, 6)
    
    @task
    def voice_chat(self):
        """Test voice chat endpoint with mock audio data."""
        # Mock base64 audio data (in real scenario, this would be actual audio)
        mock_audio_data = "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT"
        
        payload = {
            "audio_data": mock_audio_data,
            "session_id": None
        }
        
        with self.client.post(
            "/chat/voice",
            json=payload,
            catch_response=True
        ) as response:
            if response.status_code == 200:
                data = response.json()
                if "response" in data:
                    response.success()
                else:
                    response.failure("Missing response field")
            else:
                response.failure(f"Voice chat failed: {response.status_code}")