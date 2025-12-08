"""Integration tests for API endpoints"""

import pytest
from fastapi.testclient import TestClient


class TestContentEndpoints:
    """Test content management endpoints"""
    
    def test_create_book(self, client, sample_book_data):
        """Test creating a book via API"""
        response = client.post("/api/v1/content/books", json=sample_book_data)
        
        assert response.status_code == 201
        data = response.json()
        assert data["title"] == sample_book_data["title"]
        assert data["author"] == sample_book_data["author"]
        assert "id" in data
    
    def test_get_book(self, client, sample_book_data):
        """Test retrieving a book via API"""
        # Create book
        create_response = client.post("/api/v1/content/books", json=sample_book_data)
        book_id = create_response.json()["id"]
        
        # Get book
        response = client.get(f"/api/v1/content/books/{book_id}")
        
        assert response.status_code == 200
        data = response.json()
        assert data["id"] == book_id
        assert data["title"] == sample_book_data["title"]
    
    def test_get_nonexistent_book(self, client):
        """Test retrieving a non-existent book"""
        response = client.get("/api/v1/content/books/999")
        
        assert response.status_code == 404
    
    def test_create_chapter(self, client, sample_book_data, sample_chapter_data):
        """Test creating a chapter via API"""
        # Create book first
        book_response = client.post("/api/v1/content/books", json=sample_book_data)
        book_id = book_response.json()["id"]
        
        # Create chapter
        chapter_data = {**sample_chapter_data, "book_id": book_id}
        response = client.post("/api/v1/content/chapters", json=chapter_data)
        
        assert response.status_code == 201
        data = response.json()
        assert data["title"] == sample_chapter_data["title"]
        assert data["book_id"] == book_id
    
    def test_get_book_chapters(self, client, sample_book_data):
        """Test retrieving chapters for a book"""
        # Create book
        book_response = client.post("/api/v1/content/books", json=sample_book_data)
        book_id = book_response.json()["id"]
        
        # Create chapters
        chapter1_data = {
            "title": "Chapter 1",
            "chapter_number": 1,
            "content": "Content 1",
            "book_id": book_id
        }
        chapter2_data = {
            "title": "Chapter 2", 
            "chapter_number": 2,
            "content": "Content 2",
            "book_id": book_id
        }
        
        client.post("/api/v1/content/chapters", json=chapter1_data)
        client.post("/api/v1/content/chapters", json=chapter2_data)
        
        # Get chapters
        response = client.get(f"/api/v1/content/books/{book_id}/chapters")
        
        assert response.status_code == 200
        data = response.json()
        assert len(data) == 2


class TestAuthEndpoints:
    """Test authentication endpoints"""
    
    def test_register_user(self, client, sample_user_data):
        """Test user registration"""
        response = client.post("/api/v1/auth/register", json=sample_user_data)
        
        assert response.status_code == 201
        data = response.json()
        assert data["username"] == sample_user_data["username"]
        assert data["email"] == sample_user_data["email"]
        assert "password" not in data  # Password should not be returned
    
    def test_register_duplicate_user(self, client, sample_user_data):
        """Test registering duplicate user"""
        # Register first user
        client.post("/api/v1/auth/register", json=sample_user_data)
        
        # Try to register same user again
        response = client.post("/api/v1/auth/register", json=sample_user_data)
        
        assert response.status_code == 400
    
    def test_login_user(self, client, sample_user_data):
        """Test user login"""
        # Register user first
        client.post("/api/v1/auth/register", json=sample_user_data)
        
        # Login
        login_data = {
            "username": sample_user_data["username"],
            "password": sample_user_data["password"]
        }
        response = client.post("/api/v1/auth/login", json=login_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "access_token" in data
        assert data["token_type"] == "bearer"
    
    def test_login_invalid_credentials(self, client):
        """Test login with invalid credentials"""
        login_data = {
            "username": "nonexistent",
            "password": "wrongpassword"
        }
        response = client.post("/api/v1/auth/login", json=login_data)
        
        assert response.status_code == 401


class TestChatbotEndpoints:
    """Test chatbot endpoints"""
    
    def test_create_chat_session(self, client):
        """Test creating a chat session"""
        session_data = {
            "title": "Test Chat Session",
            "session_type": "general",
            "language": "en"
        }
        
        response = client.post("/api/v1/chatbot/sessions", json=session_data)
        
        assert response.status_code == 201
        data = response.json()
        assert data["title"] == session_data["title"]
        assert data["session_type"] == session_data["session_type"]
        assert "id" in data
    
    def test_get_chat_session(self, client):
        """Test retrieving a chat session"""
        # Create session
        session_data = {
            "title": "Test Chat Session",
            "session_type": "general",
            "language": "en"
        }
        create_response = client.post("/api/v1/chatbot/sessions", json=session_data)
        session_id = create_response.json()["id"]
        
        # Get session
        response = client.get(f"/api/v1/chatbot/sessions/{session_id}")
        
        assert response.status_code == 200
        data = response.json()
        assert data["id"] == session_id
        assert data["title"] == session_data["title"]
    
    def test_chat_query(self, client):
        """Test chatbot query"""
        # Create session
        session_data = {
            "title": "Test Chat Session",
            "session_type": "general",
            "language": "en"
        }
        session_response = client.post("/api/v1/chatbot/sessions", json=session_data)
        session_id = session_response.json()["id"]
        
        # Send chat query
        query_data = {
            "session_id": session_id,
            "message": "Hello, what can you tell me about this book?",
            "use_rag": False  # Disable RAG for testing
        }
        
        response = client.post("/api/v1/chatbot/query", json=query_data)
        
        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert "session_id" in data
        assert data["session_id"] == session_id
    
    def test_get_chat_suggestions(self, client):
        """Test getting chat suggestions"""
        # Create session
        session_data = {
            "title": "Test Chat Session",
            "session_type": "general",
            "language": "en"
        }
        session_response = client.post("/api/v1/chatbot/sessions", json=session_data)
        session_id = session_response.json()["id"]
        
        # Get suggestions
        response = client.get(f"/api/v1/chatbot/suggestions/{session_id}")
        
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)


class TestHealthEndpoints:
    """Test health and status endpoints"""
    
    def test_root_endpoint(self, client):
        """Test root endpoint"""
        response = client.get("/")
        
        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert "version" in data
    
    def test_health_check(self, client):
        """Test health check endpoint"""
        response = client.get("/health")
        
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "service" in data