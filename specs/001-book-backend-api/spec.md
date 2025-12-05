# AI Book Creation Project - Book Backend API Specification

## Feature: Book Backend API Enhancements

### 1. Introduction
This document outlines the features and requirements for enhancing the Book Backend API. The primary goal is to add new functionalities for managing book content, user personalization, multi-lingual support, and an integrated RAG chatbot, while strictly preserving the existing `main.py` (which contains `uv` and `openaiagents sdk` code) as a base.

### 2. Goals
- Provide robust API endpoints for CRUD operations on book chapters.
- Implement secure user authentication and authorization.
- Enable personalized book chapter content based on user preferences.
- Support multi-lingual content delivery, specifically Urdu translation.
- Integrate a RAG chatbot capable of answering questions about book content.
- Utilize Neon Serverless Postgres, Qdrant Cloud, and FastAPI for new features.

### 3. Non-Goals
- Modifying the existing `main.py` file.
- Frontend development or UI changes.
- Implementing a full-fledged content management system within this backend.

### 4. User Stories

#### P1 User Story: Book Content Management
As a content manager, I want to be able to create, read, update, and delete book chapters so that I can manage the book's content through the API.

**Acceptance Criteria:**
- API endpoints exist for:
    - Creating a new chapter (`POST /api/v1/books/{book_id}/chapters`)
    - Retrieving a chapter by ID (`GET /api/v1/books/{book_id}/chapters/{chapter_id}`)
    - Updating an existing chapter (`PUT /api/v1/books/{book_id}/chapters/{chapter_id}`)
    - Deleting a chapter (`DELETE /api/v1/books/{book_id}/chapters/{chapter_id}`)
- Each endpoint requires authentication.
- Chapters include fields for title, content (original), book ID, chapter number.

#### P1 User Story: Secure User Authentication (via Better-Auth)
As a user, I want to securely sign up and sign in to the application so that my personalized content and features are protected.

**Acceptance Criteria:**
- Integration with the Better-Auth platform for user registration and login.
- API endpoints for:
    - User registration (`POST /api/v1/auth/signup`)
    - User login (`POST /api/v1/auth/signin`)
- Successful authentication returns a valid token (e.g., JWT).
- All sensitive operations (e.g., personalization, translation, content management) require a valid authentication token.

#### P2 User Story: User-Centric Content Personalization
As a logged-in user, I want to receive personalized book chapter content based on my software and hardware background so that the content is more relevant to my learning needs.

**Acceptance Criteria:**
- During signup, users can provide their software and hardware background.
- API endpoint to update user preferences (`PUT /api/v1/users/me/preferences`).
- API endpoint to retrieve personalized chapter content (`GET /api/v1/books/{book_id}/chapters/{chapter_id}/personalized`) which customizes the original content based on stored preferences.
- Personalization logic should be configurable and extensible.

#### P2 User Story: Multi-Lingual Content Support (Urdu Translation)
As a logged-in user, I want to view book chapter content translated into Urdu so that I can read the book in my preferred language.

**Acceptance Criteria:**
- API endpoint to retrieve a translated version of a chapter into Urdu (`GET /api/v1/books/{book_id}/chapters/{chapter_id}/translated/ur`).
- The translation process should utilize an external translation service (e.g., Gemini API or similar).
- Translated content is stored for subsequent faster retrieval.

#### P3 User Story: Integrated RAG Chatbot Interaction
As a user, I want to ask questions about the book's content via an integrated chatbot and receive accurate answers, optionally providing specific text as context.

**Acceptance Criteria:**
- API endpoint to send a user query and receive a chatbot response (`POST /api/v1/chatbot/query`).
- The chatbot should leverage OpenAI Agents/ChatKit SDKs with Gemini API, Neon Serverless Postgres, and Qdrant Cloud.
- The chatbot should be able to answer questions based on the full book content.
- An additional API endpoint allows users to provide specific book text as context for the chatbot query (`POST /api/v1/chatbot/context`).

### 5. Data Models (Conceptual)
- **User**: ID, username, email, password_hash, preferences (JSONB).
- **Book**: ID, title, author, description.
- **Chapter**: ID, book_id, chapter_number, title, original_content, personalized_content (JSONB/text), translated_content (JSONB/text for different languages).
- **Vector Embedding**: Text content, vector, chapter_id, book_id.

### 6. Technical Considerations
- Python 3.11, FastAPI, uv.
- Neon Serverless Postgres for relational data.
- Qdrant Cloud for vector database operations.
- Gemini API for RAG chatbot (via OpenAI Agents SDK) and potentially translation.
- Better-Auth for authentication.
- Modular project structure for scalability and maintainability.

### 7. Open Questions / Dependencies
- Exact schema for user preferences.
- Details of personalization logic (e.g., rules engine, template substitution).
- Choice of external translation service if not Gemini API.
- Specifics of chatbot prompt engineering and knowledge base construction.
- Error handling strategy for external API failures.

---

**Version**: 0.1.0 | **Last Updated**: 2025-12-04 | **Feature Branch**: `001-book-backend-api`