# AI Book Creation Project - Book Backend API Tasks

**Feature**: `001-book-backend-api` | **Date**: 2025-12-04 | **Plan**: /home/hadiqa/Documents/SpecifyPlus/Hackthon/Book/specs/001-book-backend-api/plan.md

## Summary

This document outlines the tasks required to implement the enhanced Book Backend API functionality, adhering to the architectural plan and feature specification. Tasks are organized into phases, with clear dependencies and opportunities for parallel execution.

## Phase 1: Setup

_Initial project setup and dependency management._

- [x] T001 Create `book-backend/` directory if it doesn't exist, and create a basic `README.md` within it.
- [x] T002 Set up Python environment using `uv` and generate `requirements.txt` with initial dependencies (FastAPI, uvicorn, psycopg2/asyncpg, python-dotenv, qdrant-client, openai, httpx).
  - `book-backend/requirements.txt`
- [x] T003 Create `book-backend/app/` directory structure: `app/__init__.py`, `app/api/v1/__init__.py`, `app/services/__init__.py`, `app/models/__init__.py`, `app/core/__init__.py`.
- [x] T004 Configure initial `.env` file for environment variables (e.g., `DATABASE_URL`, `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `BETTER_AUTH_API_KEY`).
  - `book-backend/.env`

## Phase 2: Foundational Services

_Core infrastructure and database connections, blocking prerequisites for user stories._

- [x] T005 Initialize FastAPI application in `book-backend/app/main.py`. This will be the entry point for the new API functionalities.
  - `book-backend/app/main.py`
- [x] T006 Integrate FastAPI application with the existing `main.py` (OpenAI Agents SDK), ensuring existing functionality is preserved. This might involve careful routing or mounting the FastAPI app.
  - `book-backend/main.py` (existing, will be minimally modified to include new app)
  - `book-backend/app/main.py` (new FastAPI app)
- [x] T007 Set up database connection to Neon Serverless Postgres. Implement `db_service.py` for SQLAlchemy/SQLModel engine and session management.
  - `book-backend/app/core/config.py`
  - `book-backend/app/services/db_service.py`
- [x] T008 Implement basic database session dependency for FastAPI routes.
  - `book-backend/app/api/dependencies.py`
- [x] T009 Set up Qdrant client connection and collection initialization in `book-backend/app/services/qdrant_service.py` (new file).
  - `book-backend/app/core/config.py`
  - `book-backend/app/services/qdrant_service.py`

## Phase 3: P1 User Story: Book Content Management [US1]

_As a content manager, I want to be able to create, read, update, and delete book chapters so that I can manage the book's content through the API._

**Independent Test Criteria**: New chapters can be created, retrieved, updated, and deleted via authenticated API calls. All operations handle valid data and return appropriate responses.

- [x] T010 [US1] Create `Book` and `Chapter` Pydantic models for request/response and SQLAlchemy/SQLModel ORM.
  - `book-backend/app/models/content.py`
- [x] T011 [US1] Implement database schema migration (e.g., using Alembic if chosen, or direct table creation/update) for `Book` and `Chapter` tables.
  - `book-backend/app/services/db_service.py` (for initial table creation/update logic if no Alembic)
- [x] T012 [P] [US1] Implement `ContentService` in `book-backend/app/services/content_service.py` with CRUD logic for `Book` and `Chapter` entities.
  - `book-backend/app/services/content_service.py`
- [x] T013 [P] [US1] Create FastAPI endpoints for chapter management (Create, Retrieve, Update, Delete) in `book-backend/app/api/v1/content.py`. These endpoints will interact with `ContentService`.
  - `book-backend/app/api/v1/content.py`

## Phase 4: P1 User Story: Secure User Authentication (via Better-Auth) [US2]

_As a user, I want to securely sign up and sign in to the application so that my personalized content and features are protected._

**Independent Test Criteria**: Users can register, log in, receive a valid authentication token. Protected endpoints are inaccessible without a valid token. Unauthorized access attempts are rejected.

- [x] T014 [US2] Create `User` Pydantic model for request/response and SQLAlchemy/SQLModel ORM, including fields for username, email, password hash, and `preferences` (JSONB).
  - `book-backend/app/models/user.py`
- [x] T015 [US2] Implement database schema migration for the `User` table.
  - `book-backend/app/services/db_service.py` (or Alembic migration script)
- [x] T016 [P] [US2] Implement `AuthService` in `book-backend/app/services/auth_service.py` for Better-Auth integration, user registration, login (password hashing, token generation), and token validation logic.
  - `book-backend/app/services/auth_service.py`
- [x] T017 [P] [US2] Create FastAPI endpoints for user authentication (signup, signin) in `book-backend/app/api/v1/auth.py`. These endpoints will use `AuthService`.
  - `book-backend/app/api/v1/auth.py`
- [x] T018 [US2] Implement FastAPI dependency for token-based authentication and authorization for protected routes. This dependency will validate the token and inject the current user into the request.
  - `book-backend/app/api/dependencies.py`

## Phase 5: P2 User Story: User-Centric Content Personalization [US3]

_As a logged-in user, I want to receive personalized book chapter content based on my software and hardware background so that the content is more relevant to my learning needs._

**Independent Test Criteria**: User preferences can be updated via API. Personalized chapter content is delivered via API, dynamically customizing original content based on the stored user preferences. Unauthenticated requests fail.

- [x] T019 [US3] Ensure `User` model in `book-backend/app/models/user.py` includes `preferences` (JSONB) and update schema if not done in T014.
  - `book-backend/app/models/user.py`
- [x] T020 [P] [US3] Extend `AuthService` (or create `UserService` if deemed more appropriate for user profile management) in `book-backend/app/services/auth_service.py` to handle updating and retrieving user preferences.
  - `book-backend/app/services/auth_service.py`
- [x] T021 [P] [US3] Create FastAPI endpoint to update user preferences in `book-backend/app/api/v1/auth.py` (or a new `user.py` if `UserService` is created). This endpoint will be protected.
  - `book-backend/app/api/v1/auth.py`
- [x] T022 [US3] Modify `ContentService` in `book-backend/app/services/content_service.py` to include logic for applying personalization rules to chapter content based on a given user's preferences.
  - `book-backend/app/services/content_service.py`
- [x] T023 [US3] Create FastAPI endpoint to retrieve personalized chapter content in `book-backend/app/api/v1/content.py`. This endpoint will be protected and use the personalization logic from `ContentService`.
  - `book-backend/app/api/v1/content.py`

## Phase 6: P2 User Story: Multi-Lingual Content Support (Urdu Translation) [US4]

_As a logged-in user, I want to view book chapter content translated into Urdu so that I can read the book in my preferred language._

**Independent Test Criteria**: A chapter can be requested in Urdu via API and returned successfully. Subsequent requests for the same translated chapter are served from cache/storage, indicating faster retrieval. Translation fidelity is acceptable.

- [x] T024 [US4] Extend `Chapter` model in `book-backend/app/models/content.py` to include `translated_content` (JSONB field to store translations for various languages).
  - `book-backend/app/models/content.py`
- [x] T025 [US4] Update `Chapter` table schema to include `translated_content` column.
  - `book-backend/app/services/db_service.py` (or Alembic migration script)
- [x] T026 [P] [US4] Implement `TranslationService` in `book-backend/app/services/translation_service.py` (new file) to handle translation requests (using Gemini API) and caching/storage of translated content in the database.
  - `book-backend/app/services/translation_service.py`
- [x] T027 [US4] Modify `ContentService` in `book-backend/app/services/content_service.py` to interact with `TranslationService` to request and retrieve Urdu translations for chapters.
  - `book-backend/app/services/content_service.py`
- [x] T028 [US4] Create FastAPI endpoint to retrieve translated chapter content (e.g., for Urdu) in `book-backend/app/api/v1/content.py`. This endpoint will be protected.
  - `book-backend/app/api/v1/content.py`

## Phase 7: P3 User Story: Integrated RAG Chatbot Interaction [US5]

_As a user, I want to ask questions about the book's content via an integrated chatbot and receive accurate answers, optionally providing specific text as context._

**Independent Test Criteria**: Chatbot provides relevant and accurate answers to book-related queries. Chatbot can successfully incorporate user-provided context to refine its responses. Unauthenticated requests are handled appropriately.

- [x] T029 [US5] Implement `ChatbotService` in `book-backend/app/services/chatbot_service.py` for RAG logic. This service will orchestrate interactions with OpenAI Agents SDK, Gemini API, Neon DB (for retrieving source content), and Qdrant (for vector search).
  - `book-backend/app/services/chatbot_service.py`
- [x] T030 [US5] Design and implement vector embedding generation logic (e.g., using Gemini API or another embedding model) within `ChatbotService` for book content.
  - `book-backend/app/services/chatbot_service.py`
- [x] T031 [US5] Develop a script or function to initially populate Qdrant with vector embeddings of existing book content from the Neon Postgres database.
  - `book-backend/scripts/ingest_content.py` (new file, can be run as a one-off or scheduled task)
- [x] T032 [P] [US5] Create FastAPI endpoint for chatbot queries (`POST /api/v1/chatbot/query`) in `book-backend/app/api/v1/chatbot.py`. This endpoint will pass the query to `ChatbotService`.
  - `book-backend/app/api/v1/chatbot.py`
- [x] T033 [P] [US5] Create FastAPI endpoint for providing chatbot context (`POST /api/v1/chatbot/context`) in `book-backend/app/api/v1/chatbot.py`. This endpoint will allow users to specify text from the book to guide the chatbot's response.
  - `book-backend/app/api/v1/chatbot.py`

## Phase 8: Polish & Cross-Cutting Concerns

_Finalizing the application, improving robustness, and ensuring quality._

- [x] T034 Implement comprehensive logging configuration (e.g., `logging` module) in `book-backend/app/core/config.py` and ensure logs are structured and informative.
  - `book-backend/app/core/config.py`
- [x] T035 Add global error handling middleware for FastAPI to catch unhandled exceptions and return consistent error responses.
  - `book-backend/app/main.py` (FastAPI app)
- [x] T036 Configure CORS (Cross-Origin Resource Sharing) settings for the FastAPI application to allow requests from the frontend.
  - `book-backend/app/main.py` (FastAPI app)
- [x] T037 Write basic unit and integration tests for core services and API endpoints using `pytest`.
  - `book-backend/tests/unit/`
  - `book-backend/tests/integration/`
- [x] T038 Update `README.md` in `book-backend/` with setup instructions, environment variable requirements, and basic API usage/documentation.
  - `book-backend/README.md`
- [x] T039 Review and verify security measures, ensuring proper use of environment variables, secure authentication (Better-Auth), and input validation.

## Dependencies Graph

_Shows the recommended order of completing user stories and phases._

- **Phase 1 (Setup)**: No direct dependencies.
- **Phase 2 (Foundational Services)**: Depends on Phase 1.
- **Phase 3 (Book Content Management)**: Depends on Phase 2.
- **Phase 4 (Secure User Authentication)**: Depends on Phase 2.
- **Phase 5 (User-Centric Content Personalization)**: Depends on Phase 3 (for content access), Phase 4 (for user authentication).
- **Phase 6 (Multi-Lingual Content Support)**: Depends on Phase 3 (for content access), Phase 4 (for user authentication).
- **Phase 7 (Integrated RAG Chatbot Interaction)**: Depends on Phase 3 (for content access), Phase 4 (for user authentication), Phase 2 (for Qdrant/Gemini setup).
- **Phase 8 (Polish & Cross-Cutting Concerns)**: Depends on completion of Phases 3, 4, 5, 6, and 7.

## Parallel Execution Examples per User Story

_Tasks within a user story that can be worked on concurrently._

- **P1 Book Content Management [US1]**: Tasks T012 (ContentService implementation) and T013 (FastAPI endpoints) can be developed in parallel after the models (T010) and database schema (T011) are established.
- **P1 Secure User Authentication [US2]**: Tasks T016 (AuthService implementation) and T017 (FastAPI endpoints) can be developed in parallel after the User model (T014) and database schema (T015) are in place.
- **P2 User-Centric Content Personalization [US3]**: Tasks T020 (update AuthService/UserService) and T021 (API endpoint for preferences) can proceed in parallel after model extensions (T019) are done. T022 (ContentService modification) and T023 (personalized chapter endpoint) can then be done in parallel.
- **P2 Multi-Lingual Content Support [US4]**: Task T026 (TranslationService) can be developed in parallel with modifications to ContentService (T027) and the API endpoint (T028), once the Chapter model is extended (T024) and schema updated (T025).
- **P3 Integrated RAG Chatbot Interaction [US5]**: Tasks T032 (chatbot query endpoint) and T033 (chatbot context endpoint) can be developed in parallel after `ChatbotService` implementation (T029), vector embedding logic (T030), and Qdrant population (T031) are complete.

## Implementation Strategy

- **MVP First**: Prioritize the completion of Phase 1 (Setup), Phase 2 (Foundational Services), and the P1 User Stories (Book Content Management and Secure User Authentication) to establish a functional core API.
- **Incremental Delivery**: Subsequent P2 and P3 User Stories will be implemented incrementally, ensuring each feature is thoroughly tested and integrated before moving to the next. This approach allows for continuous feedback and reduces risk.
