# Book Frontend-Backend Integration Plan

## 1. Scope and Dependencies

### In Scope:
-   Integration of the existing `book-frontend` (Docusaurus-based) with the `book-backend` FastAPI application.
-   Enabling the frontend to consume all public API endpoints exposed by the backend, including:
    -   Book content retrieval (chapters, personalized, translated).
    -   User authentication (signup, signin).
    -   User preference updates.
    -   RAG Chatbot interaction (query, context).
-   Configuration of CORS in the `book-backend` to allow `book-frontend` origin.
-   Handling of authentication tokens (e.g., JWT) on the frontend for authenticated requests.

### Out of Scope:
-   Any modifications to the core Docusaurus structure or styling of the `book-frontend` beyond necessary API integration points.
-   Re-implementing or significantly altering the `book-backend` API endpoints defined in `001-book-backend-api`.
-   Implementing complex caching strategies on the frontend beyond browser-level caching.

### External Dependencies:
-   **`book-frontend`**: Existing Docusaurus application. Specifics on how API calls are made (e.g., Fetch API, Axios) need to be understood.
-   **`book-backend`**: The FastAPI application developed under `001-book-backend-api`, exposing documented API endpoints.
-   **Better-Auth**: The authentication platform integrated with the backend, which the frontend will interact with indirectly via backend API endpoints.
-   **Docusaurus CSS**: As mentioned in the constitution, frontend will use Docusaurus CSS.

## 2. Key Decisions and Rationale

-   **Backend FastAPI Exposure**: The new FastAPI application (`book-backend/app/main.py`) will be mounted as a sub-application within the existing `book-backend/main.py` (which contains the OpenAI Agents SDK code). This approach ensures `main.py` remains the primary entry point while cleanly separating the new API functionalities under a clear path (e.g., `/api`).
    -   **Rationale**: Preserves the user's requirement to not change `main.py` significantly, provides a clear API boundary, and allows for modular development.
-   **CORS Configuration**: Implement `CORS_MIDDLEWARE` in the `book-backend` FastAPI application to explicitly allow requests from the `book-frontend`'s origin. This will be configurable via environment variables.
    -   **Rationale**: Essential for cross-origin communication between frontend and backend in different domains/ports, ensuring security and proper request handling.
-   **Authentication Token Handling**: The frontend will receive a JWT (or similar token) upon successful login from the backend. This token will be stored securely (e.g., `localStorage` or `sessionStorage` with appropriate security measures) and sent with subsequent authenticated requests in the `Authorization` header.
    -   **Rationale**: Follows standard practices for securing single-page applications with backend APIs, leveraging Better-Auth indirectly.

## 3. Interfaces and API Contracts

### Public APIs:
-   The `book-frontend` will interact with the following `book-backend` API endpoints (as defined in `001-book-backend-api/plan.md` and `spec.md`):
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}` (Retrieve chapter)
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}/personalized` (Retrieve personalized chapter)
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}/translated/{language_code}` (Retrieve translated chapter)
    -   `POST /api/v1/auth/signup` (User registration)
    -   `POST /api/v1/auth/signin` (User login)
    -   `PUT /api/v1/users/me/preferences` (Update user preferences)
    -   `POST /api/v1/chatbot/query` (Send chatbot query)
    -   `POST /api/v1/chatbot/context` (Provide chatbot context)

### Versioning Strategy:
-   Frontend will consume the `/api/v1/` endpoints, aligning with the backend's semantic versioning.

### Frontend Request/Response Formats:
-   Frontend will send JSON payloads for `POST` and `PUT` requests and expect JSON responses for all API calls.

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance:
-   **API Call Latency**: Frontend-perceived latency for critical API calls (e.g., chapter loading, chatbot responses) should be minimal (target < 500ms, including network and backend processing).
-   **Responsiveness**: Frontend should remain responsive during API calls, potentially using loading indicators.

### Reliability:
-   **Error Handling**: Frontend will implement robust error handling for API calls, displaying user-friendly messages for failures and gracefully degrading functionality where appropriate (e.g., if a personalization service is unavailable).
-   **Retry Mechanisms**: Frontend may implement client-side retry logic for transient network errors.

### Security:
-   **Secure Communication**: All communication between frontend and backend will occur over HTTPS.
-   **Token Storage**: Authentication tokens will be stored securely on the client-side (e.g., HTTP-only cookies if possible, or `localStorage` with XSS prevention measures).
-   **AuthZ Enforcement**: Frontend will respect backend's authorization responses, disabling/hiding features not accessible to the current user.

### Cost:
-   No direct cost implications from the integration itself, but will leverage existing infrastructure.

## 5. Data Management and Migration

-   **Data Flow**: Frontend will primarily display data fetched from the backend. Minimal client-side state management for user preferences or temporary data.
-   **No Migration**: No data migration is required for this integration; frontend will consume existing backend data.

## 6. Operational Readiness

### Observability:
-   **Frontend Logging**: Implement basic client-side logging for API errors and significant user actions, which can be reported to a monitoring system if applicable.
-   **Backend Metrics**: Continue leveraging backend metrics (from `001-book-backend-api` plan) to monitor API usage and performance as consumed by the frontend.

### Alerting:
-   Backend alerts (from `001-book-backend-api` plan) will cover API failures relevant to frontend consumption.

### Deployment and Rollback strategies:
-   Frontend and backend deployments will be independent but coordinated. Backend API must be stable before frontend deployment to new features.

## 7. Risk Analysis and Mitigation

### Top 3 Risks:
1.  **CORS Configuration Issues**: Incorrect CORS settings on the backend could block all frontend API requests, preventing integration.
    -   **Mitigation**: Thorough testing of CORS configuration in development and staging environments. Clear environment variable management for allowed origins. Utilize pre-flight request logging.
2.  **Authentication Mismatches**: Discrepancies in token generation/validation or storage mechanisms between frontend and backend could lead to failed authentication and unauthorized access issues.
    -   **Mitigation**: Standardized JWT implementation. Clear contract for token exchange. Comprehensive integration tests for authentication flow. Secure token storage on the frontend.
3.  **Frontend Breaking Changes from Backend**: Backend API changes not communicated or handled by the frontend could lead to a broken user experience.
    -   **Mitigation**: Semantic API versioning (`/api/v1/`). Clear communication channels between frontend and backend teams. Contract testing to ensure API adherence.

## 8. Evaluation and Validation

### Definition of Done:
-   The `book-frontend` successfully consumes all specified `book-backend` API endpoints.
-   Authentication flow is secure and functional from the frontend's perspective.
-   CORS is correctly configured, allowing frontend access.
-   User stories defined in `spec.md` are fully supported by the integration.

### Output Validation for Format/Requirements/Safety:
-   Frontend correctly parses and displays data received from the backend, validating data integrity.
-   Frontend properly handles API errors and displays informative messages.
-   Security audit of frontend token handling and backend CORS configuration.

## 9. Architectural Decision Record (ADR)

-   No immediate ADRs are suggested as the key architectural decisions (FastAPI mounting, CORS, token handling) are straightforward integrations of existing patterns. If more complex alternatives arise, an ADR will be created.

---

**Version**: 0.1.0 | **Last Updated**: 2025-12-05 | **Feature Branch**: `002-frontend-backend-integration`