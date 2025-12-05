# Implementation Plan: 001-book-backend-api

**Branch**: `001-book-backend-api` | **Date**: 2025-12-04 | **Spec**: /home/hadiqa/Documents/SpecifyPlus/Hackthon/Book/specs/001-book-backend-api/spec.md
**Input**: Feature specification from `/specs/001-book-backend-api/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Develop additional functionality for the book-backend API. This involves integrating with Neon Serverless Postgres for data storage, Qdrant Cloud Free Tier for vector database capabilities (for RAG chatbot), and exposing new FastAPI endpoints for managing book content, user personalization data, and multi-lingual content support. The existing `main.py`, which contains `uv` and `openaiagents sdk` code, will remain unchanged as a base, with new functionalities added alongside it.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents/ChatKit SDKs, Gemini API, uv, Neon Serverless Postgres client (e.g., asyncpg or psycopg3 with SQLAlchemy/SQLModel), Qdrant client, Better-Auth client library (if available, otherwise custom integration).
**Storage**: Neon Serverless Postgres database (structured data), Qdrant Cloud Free Tier (vector embeddings).
**Testing**: pytest (unit, integration, contract tests).
**Target Platform**: Linux server (containerized deployment, e.g., Docker).
**Performance Goals**:
- P95 latency under 200ms for core API endpoints.
- Support up to X requests/second (to be determined based on expected user load and specific endpoint characteristics).
- Efficient RAG chatbot responses (latency and accuracy).
**Constraints**:
- Do not modify `main.py` directly; all new functionality must be added in separate modules/files.
- Utilize environment variables for all sensitive configurations (API keys, database URLs).
- Adhere to the `AI Book Creation Project Constitution` principles.
**Scale/Scope**:
- Initial deployment targeting a small to medium user base for a single book.
- Architecture should be extensible to support multiple books and increased user load in the future.
- Focus on robust API endpoints for content delivery, personalization, translation, and RAG chatbot interaction.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Docusaurus & Spec-Kit Plus for Book Development**: The backend will provide data/APIs to support the Docusaurus frontend. (Compliant)
- **Integrated RAG Chatbot Development**: This is a core feature of the backend, involving OpenAI Agents/ChatKit SDKs, Gemini API, FastAPI, Neon Serverless Postgres, and Qdrant. (Compliant)
- **Reusable Intelligence via Claude Code Subagents & Skills**: Backend services might expose APIs or integrate with subagents. (Compliant)
- **Secure User Authentication (Better-Auth)**: The backend will integrate with Better-Auth for user authentication. (Compliant)
- **User-Centric Content Personalization**: The backend will handle storing and serving personalized book chapter content. (Compliant)
- **Multi-Lingual Content Support (Urdu Translation)**: The backend will provide the content to be translated and manage its storage. (Compliant)

## Project Structure

### Documentation (this feature)

```text
specs/001-book-backend-api/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-backend/
├── main.py              # Existing base code with uv and openaiagents sdk
├── app/
│   ├── __init__.py
│   ├── api/             # FastAPI endpoints for new functionalities
│   │   ├── __init__.py
│   │   └── v1/
│   │       ├── __init__.py
│   │       ├── auth.py    # Better-Auth integration
│   │       ├── content.py # Personalized and multi-lingual content
│   │       └── chatbot.py # RAG Chatbot related endpoints
│   ├── services/        # Business logic and external integrations
│   │   ├── __init__.py
│   │   ├── auth_service.py
│   │   ├── content_service.py
│   │   ├── chatbot_service.py
│   │   └── db_service.py    # Neon Postgres interaction
│   ├── models/          # Pydantic models for data
│   │   ├── __init__.py
│   │   ├── user.py
│   │   ├── content.py
│   │   └── chatbot.py
│   └── core/            # Configuration, utilities
│       ├── __init__.py
│       └── config.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── .env                 # Environment variables (e.g., API keys, DB connection)
├── requirements.txt     # Project dependencies
└── README.md
```

**Structure Decision**: The project will adopt a modular structure under `book-backend/app/` to segregate new functionalities (API, services, models) from the existing `main.py`. This ensures `main.py` remains untouched as a base while allowing for scalable and organized development of new features.


## 2. Key Decisions and Rationale

-   **Database choice**: Neon Serverless Postgres for scalability, serverless architecture, and managed service benefits. Its PostgreSQL compatibility ensures wide tool support.
-   **Vector database choice**: Qdrant Cloud Free Tier for efficient vector similarity search required for RAG capabilities, leveraging its cloud offering for ease of setup and reduced operational overhead.
-   **API Framework**: FastAPI for high performance, asynchronous support, ease of use, and automatic generation of OpenAPI documentation, which aligns well with a microservices approach.

## 3. Interfaces and API Contracts

### Public APIs:
-   **Book Content Endpoints**: CRUD operations for managing book chapters, sections, and their content.
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}`: Retrieve a specific chapter.
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}/personalized`: Retrieve a personalized version of a chapter (requires authentication and user preferences).
    -   `GET /api/v1/books/{book_id}/chapters/{chapter_id}/translated/{language_code}`: Retrieve a translated version of a chapter (requires authentication).
-   **User Personalization Endpoints**: Store and retrieve user-specific preferences (e.g., software/hardware background).
    -   `POST /api/v1/users/me/preferences`: Update user preferences.
    -   `GET /api/v1/users/me/preferences`: Retrieve user preferences.
-   **RAG Chatbot Endpoints**: For interacting with the embedded chatbot.
    -   `POST /api/v1/chatbot/query`: Send a user query and receive a chatbot response.
    -   `POST /api/v1/chatbot/context`: Provide specific text from the book as context for the chatbot (user-selected text).

### Versioning Strategy:
-   Semantic versioning for API endpoints (e.g., `/api/v1/...`) to ensure backward compatibility and clear evolution.

### Idempotency, Timeouts, Retries:
-   Implement idempotency for critical write operations (e.g., updating user preferences) using unique request IDs.
-   Configure appropriate timeouts for external API calls (e.g., Gemini API, Qdrant) to prevent hanging requests.
-   Implement retry mechanisms with exponential backoff for transient errors in external service calls.

### Error Taxonomy with Status Codes:
-   Utilize standard HTTP status codes (e.g., 200 OK, 201 Created, 400 Bad Request, 401 Unauthorized, 403 Forbidden, 404 Not Found, 500 Internal Server Error).
-   Provide clear, consistent, and machine-readable error messages in API responses, including an error code for programmatic handling.

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance:
-   **p95 latency**: Target under 200ms for core API endpoints (content retrieval, personalization, chatbot queries).
-   **Throughput**: Initial target of 100 requests per second, scalable to higher loads.
-   **Resource caps**: Monitor and optimize resource consumption on Neon and Qdrant to stay within free tier limits initially, with clear scaling paths.

### Reliability:
-   **SLOs**: Define specific Service Level Objectives for API availability (e.g., 99.9% uptime) and error rates (e.g., <0.1% server errors).
-   **Error budgets**: Allocate acceptable error rates for different services; track and alert when budgets are exceeded.
-   **Degradation strategy**: Implement graceful degradation for non-critical features (e.g., temporarily disable personalization if external service is down) during high load or partial service outages.

### Security:
-   **AuthN/AuthZ**: Leverage Better-Auth for robust user authentication and authorization across all protected endpoints. Implement token-based authentication (e.g., JWT).
-   **Data handling**: Implement data encryption at rest (Neon, Qdrant) and in transit (HTTPS/TLS) for sensitive user and book content. Adhere to relevant privacy regulations (e.g., GDPR, CCPA).
-   **Secrets**: Store API keys, database credentials, and other sensitive information securely using environment variables, managed by deployment platform (e.g., Docker secrets, Kubernetes secrets, or cloud-specific secret managers).
-   **Auditing**: Implement comprehensive logging for security-relevant events (e.g., failed login attempts, unauthorized access, data modification).

### Cost:
-   Optimize resource usage to stay within the free tiers of Qdrant Cloud and manage Neon Serverless Postgres costs efficiently through connection pooling and query optimization.
-   Monitor Gemini API usage to manage costs.

## 5. Data Management and Migration

### Source of Truth:
-   **Neon Serverless Postgres**: Primary source of truth for all structured application data, including user profiles, book metadata, chapter content (original and translated versions), and personalization settings.
-   **Qdrant Cloud**: Source of truth for vector embeddings of book content, used for efficient semantic search within the RAG chatbot.

### Schema Evolution:
-   Use database migration tools (e.g., Alembic for SQLAlchemy/SQLModel) to manage schema changes in Neon in a controlled and versioned manner.
-   Plan for backward compatibility during schema updates to minimize impact on existing data and clients.

### Migration and Rollback:
-   Define clear procedures for data migration (e.g., when introducing new features or schema changes) and rollback strategies for quick recovery in case of deployment issues.

### Data Retention:
-   Establish data retention policies for all stored data (e.g., user activity logs, deprecated content versions), complying with legal and business requirements.

## 6. Operational Readiness

### Observability:
-   **Logs**: Implement structured logging (e.g., JSON format) for API requests, errors, warnings, and key application events. Centralize logs for easy analysis.
-   **Metrics**: Collect metrics on API latency, request rates, error rates, database query performance, external service call performance, and resource utilization. Use Prometheus/Grafana or similar.
-   **Traces**: Implement distributed tracing (e.g., OpenTelemetry) to track requests across different services (FastAPI, database, external APIs) for debugging and performance profiling.

### Alerting:
-   Set up alerts for critical errors (e.g., high rate of 5xx errors, database connection failures, external API timeouts), performance degradation (e.g., sustained high latency, low throughput), and service unavailability.
-   Define on-call owners and escalation paths for different service components.

### Runbooks for Common Tasks:
-   Create comprehensive runbooks for common operational tasks (e.g., deployment, scaling), troubleshooting guides for known issues, and incident response procedures.

### Deployment and Rollback Strategies:
-   Implement automated CI/CD pipelines (e.g., GitHub Actions, GitLab CI/CD) for deploying to production and staging environments.
-   Enable quick and reliable rollback mechanisms to previous stable versions with minimal service interruption.

### Feature Flags and Compatibility:
-   Utilize feature flags for new functionalities to enable controlled rollouts (e.g., canary deployments, A/B testing) and easy disabling if issues arise without requiring a full redeploy.

## 7. Risk Analysis and Mitigation

### Top 3 Risks:
1.  **Complexity of Integrations**: Integrating multiple external services (OpenAI Agents, Gemini API, Neon, Qdrant, Better-Auth) could lead to integration challenges, increased development time, brittle dependencies, and potential single points of failure.
    -   **Mitigation**: Adopt a phased integration approach, starting with mock services; enforce clear API contracts; implement thorough unit and integration testing; utilize robust error handling, circuit breakers, and fallbacks for external calls.
2.  **Performance Bottlenecks with RAG Chatbot**: The RAG chatbot's performance (latency and accuracy) could be significantly impacted by the efficiency of vector search in Qdrant, the speed and rate limits of the Gemini API, and the overall processing logic for retrieval and generation.
    -   **Mitigation**: Rigorous performance testing and profiling; optimize Qdrant indexing, query strategies, and data chunking; implement caching mechanisms for frequent queries; utilize asynchronous processing to maximize concurrency; explore Gemini API's rate limit management.
3.  **Data Security and Privacy**: Handling user personalization data (potentially sensitive), book content, and authentication through Better-Auth requires stringent security measures to prevent data breaches, unauthorized access, and ensure compliance with privacy regulations.
    -   **Mitigation**: Adherence to industry-standard security best practices (OWASP Top 10); implement data encryption at rest and in transit; conduct regular security audits and penetration testing; enforce strict access controls and principle of least privilege; ensure compliance with data privacy regulations (e.g., GDPR, CCPA).

## 8. Evaluation and Validation

### Definition of Done:
-   All API endpoints for book management, user personalization, multi-lingual content, and RAG chatbot interaction are implemented, thoroughly tested (unit, integration, end-to-end), and meet defined performance and reliability targets.
-   All external service integrations (Neon, Qdrant, Gemini API, Better-Auth) are functional, robust, and correctly configured.
-   Security measures are implemented, verified through audits/scans, and comply with project constitution.
-   Comprehensive documentation for API endpoints, data models, and operational procedures is complete and up-to-date.

### Output Validation for Format/Requirements/Safety:
-   Implement comprehensive input validation for all API requests (e.g., Pydantic models in FastAPI) to prevent invalid data and security vulnerabilities.
-   Ensure API responses conform to defined OpenAPI contracts and data formats.
-   Conduct automated security scanning (SAST, DAST) and manual penetration testing to identify and address vulnerabilities before deployment.

## 9. Architectural Decision Record (ADR)

-   Architectural Decision Record (ADR) will be suggested if significant architectural decisions arise during the detailed planning and implementation phases that meet the criteria of impact, alternatives, and scope outlined in the project constitution.
