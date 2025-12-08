# API Contracts

**Feature**: 002-gazebo-unity-sim (Module 2: The Digital Twin)
**Date**: 2025-12-07

This directory contains API contracts and interface specifications for the RAG chatbot backend.

## Files

### chatbot-api.yaml

**Format**: OpenAPI 3.0.3

**Description**: REST API specification for the RAG chatbot that powers the embedded Q&A widget in the Docusaurus book.

**Key Endpoints**:
- `POST /query` - Submit a natural language query to the chatbot
- `GET /query/{query_id}` - Retrieve a previous query result
- `POST /feedback` - Submit user feedback (helpful/not helpful) on responses
- `GET /health` - Health check for API and dependencies (Qdrant, PostgreSQL, LLM)

**Authentication**: Optional API key authentication (for future rate limiting by key)

**Error Handling**:
- Standard HTTP status codes (200, 400, 404, 429, 500, 503)
- Machine-readable error codes for client-side handling
- Detailed error messages for debugging

**Rate Limiting**: 10 requests per minute per session_id (configurable)

**Dependencies**:
- **Qdrant**: Vector database for semantic search over document chunks
- **PostgreSQL/Neon**: Relational database for query history, retrievals, and feedback
- **LLM**: Large language model for answer generation (e.g., GPT-4, Claude, or open-source alternative)

## Usage

### Validation

Validate the OpenAPI spec using:

```bash
# Using openapi-generator-cli
openapi-generator-cli validate -i chatbot-api.yaml

# Or using Swagger Editor online
# Visit: https://editor.swagger.io/
# Import chatbot-api.yaml
```

### Code Generation

Generate client/server code:

```bash
# Generate Python FastAPI server
openapi-generator-cli generate \
  -i chatbot-api.yaml \
  -g python-fastapi \
  -o ../../chatbot/src/api

# Generate TypeScript React client
openapi-generator-cli generate \
  -i chatbot-api.yaml \
  -g typescript-axios \
  -o ../../docusaurus/src/api-client
```

### Testing

Example cURL requests:

```bash
# Submit a query
curl -X POST https://chatbot-api.example.com/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I set up gravity in Gazebo?",
    "session_id": "test-session-123"
  }'

# Submit feedback
curl -X POST https://chatbot-api.example.com/v1/feedback \
  -H "Content-Type: application/json" \
  -d '{
    "query_id": "550e8400-e29b-41d4-a716-446655440000",
    "is_helpful": true,
    "feedback_text": "Very helpful explanation!"
  }'

# Health check
curl https://chatbot-api.example.com/v1/health
```

## Contract Evolution

**Versioning**: API follows semantic versioning (MAJOR.MINOR.PATCH)

**Current Version**: 1.0.0

**Changes**:
- v1.0.0 (2025-12-07): Initial contract definition for Module 2 planning

**Breaking Changes Policy**:
- Breaking changes require MAJOR version increment
- New optional fields/endpoints require MINOR version increment
- Bug fixes and documentation updates require PATCH version increment

## Related Documents

- [../data-model.md](../data-model.md) - Entity definitions and validation rules
- [../research.md](../research.md) - RAG architecture decisions
- [../plan.md](../plan.md) - Overall implementation plan

## Future Considerations

**Potential Additions** (post-MVP):
- `GET /analytics/top-queries` - Most common user queries
- `GET /analytics/feedback-summary` - Feedback statistics
- `POST /embeddings/update` - Trigger re-embedding of modified content
- WebSocket endpoint for streaming responses (for long-form answers)
- Multi-language support (query translation)

**Security Enhancements**:
- OAuth 2.0 / JWT authentication for identified users
- Rate limiting by API key (currently by session_id)
- Input sanitization and injection prevention
- CORS configuration for production deployment
