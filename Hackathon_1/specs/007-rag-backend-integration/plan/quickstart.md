# Quickstart Guide: Frontend ↔ Backend Integration

**Feature**: RAG Pipeline: Frontend and FastAPI Backend Integration
**Created**: 2025-12-15
**Status**: Draft

## Overview

This guide provides a quick setup for developers to understand and implement the RAG query API integration between the Docusaurus frontend and FastAPI backend.

## Prerequisites

- Python 3.10+ installed
- FastAPI and related dependencies
- Qdrant vector database running
- Anthropic API access for the RAG agent
- Docusaurus documentation site

## Development Environment Setup

### Backend Setup

1. **Install dependencies**:
   ```bash
   pip install fastapi uvicorn pydantic python-multipart
   pip install qdrant-client
   pip install anthropic
   ```

2. **Configure environment variables**:
   ```bash
   export ANTHROPIC_API_KEY="your-api-key"
   export QDRANT_HOST="localhost"
   export QDRANT_PORT=6333
   export API_KEY="your-secure-api-key"
   ```

3. **Run the backend server**:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

### Frontend Setup

1. **Install Docusaurus dependencies**:
   ```bash
   npm install
   ```

2. **Configure API endpoint in frontend**:
   Update the API endpoint in your Docusaurus configuration to point to your backend server.

## API Usage Examples

### Making a Query Request

```javascript
const response = await fetch('http://localhost:8000/v1/query', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'X-API-Key': 'your-api-key'
  },
  body: JSON.stringify({
    query: 'How do I configure ROS 2 parameters?',
    selected_text: '',
    session_id: '550e8400-e29b-41d4-a716-446655440000'
  })
});

const data = await response.json();
console.log(data.answer);
console.log(data.citations);
```

### Response Format

The API returns responses in the following format:

```json
{
  "answer": "Response text from the RAG agent...",
  "citations": [
    {
      "document_id": "doc-id-123",
      "title": "Document Title",
      "url": "/path/to/document",
      "text_snippet": "Relevant text snippet...",
      "relevance_score": 0.95
    }
  ],
  "query_id": "query-12345",
  "timestamp": "2025-12-15T10:30:00.000Z",
  "metadata": {
    "processing_time_ms": 1250
  }
}
```

## Testing the Integration

### Unit Tests

```bash
# Run backend unit tests
python -m pytest tests/unit/

# Run integration tests
python -m pytest tests/integration/
```

### End-to-End Test

1. Start the backend server
2. Navigate to your Docusaurus site
3. Use the embedded chatbot to submit a query
4. Verify that you receive a response with citations

## Error Handling

Common error responses:

- **400 Bad Request**: Invalid query format or missing required fields
- **401 Unauthorized**: Missing or invalid API key
- **429 Rate Limit**: Too many requests from the same API key
- **500 Internal Server Error**: Backend processing error

## Security Considerations

- Always use HTTPS in production
- Store API keys securely (never commit to version control)
- Implement rate limiting per API key
- Validate all user inputs
- Log security-relevant events

## Performance Tips

- Enable connection pooling for Qdrant client
- Implement caching for frequently asked questions
- Use async/await for non-blocking operations
- Monitor response times and optimize as needed