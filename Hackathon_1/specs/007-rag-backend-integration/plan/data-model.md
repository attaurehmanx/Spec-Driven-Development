# Data Model: Frontend ↔ Backend Integration

**Feature**: RAG Pipeline: Frontend and FastAPI Backend Integration
**Created**: 2025-12-15
**Status**: Draft

## Entity Definitions

### QueryRequest
**Description**: Represents a user's query and optional selected text from the frontend

**Fields**:
- `query` (string, required): The user's question or query text
- `selected_text` (string, optional): Text selected by the user on the page (default: "")
- `session_id` (string, optional): Unique identifier for the conversation session
- `metadata` (object, optional): Additional context information

**Validation Rules**:
- `query` must be 1-2000 characters
- `selected_text` must be 0-5000 characters if provided
- `session_id` must be a valid UUID format if provided

### QueryResponse
**Description**: Contains the RAG-generated answer with source citations and metadata

**Fields**:
- `answer` (string, required): The AI-generated response to the query
- `citations` (array of Citation, required): List of source documents referenced
- `query_id` (string, required): Unique identifier for this query
- `timestamp` (string, required): ISO 8601 timestamp of response generation
- `metadata` (object, optional): Additional response metadata

**Validation Rules**:
- `answer` must be 1-10000 characters
- `citations` must contain 0-20 items
- `query_id` must be a valid UUID

### Citation
**Description**: Points to source documents in Qdrant that support the response

**Fields**:
- `document_id` (string, required): Unique identifier of the source document
- `title` (string, required): Title of the source document
- `url` (string, required): URL to access the source document
- `text_snippet` (string, required): Relevant text snippet from the source
- `relevance_score` (number, required): Score indicating relevance (0.0-1.0)

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `url` must be a valid URL format
- `text_snippet` must be 10-1000 characters

### ErrorResponse
**Description**: Standard error response format for API failures

**Fields**:
- `error_code` (string, required): Machine-readable error code
- `message` (string, required): Human-readable error description
- `details` (object, optional): Additional error details
- `timestamp` (string, required): ISO 8601 timestamp of error

**Validation Rules**:
- `error_code` must follow format ERROR_XXXX
- `message` must be 1-500 characters

## State Transitions

### Query Processing Flow
1. **Query Received**: QueryRequest is validated and accepted
2. **Context Retrieval**: Relevant documents retrieved from Qdrant
3. **Response Generation**: RAG agent generates answer with citations
4. **Response Ready**: QueryResponse is prepared and returned

## Relationships
- One QueryRequest generates one QueryResponse
- One QueryResponse contains multiple Citation objects
- Citation objects reference external documents in Qdrant

## Constraints
- All string fields are UTF-8 encoded
- Timestamps follow ISO 8601 format (YYYY-MM-DDTHH:MM:SS.sssZ)
- All IDs use UUID v4 format
- Arrays have maximum length limits to prevent abuse