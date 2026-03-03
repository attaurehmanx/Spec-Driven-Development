# Chat API Contract

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Complete

## Overview

This document defines the contract between the frontend chat interface and the backend chat endpoint. The backend endpoint is already implemented (spec 006-chat-endpoint).

---

## Endpoint Details

### Base Information

**Endpoint**: `POST /api/{user_id}/chat`
**Backend Implementation**: `backend/api/chat.py`
**Authentication**: Required (JWT Bearer token)
**Content-Type**: `application/json`

---

## Request Contract

### URL Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `user_id` | string (UUID) | Yes | Authenticated user's ID (must match JWT claims) |

### Headers

| Header | Value | Required | Description |
|--------|-------|----------|-------------|
| `Authorization` | `Bearer <jwt_token>` | Yes | JWT access token from Better Auth |
| `Content-Type` | `application/json` | Yes | Request body format |

### Request Body

```typescript
{
  message: string;           // User's message text (1-10,000 characters)
  conversation_id?: number;  // Optional conversation ID (null for new conversation)
}
```

**Field Constraints**:
- `message`:
  - **Required**: Yes
  - **Type**: string
  - **Min Length**: 1 character (after trimming whitespace)
  - **Max Length**: 10,000 characters
  - **Validation**: Cannot be empty or whitespace-only
- `conversation_id`:
  - **Required**: No
  - **Type**: number | null | undefined
  - **Default**: null (creates new conversation)
  - **Validation**: Must be a valid conversation ID owned by the authenticated user

### Example Requests

**New Conversation**:
```json
POST /api/550e8400-e29b-41d4-a716-446655440000/chat
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "message": "What tasks do I have today?",
  "conversation_id": null
}
```

**Continue Existing Conversation**:
```json
POST /api/550e8400-e29b-41d4-a716-446655440000/chat
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "message": "Mark the first one as complete",
  "conversation_id": 123
}
```

---

## Response Contract

### Success Response (200 OK)

```typescript
{
  conversation_id: number;   // Conversation ID (new or existing)
  response: string;          // AI assistant's response text
  tool_calls: string[];      // List of MCP tools called by AI
}
```

**Field Descriptions**:
- `conversation_id`:
  - **Type**: number
  - **Description**: Unique conversation identifier
  - **Behavior**:
    - For new conversations: newly created ID from database
    - For existing conversations: same ID as request
- `response`:
  - **Type**: string
  - **Description**: AI assistant's natural language response
  - **Max Length**: No explicit limit (typically < 2000 characters)
- `tool_calls`:
  - **Type**: string[]
  - **Description**: List of MCP tool names called during processing
  - **Possible Values**:
    - `"add_task"` - Task was created
    - `"update_task"` - Task was modified
    - `"delete_task"` - Task was deleted
    - `"complete_task"` - Task completion status changed
    - `"list_tasks"` - Tasks were queried (read-only, no refresh needed)
  - **Empty Array**: No tools were called (conversational response only)

**Example Success Response**:
```json
{
  "conversation_id": 123,
  "response": "I've created a task titled 'Buy milk' for you. It's been added to your task list.",
  "tool_calls": ["add_task"]
}
```

---

## Error Responses

### 400 Bad Request

**Cause**: Invalid request body or validation failure

**Response Body**:
```json
{
  "detail": "Message cannot be empty"
}
```

**Common Scenarios**:
- Empty message (after trimming)
- Message exceeds 10,000 characters
- Invalid JSON in request body
- Missing required `message` field

**Frontend Handling**:
```typescript
if (response.status === 400) {
  setError('Invalid message. Please check your input.');
}
```

---

### 401 Unauthorized

**Cause**: Missing, invalid, or expired JWT token

**Response Body**:
```json
{
  "detail": "Not authenticated"
}
```

**Common Scenarios**:
- No `Authorization` header
- Invalid JWT token format
- Expired JWT token
- JWT signature verification failed

**Frontend Handling**:
```typescript
if (response.status === 401) {
  // API client automatically handles this:
  // 1. Attempts token refresh
  // 2. Retries request
  // 3. If refresh fails, redirects to sign-in
  // 4. Dispatches 'auth-token-invalidated' event
}
```

---

### 403 Forbidden

**Cause**: User ID in URL doesn't match authenticated user in JWT

**Response Body**:
```json
{
  "detail": "User ID mismatch"
}
```

**Common Scenarios**:
- URL `user_id` doesn't match JWT claims
- User attempting to access another user's conversation

**Frontend Handling**:
```typescript
if (response.status === 403) {
  setError('Access denied. Please sign in again.');
  // Redirect to sign-in
}
```

---

### 404 Not Found

**Cause**: Conversation ID not found or doesn't belong to user

**Response Body**:
```json
{
  "detail": "Conversation not found"
}
```

**Common Scenarios**:
- Invalid `conversation_id` in request
- Conversation was deleted
- Conversation belongs to different user

**Frontend Handling**:
```typescript
if (response.status === 404) {
  // Start new conversation
  setConversationId(null);
  setError('Conversation not found. Starting new conversation.');
}
```

---

### 422 Unprocessable Entity

**Cause**: Request validation failed (Pydantic validation error)

**Response Body**:
```json
{
  "detail": [
    {
      "loc": ["body", "message"],
      "msg": "field required",
      "type": "value_error.missing"
    }
  ]
}
```

**Common Scenarios**:
- Type mismatch (e.g., `conversation_id` is string instead of number)
- Missing required fields
- Invalid field values

**Frontend Handling**:
```typescript
if (response.status === 422) {
  setError('Invalid request format. Please try again.');
}
```

---

### 500 Internal Server Error

**Cause**: Backend error (database, agent service, etc.)

**Response Body**:
```json
{
  "detail": "Internal server error"
}
```

**Common Scenarios**:
- Database connection failure
- Agent service error
- Unexpected exception in backend

**Frontend Handling**:
```typescript
if (response.status === 500) {
  setError('Server error. Please try again later.');
}
```

---

### 503 Service Unavailable

**Cause**: Agent service is unavailable or timed out

**Response Body**:
```json
{
  "detail": "Agent service unavailable"
}
```

**Common Scenarios**:
- AI agent service is down
- Agent processing timeout
- Gemini API rate limit exceeded

**Frontend Handling**:
```typescript
if (response.status === 503) {
  setError('AI assistant is temporarily unavailable. Please try again in a moment.');
}
```

---

## Frontend Implementation

### API Client Method

**Location**: `frontend-app/services/api-client.js`

**Method Signature**:
```typescript
async postChat(userId: string, request: ChatRequest): Promise<ChatResponse>
```

**Implementation**:
```typescript
class ApiClient {
  // ... existing methods ...

  /**
   * Send a chat message to the AI assistant
   * @param userId - Authenticated user's ID
   * @param request - Chat request with message and optional conversation_id
   * @returns Promise resolving to chat response
   * @throws Error if request fails
   */
  async postChat(userId: string, request: ChatRequest): Promise<ChatResponse> {
    return this.post(`/api/${userId}/chat`, request);
  }
}
```

**Usage in useChat Hook**:
```typescript
const sendMessage = async (content: string) => {
  try {
    setIsLoading(true);
    setError(null);

    // Add user message optimistically
    const userMessage: ChatMessage = {
      id: crypto.randomUUID(),
      role: 'user',
      content,
      timestamp: new Date(),
      status: 'sending'
    };
    setMessages(prev => [...prev, userMessage]);

    // Send to backend
    const response = await apiClient.postChat(user.id, {
      message: content,
      conversation_id: conversationId
    });

    // Update conversation ID if new
    if (conversationId === null) {
      setConversationId(response.conversation_id);
    }

    // Update user message status
    setMessages(prev =>
      prev.map(msg =>
        msg.id === userMessage.id
          ? { ...msg, status: 'sent' }
          : msg
      )
    );

    // Add AI response
    const aiMessage: ChatMessage = {
      id: crypto.randomUUID(),
      role: 'assistant',
      content: response.response,
      timestamp: new Date()
    };
    setMessages(prev => [...prev, aiMessage]);

    // Trigger task refresh if needed
    if (response.tool_calls.some(tool =>
      ['add_task', 'update_task', 'delete_task', 'complete_task'].includes(tool)
    )) {
      window.dispatchEvent(new CustomEvent('tasks-updated'));
    }

  } catch (error) {
    // Handle error
    setError(getErrorMessage(error));

    // Update user message status to error
    setMessages(prev =>
      prev.map(msg =>
        msg.id === userMessage.id
          ? { ...msg, status: 'error', errorMessage: getErrorMessage(error) }
          : msg
      )
    );
  } finally {
    setIsLoading(false);
  }
};
```

---

## Testing

### Manual Testing Checklist

- [ ] **New Conversation**: Send first message with `conversation_id: null`, verify response includes new ID
- [ ] **Continue Conversation**: Send follow-up message with existing ID, verify context is maintained
- [ ] **Empty Message**: Attempt to send empty message, verify 400 error
- [ ] **Long Message**: Send message > 10,000 characters, verify 400 error
- [ ] **Expired Token**: Use expired JWT, verify 401 error and automatic refresh
- [ ] **Invalid Conversation ID**: Use non-existent conversation ID, verify 404 error
- [ ] **Task Creation**: Send "Create a task", verify `tool_calls` includes `"add_task"`
- [ ] **Task Update**: Send "Mark task as complete", verify `tool_calls` includes `"complete_task"`
- [ ] **Read-Only Query**: Send "What tasks do I have?", verify `tool_calls` includes `"list_tasks"` only

### Example Test Requests (curl)

```bash
# New conversation
curl -X POST http://localhost:8000/api/550e8400-e29b-41d4-a716-446655440000/chat \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"message": "What tasks do I have?", "conversation_id": null}'

# Continue conversation
curl -X POST http://localhost:8000/api/550e8400-e29b-41d4-a716-446655440000/chat \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"message": "Mark the first one as complete", "conversation_id": 123}'
```

---

## Contract Guarantees

### Backend Guarantees
- ✅ Endpoint is stateless (no in-memory conversation state)
- ✅ Conversation history loaded from database on each request
- ✅ User messages persisted before agent processing
- ✅ AI responses persisted after agent processing
- ✅ User data isolation enforced (user can only access own conversations)
- ✅ JWT validation on every request
- ✅ Consistent error response format

### Frontend Responsibilities
- ✅ Include valid JWT token in Authorization header
- ✅ Validate message content before sending (1-10,000 characters)
- ✅ Handle all error responses gracefully
- ✅ Display user-friendly error messages
- ✅ Manage conversation_id state correctly
- ✅ Trigger task refresh when tool_calls indicate data modifications

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-30 | Initial contract definition based on spec 006 |

---

## Related Documents

- Backend Implementation: `specs/006-chat-endpoint/spec.md`
- Backend API Code: `backend/api/chat.py`
- Backend Models: `backend/models/chat_models.py`
- Frontend Data Models: `specs/007-frontend-chat-interface/data-model.md`
