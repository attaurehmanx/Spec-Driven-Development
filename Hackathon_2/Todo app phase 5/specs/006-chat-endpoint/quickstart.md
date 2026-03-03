# FastAPI Chat Endpoint - Quick Start Guide

**Feature**: 006-chat-endpoint
**Status**: MVP Implementation Complete - Ready for Testing

## What Was Implemented

The chat endpoint allows authenticated users to:
- Send messages to an AI agent
- Receive intelligent responses with tool execution
- Maintain conversation history across multiple turns
- Create new conversations or continue existing ones

## Quick Test (5 Minutes)

### Step 1: Start the FastAPI Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

Server should start at: http://localhost:8000

### Step 2: Get a JWT Token

**Option A: Use existing account**
```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email": "your-email@example.com", "password": "your-password"}'
```

**Option B: Create new account**
```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "Test123!", "name": "Test User"}'
```

**Save these from the response:**
- `token` - Your JWT token
- `user.id` - Your user ID

### Step 3: Test the Chat Endpoint

Replace `{USER_ID}` and `{TOKEN}` with your values:

```bash
curl -X POST http://localhost:8000/api/{USER_ID}/chat \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "Show me my tasks"}'
```

**Expected Response:**
```json
{
  "conversation_id": 1,
  "response": "You currently have X tasks...",
  "tool_calls": ["list_tasks"]
}
```

### Step 4: Continue the Conversation

Use the `conversation_id` from the previous response:

```bash
curl -X POST http://localhost:8000/api/{USER_ID}/chat \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "Create a task to buy milk", "conversation_id": 1}'
```

## Run Automated Test Scripts

### Update Test Configuration

Edit both test scripts:
- `backend/test_chat_auth.py`
- `backend/test_chat_new_conversation.py`

Set these variables:
```python
USER_ID = "your-user-id-here"
JWT_TOKEN = "your-jwt-token-here"
```

### Run Tests

```bash
# Test authentication scenarios
python backend/test_chat_auth.py

# Test new conversation creation
python backend/test_chat_new_conversation.py
```

## API Documentation

Once the server is running, visit:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

Look for the "chat" tag to see the endpoint documentation.

## Endpoint Details

### POST /api/{user_id}/chat

**Authentication**: Required (Bearer token)

**Request Body:**
```json
{
  "message": "Your message here (1-10,000 characters)",
  "conversation_id": 123  // Optional - omit to create new conversation
}
```

**Success Response (200):**
```json
{
  "conversation_id": 123,
  "response": "AI agent's response",
  "tool_calls": ["tool1", "tool2"]
}
```

**Error Responses:**
- `400` - Invalid input (empty message, too long)
- `401` - Missing or invalid JWT token
- `403` - User ID mismatch (trying to access another user's data)
- `404` - Conversation not found
- `500` - Internal server error
- `503` - AI agent service unavailable

## Troubleshooting

### "Could not validate credentials" (401)
- Check that your JWT token is valid and not expired
- Verify the token is in the format: `Bearer {token}`
- Try signing in again to get a fresh token

### "User ID mismatch" (403)
- Ensure the `{user_id}` in the URL matches the user ID in your JWT token
- Check the token payload to verify your user ID

### "AI service unavailable" (503)
- Verify the agent service is configured correctly
- Check that `GEMINI_API_KEY` is set in your environment
- Review server logs for agent service errors

### "Conversation not found" (404)
- Verify the conversation_id exists and belongs to you
- Try creating a new conversation by omitting conversation_id

### Database Errors (500)
- Verify database connection is working
- Check that all tables are created (run migrations)
- Review server logs for specific database errors

## Verify Database Persistence

Connect to your database and check:

```sql
-- View your conversations
SELECT * FROM conversation WHERE user_id = '{your_user_id}';

-- View messages in a conversation
SELECT * FROM message WHERE conversation_id = {conversation_id} ORDER BY created_at;
```

You should see:
- User messages with `role = 'user'`
- AI responses with `role = 'assistant'`
- Messages ordered chronologically

## Example Conversation Flow

```bash
# 1. Create new conversation
curl -X POST http://localhost:8000/api/{USER_ID}/chat \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "What tasks do I have?"}'

# Response: {"conversation_id": 1, "response": "...", "tool_calls": ["list_tasks"]}

# 2. Continue conversation
curl -X POST http://localhost:8000/api/{USER_ID}/chat \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "Create a task to buy groceries", "conversation_id": 1}'

# Response: {"conversation_id": 1, "response": "...", "tool_calls": ["add_task"]}

# 3. Follow up
curl -X POST http://localhost:8000/api/{USER_ID}/chat \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"message": "Mark it as complete", "conversation_id": 1}'

# Response: {"conversation_id": 1, "response": "...", "tool_calls": ["complete_task"]}
```

## Manual Testing Status

**Status**: READY FOR TESTING (Scripts Created, Backend Blocked)

### Test Scripts Created

All manual testing scripts (T031-T038) have been created and are ready to execute:

- **Comprehensive Test Suite**: `backend/run_all_chat_tests.py` (RECOMMENDED)
  - Automatically tests all scenarios T031-T038
  - Auto-creates test user and authenticates
  - Colored output with detailed reporting
  - No manual configuration needed

- **Individual Test Scripts**:
  - `backend/test_chat_auth.py` - Authentication tests
  - `backend/test_chat_new_conversation.py` - Conversation tests

### Current Blocker

The correct FastAPI backend is not running on port 8000. A different service ("Kiro API Gateway") is currently running on that port.

### How to Run Tests

1. **Stop the current service on port 8000**
2. **Start the FastAPI backend**:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```
3. **Run the comprehensive test suite**:
   ```bash
   cd backend
   python run_all_chat_tests.py
   ```

See `backend/MANUAL_TESTING_FINAL_REPORT.md` for detailed instructions.

## What's Next?

After running tests and verifying they pass:

1. **Mark verification tasks complete** (T031-T038 in tasks.md)
2. **Update this file** with actual curl examples and responses
3. **Optional: Implement Phase 4** - Enhanced multi-turn context
4. **Optional: Implement Phase 5** - Production-ready error handling
5. **Optional: Implement Phase 6** - Documentation and automated tests

## Files Reference

**Implementation:**
- `backend/models/chat_models.py` - Request/response models
- `backend/services/conversation_service.py` - Business logic
- `backend/api/chat.py` - Chat endpoint router
- `backend/main.py` - Router mounting

**Testing:**
- `backend/test_chat_auth.py` - Authentication tests
- `backend/test_chat_new_conversation.py` - Conversation tests

**Documentation:**
- `specs/006-chat-endpoint/spec.md` - Feature specification
- `specs/006-chat-endpoint/plan.md` - Technical architecture
- `specs/006-chat-endpoint/tasks.md` - Task breakdown
- `specs/006-chat-endpoint/MVP_IMPLEMENTATION_SUMMARY.md` - Implementation details

## Support

If you encounter issues:
1. Check the FastAPI server logs for detailed error messages
2. Verify all environment variables are set (DATABASE_URL, GEMINI_API_KEY, etc.)
3. Ensure database tables exist (run migrations if needed)
4. Test authentication separately to isolate issues
5. Review the implementation summary for architecture details

---

**Ready to test!** Start with Step 1 above and work through the quick test.
