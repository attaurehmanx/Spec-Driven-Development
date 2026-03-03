# Quickstart Guide: Frontend Chat Interface

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30
**Status**: Complete

## Overview

This guide helps developers set up and test the Frontend Chat Interface feature. Follow these steps to get the chat interface running in your local development environment.

---

## Prerequisites

### Required Services

1. **Backend API Server**
   - Location: `Z:\phse 33\backend`
   - Must be running on `http://localhost:8000`
   - Chat endpoint must be available: `POST /api/{user_id}/chat`
   - Database must be configured and migrated

2. **Frontend Development Server**
   - Location: `Z:\phse 33\frontend-app`
   - Will run on `http://localhost:3000`

3. **User Account**
   - Must have a registered user account via Better Auth
   - Must have valid JWT access token

### Required Tools

- Node.js 18+ and npm
- Python 3.10+ (for backend)
- PostgreSQL database (Neon Serverless)

---

## Environment Setup

### 1. Backend Configuration

**File**: `Z:\phse 33\backend\.env`

```bash
# Database
DATABASE_URL=postgresql://user:password@host/database

# Authentication
BETTER_AUTH_SECRET=your-secret-key-here

# AI Service
GEMINI_API_KEY=your-gemini-api-key-here

# Server
PORT=8000
```

### 2. Frontend Configuration

**File**: `Z:\phse 33\frontend-app\.env.local`

```bash
# Backend API URL
NEXT_PUBLIC_BACKEND_URL=http://localhost:8000

# Better Auth (if needed)
BETTER_AUTH_SECRET=your-secret-key-here
```

---

## Installation

### Backend Setup

```bash
# Navigate to backend directory
cd "Z:\phse 33\backend"

# Create virtual environment (if not exists)
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run database migrations (if needed)
alembic upgrade head

# Start backend server
uvicorn main:app --reload --port 8000
```

**Verify Backend**:
```bash
# Check health endpoint
curl http://localhost:8000/health

# Expected response: {"status": "ok"}
```

### Frontend Setup

```bash
# Navigate to frontend directory
cd "Z:\phse 33\frontend-app"

# Install dependencies
npm install

# Start development server
npm run dev
```

**Verify Frontend**:
- Open browser to `http://localhost:3000`
- Should see the application homepage

---

## User Authentication

### Create Test User

**Option 1: Via Frontend**
1. Navigate to `http://localhost:3000/sign-up`
2. Fill in registration form:
   - Email: `test@example.com`
   - Password: `TestPassword123!`
   - Name: `Test User`
3. Submit form
4. Should redirect to dashboard

**Option 2: Via API (curl)**
```bash
curl -X POST http://localhost:8000/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPassword123!",
    "name": "Test User"
  }'
```

### Get JWT Token

**Via Frontend**:
1. Sign in at `http://localhost:3000/sign-in`
2. Open browser DevTools → Application → Local Storage
3. Find `access_token` key
4. Copy the token value

**Via API**:
```bash
curl -X POST http://localhost:8000/api/auth/sign-in \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPassword123!"
  }'

# Response includes access_token
```

---

## Testing the Chat Interface

### 1. Access the Dashboard

1. Navigate to `http://localhost:3000/dashboard`
2. Should see dashboard with task list
3. Look for chat icon/button (typically in header or sidebar)

### 2. Open Chat Interface

1. Click the chat icon/button
2. Chat interface should open (sidebar or modal)
3. Should see empty chat window with input field

### 3. Basic Chat Test

**Test 1: Simple Query**
```
User: "What tasks do I have?"
Expected: AI responds with list of tasks (or "You have no tasks")
```

**Test 2: Task Creation**
```
User: "Create a task to buy milk"
Expected:
- AI confirms task creation
- Task list automatically refreshes
- New task appears in task list
```

**Test 3: Multi-turn Conversation**
```
User: "Create a task to buy milk"
AI: "I've created a task titled 'Buy milk' for you."

User: "Mark it as complete"
Expected: AI understands "it" refers to the milk task
```

**Test 4: Task Update**
```
User: "Update the milk task description to 'Buy 2% milk'"
Expected:
- AI confirms update
- Task list refreshes
- Task description updated
```

**Test 5: Task Deletion**
```
User: "Delete the milk task"
Expected:
- AI confirms deletion
- Task list refreshes
- Task removed from list
```

### 4. Error Handling Test

**Test 1: Empty Message**
```
Action: Try to send empty message
Expected: Input validation prevents sending
```

**Test 2: Network Error**
```
Action: Stop backend server, send message
Expected: Error message displayed in chat
```

**Test 3: Expired Token**
```
Action: Wait for token to expire, send message
Expected: Automatic token refresh or redirect to sign-in
```

---

## Troubleshooting

### Chat Interface Not Loading

**Symptoms**: Chat button doesn't appear or clicking it does nothing

**Solutions**:
1. Check browser console for errors
2. Verify backend is running: `curl http://localhost:8000/health`
3. Check JWT token is valid in localStorage
4. Clear browser cache and reload

### Messages Not Sending

**Symptoms**: Clicking send does nothing or shows error

**Solutions**:
1. Open browser DevTools → Network tab
2. Send a message and check for API request
3. Look for error response (401, 403, 500, etc.)
4. Verify JWT token in Authorization header
5. Check backend logs for errors

### Task List Not Refreshing

**Symptoms**: Chat confirms task creation but list doesn't update

**Solutions**:
1. Open browser console
2. Send task-creating message
3. Look for `tasks-updated` event dispatch
4. Check if task list component is listening for event
5. Manually refresh page to verify task was created

### 401 Unauthorized Error

**Symptoms**: All chat requests return 401

**Solutions**:
1. Check JWT token in localStorage
2. Verify token is not expired
3. Try signing out and signing in again
4. Check BETTER_AUTH_SECRET matches between frontend and backend

### 403 Forbidden Error

**Symptoms**: Chat requests return 403

**Solutions**:
1. Verify user_id in URL matches authenticated user
2. Check JWT token claims
3. Ensure backend validates user_id correctly

### AI Not Responding

**Symptoms**: Message sent but no AI response

**Solutions**:
1. Check backend logs for agent service errors
2. Verify GEMINI_API_KEY is set correctly
3. Check Gemini API quota/rate limits
4. Test backend chat endpoint directly with curl

---

## Manual API Testing

### Test Chat Endpoint Directly

**Get User ID**:
```bash
# Decode JWT token to get user_id
# Use https://jwt.io or jwt-decode library
```

**Send Chat Message**:
```bash
curl -X POST http://localhost:8000/api/YOUR_USER_ID/chat \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What tasks do I have?",
    "conversation_id": null
  }'
```

**Expected Response**:
```json
{
  "conversation_id": 1,
  "response": "You currently have 3 tasks: ...",
  "tool_calls": ["list_tasks"]
}
```

**Continue Conversation**:
```bash
curl -X POST http://localhost:8000/api/YOUR_USER_ID/chat \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Create a task to buy milk",
    "conversation_id": 1
  }'
```

**Expected Response**:
```json
{
  "conversation_id": 1,
  "response": "I've created a task titled 'Buy milk' for you.",
  "tool_calls": ["add_task"]
}
```

---

## Development Workflow

### Making Changes

1. **Modify Chat Components**:
   - Edit files in `frontend-app/components/chat/`
   - Changes hot-reload automatically
   - Check browser console for errors

2. **Update State Management**:
   - Edit `frontend-app/hooks/use-chat.ts`
   - Test with React DevTools

3. **Modify API Integration**:
   - Edit `frontend-app/services/api-client.js`
   - Test with Network tab in DevTools

4. **Update Styling**:
   - Edit Tailwind classes in components
   - Changes apply immediately

### Testing Changes

1. **Browser DevTools**:
   - Console: Check for errors and logs
   - Network: Inspect API requests/responses
   - Application: Check localStorage for tokens
   - React DevTools: Inspect component state

2. **Manual Testing**:
   - Test all user scenarios from spec
   - Test on different screen sizes (responsive)
   - Test error scenarios

3. **Code Quality**:
   ```bash
   # Type check
   npm run type-check

   # Lint
   npm run lint

   # Format
   npm run format
   ```

---

## Common Development Tasks

### Add New Chat Component

```bash
# Create new component file
touch frontend-app/components/chat/my-component.tsx

# Import in chat-interface.tsx
import { MyComponent } from './my-component';
```

### Add New Message Type

```typescript
// Update types.ts
interface ChatMessage {
  // ... existing fields ...
  customField?: string;  // Add new field
}

// Update components to handle new field
```

### Modify Event Handling

```typescript
// In chat-interface.tsx
if (response.tool_calls.includes('my_custom_tool')) {
  window.dispatchEvent(new CustomEvent('custom-event'));
}

// In listener component
useEffect(() => {
  const handler = () => { /* handle event */ };
  window.addEventListener('custom-event', handler);
  return () => window.removeEventListener('custom-event', handler);
}, []);
```

---

## Performance Monitoring

### Check Chat Performance

1. **Message Send Time**:
   - Open Network tab
   - Send message
   - Check request duration (should be < 5 seconds)

2. **Task Refresh Time**:
   - Send task-creating message
   - Measure time until task list updates (should be < 1 second)

3. **Chat Load Time**:
   - Open Performance tab
   - Record while opening chat
   - Check load time (should be < 2 seconds)

### Optimize if Needed

- Lazy load chat component: `const Chat = lazy(() => import('./chat-interface'))`
- Memoize message components: `React.memo(ChatMessage)`
- Debounce task refresh: `debounce(fetchTasks, 500)`
- Virtualize long message lists: Use `react-window` or `react-virtualized`

---

## Debugging Tips

### Enable Debug Logging

```typescript
// In use-chat.ts
const DEBUG = true;

const sendMessage = async (content: string) => {
  if (DEBUG) console.log('Sending message:', content);

  try {
    const response = await apiClient.postChat(user.id, {
      message: content,
      conversation_id: conversationId
    });

    if (DEBUG) console.log('Received response:', response);

    // ... rest of logic ...
  } catch (error) {
    if (DEBUG) console.error('Send message error:', error);
  }
};
```

### Inspect Event Dispatch

```typescript
// In chat-interface.tsx
if (hasTaskModification) {
  console.log('Dispatching tasks-updated event');
  window.dispatchEvent(new CustomEvent('tasks-updated'));
}

// In task list
useEffect(() => {
  const handler = () => {
    console.log('tasks-updated event received');
    fetchTasks();
  };
  window.addEventListener('tasks-updated', handler);
  return () => window.removeEventListener('tasks-updated', handler);
}, [fetchTasks]);
```

### Check State Updates

```typescript
// Use React DevTools
// Or add console logs
useEffect(() => {
  console.log('Messages updated:', messages);
}, [messages]);

useEffect(() => {
  console.log('Conversation ID updated:', conversationId);
}, [conversationId]);
```

---

## Next Steps

After completing local testing:

1. **Run Full Test Suite**: Test all user scenarios from spec
2. **Test on Multiple Browsers**: Chrome, Firefox, Safari, Edge
3. **Test Responsive Design**: Desktop, tablet, mobile (320px-1920px)
4. **Performance Testing**: Test with 100+ message conversations
5. **Error Scenario Testing**: Network failures, expired tokens, backend errors
6. **Accessibility Testing**: Keyboard navigation, screen readers
7. **Code Review**: Review all changes before committing
8. **Documentation**: Update any relevant documentation

---

## Useful Commands

```bash
# Frontend
npm run dev          # Start dev server
npm run build        # Production build
npm run type-check   # TypeScript check
npm run lint         # ESLint
npm run format       # Prettier

# Backend
uvicorn main:app --reload              # Start with hot reload
uvicorn main:app --reload --log-level debug  # Debug mode
pytest tests/                          # Run tests
alembic upgrade head                   # Run migrations

# Database
psql $DATABASE_URL                     # Connect to database
psql $DATABASE_URL -c "SELECT * FROM conversations;"  # Query conversations
psql $DATABASE_URL -c "SELECT * FROM messages;"       # Query messages
```

---

## Support

### Getting Help

- **Documentation**: Check `specs/007-frontend-chat-interface/` for detailed specs
- **Backend Docs**: Check `specs/006-chat-endpoint/` for backend API details
- **Code Examples**: Look at existing components in `frontend-app/components/`
- **Type Definitions**: Check `frontend-app/types.ts` for all interfaces

### Reporting Issues

When reporting issues, include:
1. Steps to reproduce
2. Expected behavior
3. Actual behavior
4. Browser console errors
5. Network tab screenshots
6. Backend logs (if relevant)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-30 | Initial quickstart guide |

---

## Related Documents

- Feature Specification: `specs/007-frontend-chat-interface/spec.md`
- Implementation Plan: `specs/007-frontend-chat-interface/plan.md`
- Research Findings: `specs/007-frontend-chat-interface/research.md`
- Data Models: `specs/007-frontend-chat-interface/data-model.md`
- API Contract: `specs/007-frontend-chat-interface/contracts/chat-api.md`
- Refresh Contract: `specs/007-frontend-chat-interface/contracts/task-refresh.md`
