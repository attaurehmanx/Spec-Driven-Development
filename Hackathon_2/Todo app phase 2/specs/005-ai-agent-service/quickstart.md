# AI Agent Service Quickstart

**Feature**: 005-ai-agent-service
**Date**: 2026-01-29
**Purpose**: Quick setup and testing guide for developers

## Prerequisites

- Python 3.10+
- Existing backend with MCP tools implemented (Spec 4)
- Gemini API key from [Google AI Studio](https://aistudio.google.com/apikey)
- Backend running with database configured

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install openai
```

**Note**: `google-generativeai` is NOT needed when using OpenAI SDK with Gemini endpoint.

### 2. Configure Environment

Add to `backend/.env`:

```bash
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_MODEL=gemini-2.5-flash
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Agent Configuration
AGENT_MAX_ITERATIONS=15
AGENT_TEMPERATURE=1.0
AGENT_MAX_TOKENS=1000
AGENT_TIMEOUT=30
```

**Get API Key**:
1. Visit [Google AI Studio](https://aistudio.google.com/apikey)
2. Sign in with Google account
3. Click "Get API key"
4. Copy key to `.env` file

### 3. Verify MCP Tools

Ensure MCP tools are accessible:

```bash
python -c "from mcp_server.tools.task_tools import add_task, list_tasks, complete_task, update_task, delete_task; print('✓ MCP tools loaded successfully')"
```

Expected output:
```
✓ MCP tools loaded successfully
```

If this fails, ensure Spec 4 (MCP Task Tools) is implemented.

### 4. Verify Database Connection

```bash
python -c "from database.session import get_async_session; print('✓ Database connection configured')"
```

## Testing

### Manual Test: Basic Agent Conversation

Create `backend/test_agent_manual.py`:

```python
import asyncio
import os
from services.agent_service import run_agent

async def test_basic_conversation():
    """Test basic agent conversation with task creation."""

    # Test user ID (ensure this user exists in database)
    user_id = "550e8400-e29b-41d4-a716-446655440000"

    # Simple message history
    message_history = [
        {"role": "user", "content": "Create a task to buy groceries"}
    ]

    print("Testing agent service...")
    print(f"User ID: {user_id}")
    print(f"Message: {message_history[0]['content']}")
    print("-" * 60)

    # Run agent
    response = await run_agent(
        message_history=message_history,
        user_id=user_id
    )

    # Display results
    print(f"Status: {response.status}")
    print(f"Iterations: {response.iterations}")
    print(f"Response: {response.final_response}")
    print("-" * 60)

    # Verify success
    if response.status == "completed":
        print("✓ Test PASSED: Agent completed successfully")
        return True
    else:
        print(f"✗ Test FAILED: {response.error}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_basic_conversation())
    exit(0 if success else 1)
```

Run test:
```bash
python test_agent_manual.py
```

Expected output:
```
Testing agent service...
User ID: 550e8400-e29b-41d4-a716-446655440000
Message: Create a task to buy groceries
------------------------------------------------------------
Status: completed
Iterations: 2
Response: I've created a task titled "Buy groceries" for you. Your task has been added to your list.
------------------------------------------------------------
✓ Test PASSED: Agent completed successfully
```

### Manual Test: Multi-Turn Conversation

Create `backend/test_agent_multiturn.py`:

```python
import asyncio
from services.agent_service import run_agent

async def test_multiturn_conversation():
    """Test multi-turn conversation with context."""

    user_id = "550e8400-e29b-41d4-a716-446655440000"

    # Simulate conversation history
    message_history = [
        {"role": "user", "content": "Show me my tasks"},
        {"role": "assistant", "content": "You have 2 tasks: 1. Buy groceries (pending), 2. Call mom (pending)"},
        {"role": "user", "content": "Mark the first one as complete"}
    ]

    print("Testing multi-turn conversation...")
    print("Conversation history:")
    for msg in message_history:
        print(f"  {msg['role']}: {msg['content']}")
    print("-" * 60)

    response = await run_agent(
        message_history=message_history,
        user_id=user_id
    )

    print(f"Status: {response.status}")
    print(f"Iterations: {response.iterations}")
    print(f"Response: {response.final_response}")
    print("-" * 60)

    if response.status == "completed" and "complete" in response.final_response.lower():
        print("✓ Test PASSED: Agent understood context and completed task")
        return True
    else:
        print("✗ Test FAILED: Agent did not handle context correctly")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_multiturn_conversation())
    exit(0 if success else 1)
```

Run test:
```bash
python test_agent_multiturn.py
```

### Automated Tests

Run pytest suite:

```bash
pytest tests/test_agent_service.py -v
```

Expected output:
```
tests/test_agent_service.py::test_agent_basic_conversation PASSED
tests/test_agent_service.py::test_agent_tool_execution PASSED
tests/test_agent_service.py::test_agent_error_handling PASSED
tests/test_agent_service.py::test_agent_max_iterations PASSED
tests/test_agent_service.py::test_agent_user_isolation PASSED
```

## Usage in Chat API

### Integration Example

```python
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from services.agent_service import run_agent
from database.session import get_async_session

router = APIRouter()

class ChatRequest(BaseModel):
    message: str

class ChatResponse(BaseModel):
    response: str
    iterations: int
    status: str

@router.post("/api/{user_id}/chat", response_model=ChatResponse)
async def chat_endpoint(
    user_id: str,
    request: ChatRequest,
    session = Depends(get_async_session)
):
    """
    Chat endpoint that uses agent service.

    1. Load conversation history from database
    2. Add new user message
    3. Run agent
    4. Persist messages
    5. Return response
    """

    # Load conversation history (simplified)
    message_history = []  # Load from database

    # Add new user message
    message_history.append({
        "role": "user",
        "content": request.message
    })

    # Run agent
    response = await run_agent(
        message_history=message_history,
        user_id=user_id
    )

    # Handle errors
    if response.status == "error":
        raise HTTPException(
            status_code=500,
            detail=response.error or "Agent service error"
        )

    # Persist messages to database (implementation needed)
    # await save_messages(session, user_id, message_history, response.final_response)

    return ChatResponse(
        response=response.final_response,
        iterations=response.iterations,
        status=response.status
    )
```

## Configuration

### Customize Agent Behavior

Edit `backend/config/agent_config.py`:

```python
import os
from pydantic import Field
from pydantic_settings import BaseSettings

class AgentSettings(BaseSettings):
    """Agent service configuration."""

    # Gemini API
    gemini_api_key: str = Field(..., env="GEMINI_API_KEY")
    gemini_model: str = Field(
        default="gemini-2.5-flash",
        env="GEMINI_MODEL"
    )
    gemini_base_url: str = Field(
        default="https://generativelanguage.googleapis.com/v1beta/openai/",
        env="GEMINI_BASE_URL"
    )

    # Agent behavior
    max_iterations: int = Field(default=15, env="AGENT_MAX_ITERATIONS")
    temperature: float = Field(default=1.0, env="AGENT_TEMPERATURE")
    max_tokens: int = Field(default=1000, env="AGENT_MAX_TOKENS")
    timeout: int = Field(default=30, env="AGENT_TIMEOUT")

    class Config:
        env_file = ".env"

# Global settings instance
settings = AgentSettings()
```

### Model Selection

Choose model based on requirements:

| Model | Use Case | Cost | Speed |
|-------|----------|------|-------|
| `gemini-2.5-flash-lite` | High volume, cost-sensitive | Lowest | Fastest |
| `gemini-2.5-flash` | Balanced (recommended) | Medium | Fast |
| `gemini-2.5-pro` | Complex reasoning | Higher | Slower |
| `gemini-3-flash-preview` | Latest features | Medium | Fast |

Update in `.env`:
```bash
GEMINI_MODEL=gemini-2.5-flash-lite  # For cost optimization
```

## Troubleshooting

### Issue: "Invalid API key"

**Error**:
```
LLM API error: Invalid API key
```

**Solution**:
1. Verify `GEMINI_API_KEY` in `.env` file
2. Ensure no extra spaces or quotes
3. Test API key at [Google AI Studio](https://aistudio.google.com/)
4. Check API key has not expired

### Issue: "Tool execution failed: User not found"

**Error**:
```
Tool execution failed: User 550e8400-e29b-41d4-a716-446655440000 not found
```

**Solution**:
1. Ensure user exists in database before testing
2. Create test user:
```python
from models.task_models import User
from database.session import get_async_session

async def create_test_user():
    async for session in get_async_session():
        user = User(
            id="550e8400-e29b-41d4-a716-446655440000",
            email="test@example.com",
            username="testuser"
        )
        session.add(user)
        await session.commit()
```

### Issue: "Max iterations exceeded"

**Error**:
```
Status: max_iterations_reached
Warning: Agent reached maximum iterations before natural completion
```

**Solution**:
1. Check if agent is stuck in loop
2. Review system prompt clarity
3. Increase `AGENT_MAX_ITERATIONS` if needed
4. Simplify user query
5. Check tool responses are properly formatted

### Issue: "Rate limit exceeded"

**Error**:
```
429 RESOURCE_EXHAUSTED: Rate limit exceeded
```

**Solution**:
1. Wait 60 seconds and retry
2. Check rate limits in [Google AI Studio](https://aistudio.google.com/)
3. Upgrade to paid tier for higher limits
4. Implement exponential backoff retry logic

### Issue: "Import error: Cannot import agent_service"

**Error**:
```
ImportError: cannot import name 'run_agent' from 'services.agent_service'
```

**Solution**:
1. Ensure `backend/services/agent_service.py` exists
2. Check Python path includes backend directory
3. Verify all dependencies installed: `pip install -r requirements.txt`

### Issue: "Tool binding failed"

**Error**:
```
ValueError: Tool definition is invalid
```

**Solution**:
1. Verify MCP tools are properly decorated with `@mcp.tool()`
2. Check tool function signatures match expected format
3. Ensure Pydantic models are valid
4. Review `bind_mcp_tools()` implementation

## Performance Optimization

### Reduce Response Time

1. **Use faster model**:
   ```bash
   GEMINI_MODEL=gemini-2.5-flash-lite
   ```

2. **Reduce max_tokens**:
   ```bash
   AGENT_MAX_TOKENS=500  # Shorter responses
   ```

3. **Optimize tool execution**:
   - Ensure database queries use indexes
   - Execute tools in parallel (already implemented)

### Reduce Costs

1. **Use cost-effective model**:
   ```bash
   GEMINI_MODEL=gemini-2.5-flash-lite  # $0.10 per 1M input tokens
   ```

2. **Limit conversation history**:
   - Keep only last 20-30 messages
   - Summarize older messages

3. **Enable context caching** (future enhancement):
   - Cache system prompt and tool definitions
   - Reduces input token costs by 90%

## Monitoring

### Log Analysis

Check agent service logs:

```bash
tail -f logs/agent_service.log
```

Key metrics to monitor:
- Average iterations per request
- Tool execution success rate
- Error rate by type
- Response time (excluding LLM latency)

### Health Check

Add health check endpoint:

```python
@router.get("/api/agent/health")
async def agent_health_check():
    """Check agent service health."""
    try:
        # Test tool binding
        tools = bind_mcp_tools()

        # Test configuration
        config = get_default_config()

        return {
            "status": "healthy",
            "tools_count": len(tools),
            "model": config.model_name
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }
```

## Next Steps

1. **Implement agent service**: Follow tasks in `tasks.md` (generated by `/sp.tasks`)
2. **Create chat API endpoint**: Integrate agent service with FastAPI
3. **Add conversation persistence**: Store messages in database
4. **Build frontend chat UI**: Use OpenAI ChatKit (Spec 5)
5. **Deploy to production**: Configure environment variables

## Additional Resources

- [Gemini API Documentation](https://ai.google.dev/gemini-api/docs)
- [OpenAI Function Calling Guide](https://platform.openai.com/docs/guides/function-calling)
- [MCP Server Documentation](../004-mcp-task-tools/README.md)
- [Agent Service Contract](./contracts/agent_service_contract.md)

---

**Quickstart Complete**: Ready for implementation following task breakdown.
