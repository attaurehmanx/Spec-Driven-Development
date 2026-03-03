# Agent Service Contract

**Feature**: 005-ai-agent-service
**Date**: 2026-01-29
**Purpose**: Define the public interface for the AI agent service

## Overview

This document defines the contract (public API) for the agent service module. All functions are async and follow Python async/await patterns.

---

## Function: `run_agent`

### Purpose
Execute agent loop to process user message with conversation context and return AI-generated response.

### Signature
```python
async def run_agent(
    message_history: List[Dict[str, str]],
    user_id: str,
    config: Optional[AgentConfiguration] = None
) -> AgentResponse
```

### Input Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `message_history` | `List[Dict[str, str]]` | Yes | List of previous conversation messages |
| `user_id` | `str` | Yes | Authenticated user's UUID (for tool calls) |
| `config` | `Optional[AgentConfiguration]` | No | Agent configuration (uses default if None) |

**message_history format**:
```python
[
    {"role": "user", "content": "Show me my tasks"},
    {"role": "assistant", "content": "Let me check your tasks..."},
    {"role": "user", "content": "Create a task to buy groceries"}
]
```

**Constraints**:
- `message_history` must not be empty
- Each message must have `role` and `content` keys
- `role` must be "user" or "assistant"
- `user_id` must be valid UUID format
- `config` is optional; default configuration used if None

### Output

**Type**: `AgentResponse`

**Structure**:
```python
{
    "status": "completed",  # or "max_iterations_reached", "error"
    "final_response": "I've created a task titled 'Buy groceries' for you.",
    "messages": [...],  # Complete message history
    "iterations": 3,
    "finish_reason": "completed",
    "error": None,
    "warning": None
}
```

### Exceptions

| Exception | When Raised | User-Facing Message |
|-----------|-------------|---------------------|
| `ValueError` | Invalid input parameters | "Invalid request parameters" |
| `LLMAPIError` | LLM API communication failure | "I'm having trouble connecting to my AI service. Please try again." |
| `ToolExecutionError` | Tool execution failure | Specific error from tool (user-friendly) |
| `MaxIterationsExceededError` | Agent loop exceeded max iterations | "I need more time to process this request. Please try breaking it into smaller steps." |

**Note**: Most errors are caught and returned in `AgentResponse.error` rather than raised as exceptions.

### Behavior

1. **Input Validation**:
   - Verify `message_history` is not empty
   - Validate `user_id` format (UUID)
   - Validate message structure (role, content keys)

2. **Message Preparation**:
   - Convert `message_history` to OpenAI format
   - Inject system prompt with `user_id` context
   - Prepare tool definitions from MCP tools

3. **Agent Loop Execution**:
   - Initialize iteration counter = 0
   - While iteration < max_iterations:
     - Call LLM with messages and tool definitions
     - If no tool calls: return final response
     - Execute tool calls with `user_id` validation
     - Append tool results to messages
     - Increment iteration counter

4. **Termination Handling**:
   - Natural completion: Return final response
   - Max iterations: Request summary and return with warning
   - Error: Return error response with user-friendly message

5. **Error Handling**:
   - Catch all exceptions
   - Log technical details
   - Return user-friendly error messages
   - Never expose sensitive information

### Example Usage

**Basic Usage**:
```python
from services.agent_service import run_agent

# Simple query
message_history = [
    {"role": "user", "content": "What tasks do I have?"}
]

response = await run_agent(
    message_history=message_history,
    user_id="550e8400-e29b-41d4-a716-446655440000"
)

if response.status == "completed":
    print(f"Agent: {response.final_response}")
else:
    print(f"Error: {response.error}")
```

**Multi-Turn Conversation**:
```python
# Conversation with context
message_history = [
    {"role": "user", "content": "Show me my tasks"},
    {"role": "assistant", "content": "You have 3 tasks: ..."},
    {"role": "user", "content": "Mark the first one as complete"}
]

response = await run_agent(
    message_history=message_history,
    user_id="550e8400-e29b-41d4-a716-446655440000"
)
```

**Custom Configuration**:
```python
from config.agent_config import AgentConfiguration

# Custom config
config = AgentConfiguration(
    api_key=os.getenv("GEMINI_API_KEY"),
    model_name="gemini-2.5-flash-lite",
    temperature=0.8,
    max_tokens=500
)

response = await run_agent(
    message_history=message_history,
    user_id=user_id,
    config=config
)
```

---

## Function: `bind_mcp_tools`

### Purpose
Convert MCP tool definitions to OpenAI function calling format.

### Signature
```python
def bind_mcp_tools() -> List[Dict[str, Any]]
```

### Input Parameters
None (discovers tools from MCP server)

### Output

**Type**: `List[Dict[str, Any]]`

**Structure**: List of OpenAI function schemas
```python
[
    {
        "type": "function",
        "function": {
            "name": "add_task",
            "description": "Create a new task for the user",
            "parameters": {
                "type": "object",
                "properties": {
                    "user_id": {"type": "string", "description": "User's UUID"},
                    "title": {"type": "string", "description": "Task title"},
                    "description": {"type": "string", "description": "Optional description"}
                },
                "required": ["user_id", "title"],
                "additionalProperties": false
            }
        }
    },
    # ... other tools
]
```

### Exceptions

| Exception | When Raised |
|-----------|-------------|
| `ImportError` | Cannot import MCP tools module |
| `ValueError` | Tool definition is invalid |

### Behavior

1. **Tool Discovery**:
   - Import all MCP tools from `mcp_server.tools.task_tools`
   - Identify functions decorated with `@mcp.tool()`

2. **Schema Extraction**:
   - For each tool, extract:
     - Function name
     - Docstring (for description)
     - Parameter types and descriptions (from type hints and Pydantic models)
     - Required vs optional parameters

3. **Schema Conversion**:
   - Convert Python types to JSON Schema types
   - Build OpenAI function calling schema
   - Validate schema format

4. **Return**:
   - Return list of all tool schemas

### Example Usage

```python
from services.agent_service import bind_mcp_tools

# Get all tool schemas
tools = bind_mcp_tools()

print(f"Found {len(tools)} tools:")
for tool in tools:
    print(f"  - {tool['function']['name']}")

# Use with OpenAI client
response = await client.chat.completions.create(
    model="gemini-2.5-flash",
    messages=messages,
    tools=tools,
    tool_choice="auto"
)
```

---

## Function: `execute_tool_call`

### Purpose
Execute a single MCP tool call and return result.

### Signature
```python
async def execute_tool_call(
    tool_name: str,
    tool_arguments: Dict[str, Any],
    user_id: str
) -> Dict[str, Any]
```

### Input Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `tool_name` | `str` | Yes | Name of MCP tool to execute |
| `tool_arguments` | `Dict[str, Any]` | Yes | Tool arguments from LLM |
| `user_id` | `str` | Yes | User ID to inject into tool call |

### Output

**Type**: `Dict[str, Any]`

**Success Response**:
```python
{
    "status": "success",
    "result": {
        "task_id": 1,
        "title": "Buy groceries",
        "completed": false,
        # ... other task fields
    }
}
```

**Error Response**:
```python
{
    "status": "error",
    "error": "Task not found",
    "error_type": "TaskNotFoundError",
    "message": "Task with ID 123 does not exist"
}
```

### Exceptions

| Exception | When Raised | Handling |
|-----------|-------------|----------|
| `ToolNotFoundError` | Tool name not recognized | Return error dict |
| `TaskNotFoundError` | Task doesn't exist | Return error dict |
| `UnauthorizedTaskAccessError` | User doesn't own task | Return error dict |
| `ValidationError` | Invalid arguments | Return error dict |

**Note**: Exceptions are caught and returned as error dicts, not raised.

### Behavior

1. **Validation**:
   - Verify `tool_name` exists in MCP tools
   - Validate `tool_arguments` structure

2. **User ID Injection**:
   - Inject `user_id` into `tool_arguments`
   - Override if `user_id` already present (security)

3. **Tool Execution**:
   - Call MCP tool with arguments
   - Await async result

4. **Error Handling**:
   - Catch tool-specific exceptions (TaskNotFoundError, etc.)
   - Convert to user-friendly error messages
   - Return error dict instead of raising

5. **Result Formatting**:
   - Wrap successful result in standard format
   - Include status indicator

### Example Usage

```python
from services.agent_service import execute_tool_call

# Execute tool
result = await execute_tool_call(
    tool_name="add_task",
    tool_arguments={
        "title": "Buy groceries",
        "description": "Milk, eggs, bread"
    },
    user_id="550e8400-e29b-41d4-a716-446655440000"
)

if result["status"] == "success":
    print(f"Task created: {result['result']['task_id']}")
else:
    print(f"Error: {result['message']}")
```

---

## Integration Example

**Complete workflow from chat API endpoint**:

```python
from fastapi import APIRouter, Depends, HTTPException
from services.agent_service import run_agent
from models.conversation import Conversation
from models.message import Message
from database.session import get_async_session

router = APIRouter()

@router.post("/api/{user_id}/chat")
async def chat_endpoint(
    user_id: str,
    request: ChatRequest,
    session = Depends(get_async_session)
):
    """Chat endpoint that uses agent service."""

    # 1. Load conversation history from database
    conversation = await get_or_create_conversation(session, user_id)
    db_messages = await get_conversation_messages(session, conversation.id)

    # 2. Convert to agent format
    message_history = [
        {"role": msg.role, "content": msg.content}
        for msg in db_messages
    ]

    # 3. Add new user message
    message_history.append({
        "role": "user",
        "content": request.message
    })

    # 4. Persist user message to database
    user_message = Message(
        conversation_id=conversation.id,
        role="user",
        content=request.message
    )
    session.add(user_message)
    await session.commit()

    # 5. Run agent
    response = await run_agent(
        message_history=message_history,
        user_id=user_id
    )

    # 6. Handle response
    if response.status == "error":
        raise HTTPException(status_code=500, detail=response.error)

    # 7. Persist assistant message to database
    assistant_message = Message(
        conversation_id=conversation.id,
        role="assistant",
        content=response.final_response
    )
    session.add(assistant_message)
    await session.commit()

    # 8. Return response
    return {
        "response": response.final_response,
        "iterations": response.iterations,
        "status": response.status
    }
```

---

## Error Handling Contract

### User-Facing Error Messages

All errors returned to users must be friendly and non-technical:

| Internal Error | User-Facing Message |
|----------------|---------------------|
| `RateLimitError` | "I'm currently experiencing high demand. Please try again in a moment." |
| `APITimeoutError` | "That request took too long. Please try a simpler query." |
| `APIError` | "I'm having trouble connecting to my AI service. Please try again." |
| `TaskNotFoundError` | "I couldn't find that task. It may have been deleted." |
| `UnauthorizedTaskAccessError` | "You don't have permission to access that task." |
| `MaxIterationsExceededError` | "I need more time to process this request. Please try breaking it into smaller steps." |
| `ValidationError` | "I couldn't understand that request. Please try rephrasing." |
| `Unknown` | "An unexpected error occurred. Please try again or contact support." |

### Logging Contract

All functions must log:
- **INFO**: Normal operations (iterations, tool calls)
- **WARNING**: Max iterations reached, rate limits
- **ERROR**: Exceptions, tool failures, API errors

**Example**:
```python
logger.info(f"Agent iteration {iteration}/{max_iterations}")
logger.info(f"Executing tool: {tool_name}")
logger.warning(f"Max iterations ({max_iterations}) reached")
logger.error(f"Tool execution error ({tool_name}): {e}")
```

---

## Performance Contract

### Response Time Targets

- **95th percentile**: < 3 seconds (excluding LLM API latency)
- **99th percentile**: < 5 seconds (excluding LLM API latency)

### Resource Limits

- **Max iterations**: 15 (configurable)
- **Max tokens**: 1000 (configurable)
- **Timeout per iteration**: 30 seconds (configurable)
- **Max conversation history**: 50 messages (caller's responsibility)

---

## Security Contract

### User Isolation

- **MUST** inject `user_id` in all tool calls
- **MUST** validate `user_id` matches authenticated user
- **MUST** include user isolation rules in system prompt
- **MUST NOT** allow cross-user data access

### API Key Security

- **MUST** load API key from environment variables
- **MUST NOT** log or expose API keys
- **MUST NOT** include API keys in error messages

### Input Validation

- **MUST** validate all input parameters
- **MUST** sanitize user input before passing to LLM
- **MUST** validate tool arguments before execution

---

**Contract Complete**: All public interfaces defined with examples and constraints.
