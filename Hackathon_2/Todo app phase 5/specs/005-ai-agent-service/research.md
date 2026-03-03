# Research: AI Agent Service Configuration

**Date**: 2026-01-29
**Feature**: 005-ai-agent-service
**Purpose**: Research technical implementation details for agent service

## Research Summary

This document consolidates research findings for implementing an AI agent service using OpenAI SDK configured for Gemini API, with MCP tool binding and iterative agent loop execution.

## 1. Gemini API OpenAI Compatibility

### Configuration

**Base URL**: `https://generativelanguage.googleapis.com/v1beta/openai/`

**Authentication**:
- Use Gemini API key (not OpenAI key)
- Get from [Google AI Studio](https://aistudio.google.com/apikey)
- Set as environment variable: `GEMINI_API_KEY` or `GOOGLE_API_KEY`

**Basic Setup**:
```python
from openai import OpenAI
import os

client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)
```

### Models Supporting Function Calling

**Recommended for Production**:
- `gemini-2.5-flash` - Best price-performance (1M token context)
- `gemini-2.5-flash-lite` - Fastest, most cost-effective
- `gemini-3-flash-preview` - Balanced performance

**All Function Calling Support**:
- Gemini 3 Series: `gemini-3-pro-preview`, `gemini-3-flash-preview`
- Gemini 2.5 Series: `gemini-2.5-pro`, `gemini-2.5-flash`, `gemini-2.5-flash-lite`

**NO Function Calling**:
- `gemini-2.0-flash-lite`
- Image generation models
- Text-to-speech models

### Important Limitations

1. **Beta Status**: OpenAI compatibility is still in beta
2. **Function Calling**: Only subset of OpenAPI schema supported; very large/nested schemas may be rejected
3. **Reasoning Models**: Cannot disable reasoning for Gemini 2.5 Pro or 3 models
4. **Temperature**: Keep at default 1.0 for Gemini 3 models to avoid repetitive output
5. **File Operations**: Batch API file upload/download requires native `genai` client

### Pricing (per 1M tokens)

**Gemini 2.5 Flash-Lite** (Most Cost-Effective):
- Input: $0.10 (text/image/video), $0.30 (audio)
- Output: $0.40

**Gemini 2.5 Flash**:
- Input: $0.30 (text/image/video), $1.00 (audio)
- Output: $2.50

**Free Tier Available** for eligible models (content used to improve Google products)

### Rate Limits

Applied **per project** (not per API key):
- **RPM**: Requests per minute
- **TPM**: Tokens per minute (input)
- **RPD**: Requests per day (resets midnight Pacific)

**Usage Tiers**:
- Free Tier: Available in eligible countries
- Tier 1: Requires paid billing account
- Tier 2: Total spend > $250 and 30+ days
- Tier 3: Total spend > $1,000 and 30+ days

### Error Handling

Common error codes:
- **400 INVALID_ARGUMENT**: Malformed request
- **403 PERMISSION_DENIED**: API key lacks permissions
- **429 RESOURCE_EXHAUSTED**: Rate limit exceeded
- **500 INTERNAL**: Input context too long
- **503 UNAVAILABLE**: Service overloaded
- **504 DEADLINE_EXCEEDED**: Prompt too large

## 2. OpenAI Function Calling Format

### Schema Structure

```json
{
  "type": "function",
  "function": {
    "name": "function_name",
    "description": "Clear description of what the function does",
    "parameters": {
      "type": "object",
      "properties": {
        "param_name": {
          "type": "string|number|boolean|object|array",
          "description": "Description of the parameter",
          "enum": ["optional", "list", "of", "values"]
        }
      },
      "required": ["param1", "param2"],
      "additionalProperties": false
    }
  }
}
```

### Type Mapping: Python to JSON Schema

| Python Type | JSON Schema Type | Notes |
|-------------|------------------|-------|
| `str` | `"string"` | |
| `int` | `"integer"` | |
| `float` | `"number"` | |
| `bool` | `"boolean"` | |
| `list`, `List[T]` | `"array"` | Use `items` for element type |
| `dict`, `Dict[str, T]` | `"object"` | |
| `Optional[T]` | Same as T | Omit from `required` array |
| `Literal["a", "b"]` | Use `"enum": ["a", "b"]` | |
| Pydantic Model | `"object"` | Convert fields to properties |

### MCP Tool Conversion Example

**MCP Tool**:
```python
@mcp.tool()
async def add_task(
    user_id: str,
    title: str,
    description: str = None
) -> dict:
    """Create a new task for the user.

    Args:
        user_id: The authenticated user's UUID
        title: Task title (1-255 characters)
        description: Optional task description
    """
    pass
```

**OpenAI Schema**:
```json
{
  "type": "function",
  "function": {
    "name": "add_task",
    "description": "Create a new task for the user",
    "parameters": {
      "type": "object",
      "properties": {
        "user_id": {
          "type": "string",
          "description": "The authenticated user's UUID"
        },
        "title": {
          "type": "string",
          "description": "Task title (1-255 characters)"
        },
        "description": {
          "type": "string",
          "description": "Optional task description"
        }
      },
      "required": ["user_id", "title"],
      "additionalProperties": false
    }
  }
}
```

### Programmatic Conversion Pattern

```python
import inspect
from typing import get_type_hints, get_origin, get_args

def mcp_tool_to_openai_schema(func: callable) -> dict:
    """Convert MCP tool to OpenAI function schema."""
    sig = inspect.signature(func)
    type_hints = get_type_hints(func)
    docstring = inspect.getdoc(func) or ""

    properties = {}
    required = []

    for param_name, param in sig.parameters.items():
        if param_name == 'self':
            continue

        # Get type
        param_type = type_hints.get(param_name, str)
        json_type = python_type_to_json_schema(param_type)

        # Extract description from docstring (simplified)
        description = extract_param_description(docstring, param_name)
        if description:
            json_type["description"] = description

        properties[param_name] = json_type

        # Check if required
        if param.default == inspect.Parameter.empty:
            required.append(param_name)

    return {
        "type": "function",
        "function": {
            "name": func.__name__,
            "description": docstring.split('\n')[0],
            "parameters": {
                "type": "object",
                "properties": properties,
                "required": required,
                "additionalProperties": False
            }
        }
    }
```

## 3. Agent Loop Implementation Pattern

### Recommended Approach: While Loop

**Advantages**:
- Easier to implement max iteration limits
- Better stack management (no recursion depth issues)
- Clearer control flow
- Simpler debugging

### Core Pattern

```python
async def run_agent(
    client: AsyncOpenAI,
    messages: List[Dict[str, Any]],
    tools: List[Dict[str, Any]],
    max_iterations: int = 15
) -> str:
    """Execute agent loop with tool calling."""
    iteration_count = 0

    while iteration_count < max_iterations:
        iteration_count += 1

        # Call LLM
        response = await client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=messages,
            tools=tools,
            tool_choice="auto"
        )

        assistant_message = response.choices[0].message

        # Add assistant response to history
        messages.append({
            "role": "assistant",
            "content": assistant_message.content,
            "tool_calls": assistant_message.tool_calls
        })

        # Check termination: no tool calls = done
        if not assistant_message.tool_calls:
            return assistant_message.content

        # Execute tool calls
        tool_results = await execute_tools(assistant_message.tool_calls)

        # Append tool results to history
        for tool_result in tool_results:
            messages.append({
                "role": "tool",
                "tool_call_id": tool_result["tool_call_id"],
                "content": tool_result["content"]
            })

    # Max iterations reached - graceful exit
    return "I apologize, but I need more time to process this request. Please try breaking it into smaller steps."
```

### Termination Detection

**Primary Signal**: Absence of `tool_calls` in assistant message

```python
def should_continue(assistant_message) -> bool:
    """Agent continues if there are tool calls to execute."""
    return (assistant_message.tool_calls is not None and
            len(assistant_message.tool_calls) > 0)
```

### Graceful Max Iteration Handling

```python
async def graceful_exit(client, messages, model):
    """Request final summary when max iterations reached."""
    messages.append({
        "role": "system",
        "content": (
            "You have reached the maximum number of iterations. "
            "Please provide a final summary of what you've accomplished."
        )
    })

    final_response = await client.chat.completions.create(
        model=model,
        messages=messages,
        tool_choice="none"  # No more tool calls
    )

    return final_response.choices[0].message.content
```

### Tool Execution Pattern

```python
async def execute_tools(tool_calls) -> List[Dict[str, Any]]:
    """Execute tool calls in parallel."""

    async def execute_single(tool_call):
        try:
            function_name = tool_call.function.name
            arguments = json.loads(tool_call.function.arguments)

            # Execute tool from registry
            result = await tool_registry[function_name](**arguments)

            return {
                "tool_call_id": tool_call.id,
                "content": json.dumps(result),
                "status": "success"
            }
        except Exception as e:
            # Return error as tool result
            return {
                "tool_call_id": tool_call.id,
                "content": json.dumps({
                    "error": str(e),
                    "type": type(e).__name__
                }),
                "status": "error"
            }

    # Execute all tools in parallel
    results = await asyncio.gather(
        *[execute_single(tc) for tc in tool_calls]
    )

    return results
```

### Message Format for Tool Results

**Critical Format Requirements**:

```python
# 1. User message
{"role": "user", "content": "Create a task to buy groceries"}

# 2. Assistant message with tool calls
{
    "role": "assistant",
    "content": None,  # Can be None or explanatory text
    "tool_calls": [{
        "id": "call_abc123",
        "type": "function",
        "function": {
            "name": "add_task",
            "arguments": '{"user_id": "123", "title": "Buy groceries"}'
        }
    }]
}

# 3. Tool result message (MUST match tool_call.id)
{
    "role": "tool",
    "tool_call_id": "call_abc123",  # MUST match
    "content": '{"task_id": 1, "status": "created"}'  # MUST be string
}

# 4. Assistant's final response
{
    "role": "assistant",
    "content": "I've created the task 'Buy groceries' for you."
}
```

**Important**:
- Tool results MUST use `"role": "tool"`
- `tool_call_id` MUST match the ID from assistant's tool call
- `content` MUST be a string (serialize JSON if needed)
- One tool message per tool call, in same order

## 4. Error Handling Strategies

### LLM API Errors

```python
from openai import APIError, RateLimitError, APITimeoutError

try:
    response = await client.chat.completions.create(...)
except RateLimitError:
    return "I'm currently experiencing high demand. Please try again in a moment."
except APITimeoutError:
    return "The request took too long. Please try a simpler query."
except APIError as e:
    logger.error(f"LLM API error: {e}")
    return "I'm having trouble connecting to my AI service. Please try again."
except Exception as e:
    logger.error(f"Unexpected error: {e}")
    return "An unexpected error occurred. Please contact support."
```

### Tool Execution Errors

```python
async def execute_tool_with_error_handling(tool_call):
    """Execute tool and return result or error."""
    try:
        function_name = tool_call.function.name
        arguments = json.loads(tool_call.function.arguments)

        # Validate tool exists
        if function_name not in tool_registry:
            raise ValueError(f"Unknown tool: {function_name}")

        # Execute tool
        result = await tool_registry[function_name](**arguments)

        return {
            "tool_call_id": tool_call.id,
            "content": json.dumps(result),
            "status": "success"
        }

    except TaskNotFoundError as e:
        # Specific tool error - return as tool result
        return {
            "tool_call_id": tool_call.id,
            "content": json.dumps({
                "error": "Task not found",
                "message": str(e)
            }),
            "status": "error"
        }
    except UnauthorizedTaskAccessError as e:
        return {
            "tool_call_id": tool_call.id,
            "content": json.dumps({
                "error": "Unauthorized access",
                "message": "You don't have permission to access this task"
            }),
            "status": "error"
        }
    except Exception as e:
        logger.error(f"Tool execution error ({function_name}): {e}")
        return {
            "tool_call_id": tool_call.id,
            "content": json.dumps({
                "error": "Tool execution failed",
                "message": str(e)
            }),
            "status": "error"
        }
```

### User-Friendly Error Messages

**Principle**: Never expose technical details to users

```python
ERROR_MESSAGES = {
    "rate_limit": "I'm currently experiencing high demand. Please try again in a moment.",
    "timeout": "That request took too long. Please try a simpler query.",
    "api_error": "I'm having trouble connecting to my AI service. Please try again.",
    "tool_not_found": "I don't have access to that capability.",
    "task_not_found": "I couldn't find that task. It may have been deleted.",
    "unauthorized": "You don't have permission to perform that action.",
    "max_iterations": "I need more time to process this request. Please try breaking it into smaller steps.",
    "unknown": "An unexpected error occurred. Please try again or contact support."
}
```

## 5. Message Format Conversion

### Database to OpenAI Format

**Database Message Format** (from Spec 3):
```python
class Message(SQLModel, table=True):
    id: int
    conversation_id: int
    role: str  # "user" or "assistant"
    content: str
    created_at: datetime
```

**Conversion Function**:
```python
def convert_db_messages_to_openai(
    db_messages: List[Message],
    system_prompt: str,
    user_id: str
) -> List[Dict[str, str]]:
    """Convert database messages to OpenAI chat format."""

    # Start with system prompt (inject user_id for context)
    openai_messages = [{
        "role": "system",
        "content": system_prompt.format(user_id=user_id)
    }]

    # Convert each database message
    for msg in db_messages:
        openai_messages.append({
            "role": msg.role,  # "user" or "assistant"
            "content": msg.content
        })

    return openai_messages
```

### System Prompt Injection

```python
SYSTEM_PROMPT_TEMPLATE = """You are a helpful Todo Assistant for user {user_id}.

CRITICAL RULES:
1. You can ONLY manage tasks for user {user_id}
2. NEVER access or modify tasks for other users
3. Always include user_id={user_id} in all tool calls
4. If asked about time-relative queries (today, tomorrow), check current time first
5. Keep responses concise and friendly
6. If a tool fails, explain the error in user-friendly language

Available tools:
- add_task: Create new tasks
- list_tasks: View user's tasks
- complete_task: Mark tasks as done
- update_task: Modify task details
- delete_task: Remove tasks

Remember: You can only help with task management. For other requests, politely explain your limitations."""

# Usage
system_prompt = SYSTEM_PROMPT_TEMPLATE.format(user_id=user_id)
```

## 6. Dependencies to Add

### requirements.txt Updates

```txt
# Existing dependencies
fastapi
uvicorn
sqlmodel
pydantic
pydantic-settings
psycopg2-binary
python-jose[cryptography]
passlib
bcrypt
python-multipart
cryptography
httpx
mcp[cli]
asyncpg
pytest-asyncio

# NEW - Add these for agent service
openai>=1.0.0
```

**Note**: `google-generativeai` is NOT needed if using OpenAI SDK with Gemini endpoint

### Environment Variables

Add to `.env`:
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

## 7. Best Practices Summary

### Configuration
1. Use `gemini-2.5-flash` or `gemini-2.5-flash-lite` for cost-efficiency
2. Keep temperature at 1.0 for Gemini 3 models
3. Store API key in environment variables
4. Set reasonable timeout (30 seconds)

### Agent Loop
1. Use while loops over recursion
2. Detect termination by absence of tool_calls
3. Implement graceful max iteration handling
4. Execute tools in parallel with asyncio.gather()
5. Set max_iterations between 10-20

### Tool Binding
1. Extract descriptions from docstrings
2. Use Pydantic models for validation
3. Handle Optional types correctly
4. Always specify required parameters

### Error Handling
1. Return errors as tool results, don't crash
2. Provide user-friendly error messages
3. Log technical details for debugging
4. Handle rate limits gracefully

### Message Format
1. Preserve exact OpenAI message structure
2. Tool results must use "role": "tool"
3. tool_call_id must match exactly
4. Content must be string (serialize JSON)

### Security
1. Never commit API keys
2. Validate user_id in all tool calls
3. Inject user_id in system prompt
4. Don't expose technical errors to users

## 8. Implementation Checklist

- [ ] Install openai package
- [ ] Configure Gemini API credentials
- [ ] Create agent_config.py with settings
- [ ] Implement bind_mcp_tools() function
- [ ] Implement execute_tool_call() function
- [ ] Implement run_agent() with while loop
- [ ] Add system prompt with user isolation
- [ ] Implement graceful max iteration handling
- [ ] Add comprehensive error handling
- [ ] Create test suite
- [ ] Document usage in quickstart.md

---

**Research Complete**: All technical questions resolved. Ready for Phase 1 design and implementation.
