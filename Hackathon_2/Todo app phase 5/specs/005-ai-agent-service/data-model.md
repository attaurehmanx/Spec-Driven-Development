# Data Model: AI Agent Service Configuration

**Feature**: 005-ai-agent-service
**Date**: 2026-01-29
**Purpose**: Define entities and data structures for agent service

## Overview

This document defines the data models and entities used by the AI agent service. These are configuration and runtime entities, not database models (agent service is stateless).

## Entity Definitions

### 1. AgentConfiguration

**Purpose**: Encapsulates LLM client configuration and agent behavior settings

**Type**: Configuration dataclass/Pydantic model

**Attributes**:

| Attribute | Type | Default | Description | Validation |
|-----------|------|---------|-------------|------------|
| `api_base_url` | str | `"https://generativelanguage.googleapis.com/v1beta/openai/"` | Gemini API endpoint | Must be valid URL |
| `api_key` | str | From env | API authentication key | Must not be empty |
| `model_name` | str | `"gemini-2.5-flash"` | Gemini model identifier | Must be valid model |
| `temperature` | float | `1.0` | Response randomness (0.0-1.0) | 0.0 ≤ temp ≤ 1.0 |
| `max_tokens` | int | `1000` | Maximum response length | Must be positive |
| `timeout` | int | `30` | API request timeout (seconds) | Must be positive |

**Validation Rules**:
- `api_key` must not be empty or None
- `temperature` must be between 0.0 and 1.0 (inclusive)
- `max_tokens` must be positive integer
- `timeout` must be positive integer
- `model_name` must be one of supported Gemini models

**Example**:
```python
from pydantic import BaseModel, Field

class AgentConfiguration(BaseModel):
    api_base_url: str = Field(
        default="https://generativelanguage.googleapis.com/v1beta/openai/",
        description="Gemini API endpoint"
    )
    api_key: str = Field(..., description="API authentication key")
    model_name: str = Field(
        default="gemini-2.5-flash",
        description="Gemini model identifier"
    )
    temperature: float = Field(
        default=1.0,
        ge=0.0,
        le=1.0,
        description="Response randomness"
    )
    max_tokens: int = Field(
        default=1000,
        gt=0,
        description="Maximum response length"
    )
    timeout: int = Field(
        default=30,
        gt=0,
        description="API request timeout in seconds"
    )
```

---

### 2. SystemPrompt

**Purpose**: Defines agent behavior, constraints, and personality

**Type**: Configuration dataclass/Pydantic model

**Attributes**:

| Attribute | Type | Default | Description | Validation |
|-----------|------|---------|-------------|------------|
| `role_definition` | str | "You are a helpful Todo Assistant" | Agent's primary role | Must not be empty |
| `operational_rules` | List[str] | See below | Behavior constraints | Must include user isolation rule |
| `response_guidelines` | List[str] | See below | Tone and format instructions | Must include conciseness rule |
| `tool_usage_instructions` | str | See below | How to use available tools | Must not be empty |

**Default Operational Rules**:
1. "You can ONLY manage tasks for the authenticated user"
2. "NEVER access or modify tasks for other users"
3. "Always include the correct user_id in all tool calls"
4. "Check current time when processing time-relative queries (today, tomorrow)"
5. "If a tool fails, explain the error in user-friendly language"

**Default Response Guidelines**:
1. "Keep responses concise (under 200 words)"
2. "Maintain a friendly, helpful tone"
3. "Avoid technical jargon"
4. "Provide actionable next steps when appropriate"

**Default Tool Usage Instructions**:
```
Available tools:
- add_task: Create new tasks
- list_tasks: View user's tasks (supports filtering by status)
- complete_task: Mark tasks as completed
- update_task: Modify task title or description
- delete_task: Permanently remove tasks

Always validate user_id matches the authenticated user before calling tools.
```

**Validation Rules**:
- `role_definition` must not be empty
- `operational_rules` must contain at least one rule about user isolation
- `response_guidelines` must contain at least one rule about conciseness
- `tool_usage_instructions` must not be empty

**Example**:
```python
from pydantic import BaseModel, Field, field_validator
from typing import List

class SystemPrompt(BaseModel):
    role_definition: str = Field(
        default="You are a helpful Todo Assistant",
        description="Agent's primary role"
    )
    operational_rules: List[str] = Field(
        default_factory=lambda: [
            "You can ONLY manage tasks for the authenticated user",
            "NEVER access or modify tasks for other users",
            "Always include the correct user_id in all tool calls",
            "Check current time when processing time-relative queries",
            "If a tool fails, explain the error in user-friendly language"
        ],
        description="Behavior constraints"
    )
    response_guidelines: List[str] = Field(
        default_factory=lambda: [
            "Keep responses concise (under 200 words)",
            "Maintain a friendly, helpful tone",
            "Avoid technical jargon",
            "Provide actionable next steps when appropriate"
        ],
        description="Tone and format instructions"
    )
    tool_usage_instructions: str = Field(
        default="Available tools: add_task, list_tasks, complete_task, update_task, delete_task",
        description="How to use available tools"
    )

    @field_validator('operational_rules')
    @classmethod
    def must_include_user_isolation(cls, v):
        has_isolation_rule = any('user' in rule.lower() and 'only' in rule.lower() for rule in v)
        if not has_isolation_rule:
            raise ValueError("operational_rules must include user isolation rule")
        return v

    def to_prompt_string(self, user_id: str) -> str:
        """Convert to formatted system prompt string."""
        rules = "\n".join(f"- {rule}" for rule in self.operational_rules)
        guidelines = "\n".join(f"- {guideline}" for guideline in self.response_guidelines)

        return f"""{self.role_definition} for user {user_id}.

CRITICAL RULES:
{rules}

RESPONSE GUIDELINES:
{guidelines}

{self.tool_usage_instructions}

Remember: You can only help with task management. For other requests, politely explain your limitations."""
```

---

### 3. AgentLoopConfig

**Purpose**: Controls agent loop execution parameters

**Type**: Configuration dataclass/Pydantic model

**Attributes**:

| Attribute | Type | Default | Description | Validation |
|-----------|------|---------|-------------|------------|
| `max_iterations` | int | `15` | Maximum loop iterations | 1 ≤ max ≤ 50 |
| `iteration_timeout` | int | `30` | Timeout per iteration (seconds) | Must be positive |
| `enable_retry` | bool | `False` | Whether to retry failed tool calls | - |
| `retry_attempts` | int | `1` | Number of retry attempts if enabled | Must be non-negative |

**Validation Rules**:
- `max_iterations` must be between 1 and 50 (inclusive)
- `iteration_timeout` must be positive integer
- `retry_attempts` must be non-negative integer
- If `enable_retry` is False, `retry_attempts` is ignored

**Example**:
```python
from pydantic import BaseModel, Field

class AgentLoopConfig(BaseModel):
    max_iterations: int = Field(
        default=15,
        ge=1,
        le=50,
        description="Maximum loop iterations"
    )
    iteration_timeout: int = Field(
        default=30,
        gt=0,
        description="Timeout per iteration in seconds"
    )
    enable_retry: bool = Field(
        default=False,
        description="Whether to retry failed tool calls"
    )
    retry_attempts: int = Field(
        default=1,
        ge=0,
        description="Number of retry attempts if enabled"
    )
```

---

### 4. ToolBinding

**Purpose**: Maps MCP tools to OpenAI function calling format

**Type**: Runtime dataclass

**Attributes**:

| Attribute | Type | Description | Validation |
|-----------|------|-------------|------------|
| `tool_name` | str | MCP tool function name | Must match MCP tool |
| `openai_function_schema` | dict | OpenAI function calling schema | Must be valid OpenAI format |
| `mcp_tool_reference` | callable | Reference to actual MCP tool function | Must be callable |

**Validation Rules**:
- `tool_name` must match the actual MCP tool function name
- `openai_function_schema` must conform to OpenAI function calling schema format
- `mcp_tool_reference` must be a callable async function

**Example**:
```python
from typing import Callable, Dict, Any
from pydantic import BaseModel, Field, field_validator

class ToolBinding(BaseModel):
    tool_name: str = Field(..., description="MCP tool function name")
    openai_function_schema: Dict[str, Any] = Field(
        ...,
        description="OpenAI function calling schema"
    )
    mcp_tool_reference: Callable = Field(
        ...,
        description="Reference to actual MCP tool function"
    )

    class Config:
        arbitrary_types_allowed = True  # Allow callable type

    @field_validator('openai_function_schema')
    @classmethod
    def validate_schema_format(cls, v):
        required_keys = ['type', 'function']
        if not all(key in v for key in required_keys):
            raise ValueError("Schema must have 'type' and 'function' keys")
        if v['type'] != 'function':
            raise ValueError("Schema type must be 'function'")
        return v

    @field_validator('mcp_tool_reference')
    @classmethod
    def validate_callable(cls, v):
        if not callable(v):
            raise ValueError("mcp_tool_reference must be callable")
        return v
```

---

### 5. AgentResponse

**Purpose**: Encapsulates the final agent response with metadata

**Type**: Response dataclass

**Attributes**:

| Attribute | Type | Description |
|-----------|------|-------------|
| `status` | str | Response status: "completed", "max_iterations_reached", "error" |
| `final_response` | str | Final agent response text (None if error) |
| `messages` | List[Dict] | Complete message history including tool calls |
| `iterations` | int | Number of iterations executed |
| `finish_reason` | str | Reason for completion: "completed", "max_iterations", "error" |
| `error` | Optional[str] | Error message if status is "error" |
| `warning` | Optional[str] | Warning message if applicable |

**Example**:
```python
from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field

class AgentResponse(BaseModel):
    status: str = Field(
        ...,
        description="Response status",
        pattern="^(completed|max_iterations_reached|error)$"
    )
    final_response: Optional[str] = Field(
        None,
        description="Final agent response text"
    )
    messages: List[Dict[str, Any]] = Field(
        default_factory=list,
        description="Complete message history"
    )
    iterations: int = Field(
        ...,
        ge=0,
        description="Number of iterations executed"
    )
    finish_reason: str = Field(
        ...,
        description="Reason for completion"
    )
    error: Optional[str] = Field(
        None,
        description="Error message if status is error"
    )
    warning: Optional[str] = Field(
        None,
        description="Warning message if applicable"
    )
```

---

## Entity Relationships

```
AgentConfiguration
    ↓ (used by)
run_agent()
    ↓ (uses)
SystemPrompt → to_prompt_string() → injected into messages
    ↓ (uses)
AgentLoopConfig → controls iteration limits
    ↓ (uses)
ToolBinding[] → provides tool schemas and references
    ↓ (produces)
AgentResponse → returned to caller
```

## Usage Example

```python
from services.agent_service import run_agent
from config.agent_config import get_default_config

# Get default configuration
config = get_default_config()

# Prepare message history
message_history = [
    {"role": "user", "content": "Show me my tasks"}
]

# Run agent
response = await run_agent(
    message_history=message_history,
    user_id="550e8400-e29b-41d4-a716-446655440000",
    config=config
)

# Handle response
if response.status == "completed":
    print(f"Agent: {response.final_response}")
elif response.status == "error":
    print(f"Error: {response.error}")
```

---

**Data Model Complete**: All entities defined with validation rules and examples.
