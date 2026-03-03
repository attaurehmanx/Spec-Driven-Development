# Implementation Plan: AI Agent Service Configuration

**Branch**: `005-ai-agent-service` | **Date**: 2026-01-29 | **Spec**: [spec.md](./spec.md)

## Summary

Configure an AI agent service that orchestrates conversational task management using Google Gemini API (via OpenAI-compatible interface) and MCP tools. The agent service will accept message history and user_id, execute an iterative agent loop (max 15 iterations) that calls MCP tools as needed, and return natural language responses. This enables the backend chat API to provide intelligent, context-aware task management through natural language.

**Primary Requirement**: Implement a stateless agent runner function that processes user messages, maintains conversation context, executes MCP task tools, and returns AI-generated responses.

**Technical Approach**: Use OpenAI Python SDK configured with Gemini API endpoint, bind existing MCP tools to agent capabilities, implement system prompt with user isolation rules, and create an iterative agent loop with error handling and iteration limits.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: OpenAI Python SDK (configured for Gemini), FastMCP (already installed), existing MCP tools
**Storage**: N/A (agent service is stateless; conversation history provided by caller)
**Testing**: pytest-asyncio (already installed), manual testing with mock prompts
**Target Platform**: Linux server (FastAPI backend)
**Project Type**: Web application backend service
**Performance Goals**: <3 seconds response time for 95% of requests (excluding LLM API latency)
**Constraints**: Max 15 agent loop iterations, synchronous operation (no streaming), English language only
**Scale/Scope**: Single agent service module, 5 MCP tools to bind, system prompt configuration

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Check

- ✅ **Principle I (Spec-first)**: Approved spec exists at `specs/005-ai-agent-service/spec.md`
- ✅ **Principle II (Single responsibility)**: Agent service has single responsibility - orchestrate LLM conversations with MCP tools
- ✅ **Principle III (Explicit contracts)**: Agent service contract defined - accepts (message_history, user_id), returns text response
- ✅ **Principle IV (Security by default)**: User_id validation enforced in system prompt and tool calls
- ✅ **Principle V (Determinism)**: Same inputs produce equivalent outputs (LLM non-determinism acknowledged but controlled via temperature)
- ✅ **Principle VI (Agentic discipline)**: All code generated via Claude Code
- ✅ **Principle VII (Stateless AI)**: Agent service is stateless - no in-memory conversation state
- ✅ **Principle VIII (Conversation persistence)**: Persistence handled by calling API, not agent service
- ✅ **Principle IX (User data isolation)**: System prompt enforces user_id filtering, all tool calls include user_id validation

**Status**: ✅ PASSED - No constitutional violations

### Post-Design Check

*To be completed after Phase 1*

## Project Structure

### Documentation (this feature)

```text
specs/005-ai-agent-service/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (API configuration, best practices)
├── data-model.md        # Phase 1 output (agent configuration entities)
├── quickstart.md        # Phase 1 output (setup and testing guide)
├── contracts/           # Phase 1 output (agent service interface)
│   └── agent_service_contract.md
└── checklists/
    └── requirements.md  # Specification quality checklist
```

### Source Code (repository root)

```text
backend/
├── services/
│   ├── __init__.py
│   ├── user_service.py          # Existing
│   └── agent_service.py         # NEW - Agent orchestration service
├── mcp_server/
│   ├── server.py                # Existing - MCP server instance
│   └── tools/
│       └── task_tools.py        # Existing - 5 MCP tools (add, list, complete, update, delete)
├── config/
│   ├── __init__.py
│   ├── settings.py              # Existing - May need Gemini API key config
│   └── agent_config.py          # NEW - Agent configuration (system prompt, limits)
├── models/
│   ├── task_models.py           # Existing
│   ├── conversation.py          # Existing
│   └── message.py               # Existing
├── api/
│   └── chat.py                  # FUTURE - Chat endpoint (not in this spec)
├── tests/
│   └── test_agent_service.py    # NEW - Agent service tests
├── requirements.txt             # UPDATE - Add openai, google-generativeai
└── .env                         # UPDATE - Add GEMINI_API_KEY
```

**Structure Decision**: Web application structure (Option 2) with backend service layer. Agent service lives in `backend/services/agent_service.py` alongside existing `user_service.py`. Configuration separated into `backend/config/agent_config.py` for maintainability (NFR-005).

## Complexity Tracking

> **No constitutional violations - this section is empty**

## Phase 0: Research & Discovery

### Research Questions

1. **Gemini API Configuration**: How to configure OpenAI Python SDK to use Gemini API endpoint?
2. **Tool Binding**: How to convert MCP tool definitions to OpenAI function calling format?
3. **Agent Loop Pattern**: Best practices for implementing iterative agent loops with tool execution?
4. **Error Handling**: How to handle LLM API failures, timeouts, and tool execution errors gracefully?
5. **Message Format Conversion**: How to convert database message format to OpenAI chat completion format?

### Research Tasks

#### Task 1: Gemini API OpenAI Compatibility Research
**Objective**: Determine exact configuration for OpenAI SDK to work with Gemini API

**Questions to Answer**:
- What is the correct `base_url` for Gemini's OpenAI-compatible endpoint?
- What authentication method does Gemini use (API key header format)?
- Which Gemini model supports function calling (tool use)?
- Are there any limitations or differences from OpenAI's API?

**Expected Output**: Configuration snippet showing:
```python
from openai import OpenAI

client = OpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=os.getenv("GEMINI_API_KEY")
)
```

#### Task 2: MCP Tool to OpenAI Function Format Conversion
**Objective**: Understand how to convert MCP tool definitions to OpenAI function calling schema

**Questions to Answer**:
- What is the OpenAI function calling schema format?
- How to extract tool name, description, and parameters from MCP tools?
- How to handle Pydantic models in tool parameters?
- How to map MCP tool responses back to agent?

**Expected Output**: Conversion pattern showing:
```python
# MCP tool: @mcp.tool() async def add_task(user_id: str, title: str, description: str = None)
# Converts to OpenAI format:
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
            "required": ["user_id", "title"]
        }
    }
}
```

#### Task 3: Agent Loop Implementation Pattern
**Objective**: Research best practices for implementing agent loops with tool execution

**Questions to Answer**:
- How to structure the agent loop (while loop vs recursive)?
- How to detect when agent is done (no more tool calls)?
- How to handle max iteration limit gracefully?
- How to accumulate tool results and feed back to LLM?

**Expected Output**: Pseudocode pattern:
```python
async def run_agent(messages, tools, max_iterations=15):
    iteration = 0
    while iteration < max_iterations:
        response = await llm.chat.completions.create(messages=messages, tools=tools)

        if no tool_calls in response:
            return response.content  # Agent is done

        # Execute tool calls
        for tool_call in response.tool_calls:
            result = await execute_tool(tool_call)
            messages.append(tool_result_message)

        iteration += 1

    # Max iterations reached
    return "I apologize, but I need more time to process this request..."
```

#### Task 4: Error Handling Strategies
**Objective**: Define error handling patterns for LLM and tool failures

**Questions to Answer**:
- How to handle LLM API timeouts and rate limits?
- How to handle tool execution failures (TaskNotFoundError, etc.)?
- What user-friendly error messages to return?
- Should we retry failed operations?

**Expected Output**: Error handling patterns:
```python
try:
    response = await client.chat.completions.create(...)
except openai.APIError as e:
    logger.error(f"LLM API error: {e}")
    return "I'm having trouble connecting to my AI service. Please try again."
except openai.RateLimitError:
    return "I'm currently experiencing high demand. Please try again in a moment."
```

#### Task 5: Message Format Conversion
**Objective**: Define conversion from database message format to OpenAI format

**Questions to Answer**:
- What is the database message format (from Spec 3)?
- What is the OpenAI chat completion message format?
- How to handle system messages vs user/assistant messages?
- How to inject system prompt?

**Expected Output**: Conversion function:
```python
def convert_messages(db_messages: List[Message], system_prompt: str) -> List[dict]:
    openai_messages = [{"role": "system", "content": system_prompt}]
    for msg in db_messages:
        openai_messages.append({
            "role": msg.role,  # "user" or "assistant"
            "content": msg.content
        })
    return openai_messages
```

### Research Deliverable

Create `specs/005-ai-agent-service/research.md` with:
- Gemini API configuration details
- Tool binding conversion patterns
- Agent loop implementation approach
- Error handling strategies
- Message format conversion logic
- Dependencies to add to requirements.txt

## Phase 1: Design & Contracts

### Data Model Design

Create `specs/005-ai-agent-service/data-model.md`:

#### Entity: AgentConfiguration
**Purpose**: Encapsulates LLM client configuration and agent behavior settings

**Attributes**:
- `api_base_url`: str - Gemini API endpoint
- `api_key`: str - API authentication key (from environment)
- `model_name`: str - Gemini model identifier (e.g., "gemini-1.5-pro")
- `temperature`: float - Response randomness (0.0-1.0, default 0.7)
- `max_tokens`: int - Maximum response length (default 1000)
- `timeout`: int - API request timeout in seconds (default 30)

**Validation Rules**:
- api_key must not be empty
- temperature must be between 0.0 and 1.0
- max_tokens must be positive
- timeout must be positive

#### Entity: SystemPrompt
**Purpose**: Defines agent behavior, constraints, and personality

**Attributes**:
- `role_definition`: str - Agent's primary role ("You are a helpful Todo Assistant")
- `operational_rules`: List[str] - Behavior constraints (user isolation, time checking, etc.)
- `response_guidelines`: List[str] - Tone and format instructions (concise, friendly)
- `tool_usage_instructions`: str - How to use available tools

**Validation Rules**:
- role_definition must not be empty
- operational_rules must include user isolation rule
- response_guidelines must include conciseness rule

#### Entity: AgentLoopConfig
**Purpose**: Controls agent loop execution parameters

**Attributes**:
- `max_iterations`: int - Maximum loop iterations (15)
- `iteration_timeout`: int - Timeout per iteration in seconds (30)
- `enable_retry`: bool - Whether to retry failed tool calls (False for MVP)
- `retry_attempts`: int - Number of retry attempts if enabled (1)

**Validation Rules**:
- max_iterations must be between 1 and 50
- iteration_timeout must be positive
- retry_attempts must be non-negative

#### Entity: ToolBinding
**Purpose**: Maps MCP tools to OpenAI function calling format

**Attributes**:
- `tool_name`: str - MCP tool function name
- `openai_function_schema`: dict - OpenAI function calling schema
- `mcp_tool_reference`: callable - Reference to actual MCP tool function

**Validation Rules**:
- tool_name must match MCP tool function name
- openai_function_schema must be valid OpenAI format
- mcp_tool_reference must be callable

### API Contracts

Create `specs/005-ai-agent-service/contracts/agent_service_contract.md`:

#### Function: `run_agent`

**Purpose**: Execute agent loop to process user message with conversation context

**Signature**:
```python
async def run_agent(
    message_history: List[Dict[str, str]],
    user_id: str,
    config: Optional[AgentConfiguration] = None
) -> str
```

**Input Parameters**:
- `message_history`: List of message dictionaries with keys:
  - `role`: str - "user" or "assistant"
  - `content`: str - Message text
- `user_id`: str - Authenticated user's UUID (for tool calls)
- `config`: Optional agent configuration (uses default if None)

**Output**:
- `str`: Final agent response text

**Exceptions**:
- `AgentServiceError`: Base exception for agent service failures
- `LLMAPIError`: LLM API communication failure
- `ToolExecutionError`: Tool execution failure
- `MaxIterationsExceededError`: Agent loop exceeded max iterations

**Behavior**:
1. Validate inputs (message_history not empty, user_id valid format)
2. Convert message_history to OpenAI format
3. Inject system prompt with user_id context
4. Initialize agent loop (iteration counter = 0)
5. While iteration < max_iterations:
   - Call LLM with messages and tool definitions
   - If no tool calls: return final response
   - Execute tool calls with user_id validation
   - Append tool results to messages
   - Increment iteration counter
6. If max iterations reached: return graceful timeout message
7. Handle all errors with user-friendly messages

**Example Usage**:
```python
message_history = [
    {"role": "user", "content": "Show me my tasks"},
    {"role": "assistant", "content": "Let me check your tasks..."},
    {"role": "user", "content": "Create a task to buy groceries"}
]

response = await run_agent(
    message_history=message_history,
    user_id="550e8400-e29b-41d4-a716-446655440000"
)

# response: "I've created a task titled 'Buy groceries' for you. Is there anything else you'd like me to help with?"
```

#### Function: `bind_mcp_tools`

**Purpose**: Convert MCP tool definitions to OpenAI function calling format

**Signature**:
```python
def bind_mcp_tools() -> List[Dict[str, Any]]
```

**Output**:
- List of OpenAI function schemas for all MCP tools

**Behavior**:
1. Import all MCP tools from `mcp_server.tools.task_tools`
2. For each tool, extract:
   - Function name
   - Docstring (for description)
   - Parameter types and descriptions (from Pydantic models)
3. Convert to OpenAI function calling schema
4. Return list of schemas

#### Function: `execute_tool_call`

**Purpose**: Execute a single MCP tool call and return result

**Signature**:
```python
async def execute_tool_call(
    tool_name: str,
    tool_arguments: Dict[str, Any],
    user_id: str
) -> Dict[str, Any]
```

**Input Parameters**:
- `tool_name`: str - Name of MCP tool to execute
- `tool_arguments`: dict - Tool arguments from LLM
- `user_id`: str - User ID to inject into tool call

**Output**:
- dict: Tool execution result

**Exceptions**:
- `ToolNotFoundError`: Tool name not recognized
- `ToolExecutionError`: Tool execution failed

**Behavior**:
1. Validate tool_name exists in MCP tools
2. Inject user_id into tool_arguments (override if present)
3. Call MCP tool with arguments
4. Catch tool-specific exceptions (TaskNotFoundError, etc.)
5. Return result or error message

### Quickstart Guide

Create `specs/005-ai-agent-service/quickstart.md`:

```markdown
# AI Agent Service Quickstart

## Prerequisites

- Python 3.10+
- Existing backend with MCP tools implemented
- Gemini API key

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install openai google-generativeai
```

### 2. Configure Environment

Add to `backend/.env`:
```
GEMINI_API_KEY=your_gemini_api_key_here
```

### 3. Verify MCP Tools

```bash
python -c "from mcp_server.tools.task_tools import add_task, list_tasks; print('MCP tools loaded')"
```

## Testing

### Manual Test with Mock Prompt

Create `backend/test_agent_manual.py`:

```python
import asyncio
from services.agent_service import run_agent

async def test_add_task():
    message_history = [
        {"role": "user", "content": "Create a task to buy groceries"}
    ]

    user_id = "test-user-123"

    response = await run_agent(message_history, user_id)
    print(f"Agent response: {response}")

    # Verify add_task was called by checking response mentions task creation

if __name__ == "__main__":
    asyncio.run(test_add_task())
```

Run test:
```bash
python test_agent_manual.py
```

Expected output:
```
Agent response: I've created a task titled "Buy groceries" for you. Your task has been added to your list.
```

### Automated Tests

Run pytest:
```bash
pytest tests/test_agent_service.py -v
```

## Usage in Chat API

```python
from services.agent_service import run_agent

@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    # Load conversation history from database
    messages = await load_conversation_history(request.user_id)

    # Add new user message
    messages.append({"role": "user", "content": request.message})

    # Run agent
    response = await run_agent(messages, request.user_id)

    # Persist messages to database
    await save_messages(request.user_id, messages, response)

    return {"response": response}
```

## Configuration

Customize agent behavior in `backend/config/agent_config.py`:

```python
AGENT_CONFIG = {
    "max_iterations": 15,
    "temperature": 0.7,
    "model": "gemini-1.5-pro",
    "timeout": 30
}
```

## Troubleshooting

**Issue**: "LLM API error: Invalid API key"
- **Solution**: Verify GEMINI_API_KEY in .env file

**Issue**: "Tool execution failed: User not found"
- **Solution**: Ensure user_id exists in database before testing

**Issue**: "Max iterations exceeded"
- **Solution**: Check if agent is stuck in loop; review system prompt clarity
```

### Agent Context Update

Run agent context update script:
```bash
powershell.exe -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

This will update `.claude/context.md` (or equivalent) with:
- OpenAI SDK (configured for Gemini)
- Google Generative AI library
- Agent service module location
- MCP tool binding patterns

## Phase 2: Task Breakdown

*Task breakdown will be generated by `/sp.tasks` command - NOT part of this plan*

The `/sp.tasks` command will create `specs/005-ai-agent-service/tasks.md` with:
- Detailed implementation tasks
- Test cases for each task
- Dependency ordering
- Acceptance criteria

## Post-Design Constitution Check

### Re-evaluation After Phase 1

- ✅ **Principle I (Spec-first)**: Design artifacts trace back to spec requirements
- ✅ **Principle II (Single responsibility)**: Agent service maintains single responsibility
- ✅ **Principle III (Explicit contracts)**: `agent_service_contract.md` defines all interfaces
- ✅ **Principle IV (Security by default)**: User_id validation in system prompt and tool execution
- ✅ **Principle V (Determinism)**: Agent loop is deterministic given same inputs (LLM temperature controlled)
- ✅ **Principle VI (Agentic discipline)**: All implementation via Claude Code
- ✅ **Principle VII (Stateless AI)**: Agent service has no state; all context passed explicitly
- ✅ **Principle VIII (Conversation persistence)**: Persistence delegated to calling API
- ✅ **Principle IX (User data isolation)**: System prompt enforces isolation; user_id injected in all tool calls

**Status**: ✅ PASSED - Design maintains constitutional compliance

## Dependencies & Integration Points

### Internal Dependencies
- **MCP Server** (`backend/mcp_server/`): Agent service calls MCP tools
- **Database Session** (`backend/database/session.py`): MCP tools use async sessions
- **Task Models** (`backend/models/task_models.py`): MCP tools operate on Task entities
- **User Models** (`backend/models/task_models.py`): User validation in tools

### External Dependencies
- **OpenAI Python SDK**: LLM client library (configured for Gemini)
- **Google Generative AI**: Gemini API access (if direct SDK needed)
- **Gemini API**: LLM service provider

### Integration Points
- **Chat API Endpoint** (Future - Spec 6): Will call `run_agent()` function
- **Conversation Database** (Existing - Spec 3): Provides message history to agent
- **MCP Tools** (Existing - Spec 4): Agent executes tools during loop

## Risk Analysis

### Risk 1: Gemini API OpenAI Compatibility
**Likelihood**: Medium
**Impact**: High
**Mitigation**: Research Phase 0 will validate compatibility; fallback to direct Gemini SDK if needed

### Risk 2: Tool Binding Complexity
**Likelihood**: Low
**Impact**: Medium
**Mitigation**: MCP tools already have Pydantic schemas; conversion is straightforward

### Risk 3: Agent Loop Infinite Iterations
**Likelihood**: Low
**Impact**: Medium
**Mitigation**: Hard limit of 15 iterations; graceful timeout message

### Risk 4: LLM API Rate Limits
**Likelihood**: Medium
**Impact**: Medium
**Mitigation**: Error handling returns user-friendly message; future: implement retry with backoff

### Risk 5: Tool Execution Failures
**Likelihood**: Medium
**Impact**: Low
**Mitigation**: Agent catches tool errors and explains to user in natural language

## Success Metrics

- ✅ Agent successfully calls MCP tools in response to user requests
- ✅ Agent maintains conversation context across multiple turns
- ✅ Agent enforces user isolation (never accesses other users' tasks)
- ✅ Agent handles errors gracefully with user-friendly messages
- ✅ Agent completes within 15 iterations for typical requests
- ✅ Response time <3 seconds for 95% of requests (excluding LLM latency)

## Next Steps

1. **Complete Phase 0**: Execute research tasks and create `research.md`
2. **Complete Phase 1**: Create `data-model.md`, `contracts/`, and `quickstart.md`
3. **Run `/sp.tasks`**: Generate detailed task breakdown in `tasks.md`
4. **Implementation**: Execute tasks via Claude Code following agentic workflow
5. **Testing**: Validate with manual and automated tests
6. **Integration**: Connect to chat API endpoint (future spec)

---

**Plan Status**: ✅ COMPLETE - Ready for Phase 0 research execution
**Next Command**: Begin Phase 0 research or proceed to `/sp.tasks` if research is complete
