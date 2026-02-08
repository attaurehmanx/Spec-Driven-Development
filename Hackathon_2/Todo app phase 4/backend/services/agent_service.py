"""
AI Agent Service Module

This module implements the AI agent service that orchestrates conversational
task management using Google Gemini API (via OpenAI-compatible interface)
and MCP tools.

Core Functions:
- run_agent(): Execute agent loop to process user messages
- bind_mcp_tools(): Convert MCP tools to OpenAI function schemas
- execute_tool_call(): Execute individual tool calls with user_id injection

Classes:
- AgentResponse: Structured response from agent execution
- AgentServiceError: Base exception for agent service failures
- LLMAPIError: LLM API communication failure
- ToolExecutionError: Tool execution failure
- MaxIterationsExceededError: Agent loop exceeded maximum iterations
"""

import logging
import json
import inspect
import time
from typing import List, Dict, Any, Optional, get_type_hints, get_origin, get_args, Literal
from datetime import datetime
from dataclasses import dataclass, field

# Import OpenAI SDK
from openai import AsyncOpenAI, APIError, RateLimitError, APITimeoutError

# Import agent configuration
from config.agent_config import AgentConfiguration, get_system_prompt

# Import MCP tools
from mcp_server.tools.task_tools import (
    add_task,
    list_tasks,
    complete_task,
    update_task,
    delete_task,
    TaskNotFoundError,
    UnauthorizedTaskAccessError,
    ValidationError as TaskValidationError,
    TaskToolError
)

logger = logging.getLogger(__name__)


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class AgentResponse:
    """Structured response from agent execution.

    Attributes:
        status: Execution status - "completed", "max_iterations_reached", or "error"
        final_response: Final natural language response from agent
        messages: Complete message history including tool calls
        iterations: Number of agent loop iterations executed
        finish_reason: Reason for completion - "completed", "max_iterations", or "error"
        error: Error message if status is "error", None otherwise
        warning: Warning message if applicable, None otherwise
        execution_time_ms: Total execution time in milliseconds
    """
    status: Literal["completed", "max_iterations_reached", "error"]
    final_response: str
    messages: List[Dict[str, Any]] = field(default_factory=list)
    iterations: int = 0
    finish_reason: Literal["completed", "max_iterations", "error"] = "completed"
    error: Optional[str] = None
    warning: Optional[str] = None
    execution_time_ms: float = 0.0


# ============================================================================
# Custom Exception Classes
# ============================================================================

class AgentServiceError(Exception):
    """Base exception for agent service failures."""
    pass


class LLMAPIError(AgentServiceError):
    """LLM API communication failure."""
    pass


class ToolExecutionError(AgentServiceError):
    """Tool execution failure."""
    pass


class MaxIterationsExceededError(AgentServiceError):
    """Agent loop exceeded maximum iterations."""
    pass


# ============================================================================
# Tool Registry
# ============================================================================

# Map tool names to their callable functions
TOOL_REGISTRY = {
    "add_task": add_task,
    "list_tasks": list_tasks,
    "complete_task": complete_task,
    "update_task": update_task,
    "delete_task": delete_task
}


# ============================================================================
# Helper Functions
# ============================================================================

def python_type_to_json_schema(python_type: type) -> Dict[str, Any]:
    """Convert Python type hint to JSON Schema type.

    Transforms Python type annotations into JSON Schema type definitions
    for OpenAI function calling format. Handles Optional types, Union types,
    and basic Python types.

    Args:
        python_type: Python type annotation (e.g., str, int, Optional[str])

    Returns:
        Dict containing JSON Schema type definition with 'type' key

    Examples:
        >>> python_type_to_json_schema(str)
        {'type': 'string'}
        >>> python_type_to_json_schema(Optional[int])
        {'type': 'integer'}
    """
    # Handle Optional types
    origin = get_origin(python_type)
    if origin is type(None) or python_type is type(None):
        return {"type": "null"}

    # Handle Union types (including Optional)
    if origin is type(Optional[str]):  # Union type
        args = get_args(python_type)
        # Filter out NoneType
        non_none_types = [arg for arg in args if arg is not type(None)]
        if len(non_none_types) == 1:
            return python_type_to_json_schema(non_none_types[0])
        # Multiple non-None types - use first one (simplified)
        return python_type_to_json_schema(non_none_types[0])

    # Handle basic types
    type_mapping = {
        str: "string",
        int: "integer",
        float: "number",
        bool: "boolean",
        list: "array",
        dict: "object"
    }

    return {"type": type_mapping.get(python_type, "string")}


def extract_param_description(docstring: str, param_name: str) -> str:
    """Extract parameter description from function docstring.

    Parses Google-style docstrings to extract parameter descriptions
    from the Args section.

    Args:
        docstring: Function docstring in Google style format
        param_name: Name of parameter to extract description for

    Returns:
        Parameter description string, or empty string if not found

    Examples:
        >>> docstring = '''
        ... Args:
        ...     user_id: The user's UUID
        ...     title: Task title
        ... '''
        >>> extract_param_description(docstring, "user_id")
        "The user's UUID"
    """
    if not docstring:
        return ""

    lines = docstring.split('\n')
    in_args_section = False

    for line in lines:
        line = line.strip()

        # Detect Args section
        if line.startswith("Args:"):
            in_args_section = True
            continue

        # Exit Args section
        if in_args_section and line.endswith(":") and not line.startswith(param_name):
            break

        # Extract parameter description
        if in_args_section and line.startswith(f"{param_name}:"):
            description = line[len(param_name) + 1:].strip()
            return description

    return ""


def bind_mcp_tools() -> List[Dict[str, Any]]:
    """Convert MCP tool definitions to OpenAI function calling format.

    Discovers all MCP tools from the tool registry and converts their
    signatures to OpenAI function schemas. Extracts function names,
    descriptions, parameter types, and required parameters from Python
    function signatures and docstrings.

    The conversion process:
    1. Iterates through all tools in TOOL_REGISTRY
    2. Extracts function signature and type hints
    3. Parses docstring for descriptions
    4. Converts Python types to JSON Schema types
    5. Builds OpenAI function calling schema

    Returns:
        List of OpenAI function schemas, each containing:
            - type: "function"
            - function: Dict with name, description, and parameters schema

    Raises:
        ValueError: If tool definition is invalid or cannot be converted

    Examples:
        >>> tools = bind_mcp_tools()
        >>> len(tools)
        5
        >>> tools[0]['function']['name']
        'add_task'
    """
    logger.info("Binding MCP tools to OpenAI function schemas")

    tools = []

    for tool_name, tool_func in TOOL_REGISTRY.items():
        try:
            # Get function signature and type hints
            sig = inspect.signature(tool_func)
            type_hints = get_type_hints(tool_func)
            docstring = inspect.getdoc(tool_func) or ""

            # Extract description (first line of docstring)
            description = docstring.split('\n')[0] if docstring else f"Execute {tool_name}"

            # Build parameters schema
            properties = {}
            required = []

            for param_name, param in sig.parameters.items():
                if param_name == 'self':
                    continue

                # Get type
                param_type = type_hints.get(param_name, str)
                json_type = python_type_to_json_schema(param_type)

                # Extract description from docstring
                param_description = extract_param_description(docstring, param_name)
                if param_description:
                    json_type["description"] = param_description

                properties[param_name] = json_type

                # Check if required (no default value)
                if param.default == inspect.Parameter.empty:
                    required.append(param_name)

            # Build OpenAI function schema
            tool_schema = {
                "type": "function",
                "function": {
                    "name": tool_name,
                    "description": description,
                    "parameters": {
                        "type": "object",
                        "properties": properties,
                        "required": required,
                        "additionalProperties": False
                    }
                }
            }

            tools.append(tool_schema)
            logger.debug(f"Bound tool: {tool_name}")

        except Exception as e:
            logger.error(f"Failed to bind tool {tool_name}: {e}")
            raise ValueError(f"Invalid tool definition for {tool_name}: {e}")

    logger.info(f"Successfully bound {len(tools)} MCP tools")
    return tools


async def execute_tool_call(
    tool_name: str,
    tool_arguments: Dict[str, Any],
    user_id: str
) -> Dict[str, Any]:
    """Execute a single MCP tool call and return result.

    Executes the specified MCP tool with the provided arguments, automatically
    injecting the user_id for security and user isolation. Handles all tool
    exceptions and converts them to user-friendly error messages.

    Security Features:
    - Automatically injects user_id into tool arguments (overrides if present)
    - Validates tool exists before execution
    - Catches and sanitizes all exceptions

    Error Handling:
    - TaskNotFoundError: Returns user-friendly "task not found" message
    - UnauthorizedTaskAccessError: Returns permission denied message
    - ValidationError: Returns validation error details
    - TaskToolError: Returns tool-specific error message
    - Exception: Returns generic error message

    Args:
        tool_name: Name of MCP tool to execute (must exist in TOOL_REGISTRY)
        tool_arguments: Tool arguments from LLM (dict with parameter names and values)
        user_id: User ID to inject into tool call for security and user isolation

    Returns:
        Dict containing execution result:
            Success: {"status": "success", "result": {...}}
            Error: {"status": "error", "error": str, "error_type": str, "message": str}

    Examples:
        >>> result = await execute_tool_call(
        ...     "add_task",
        ...     {"title": "Buy groceries"},
        ...     "user-123"
        ... )
        >>> result["status"]
        'success'
        >>> result["result"]["task_id"]
        1
    """
    logger.info(f"Executing tool: {tool_name} for user {user_id}")

    try:
        # Validate tool exists
        if tool_name not in TOOL_REGISTRY:
            error_msg = f"Unknown tool: {tool_name}"
            logger.error(error_msg)
            return {
                "status": "error",
                "error": "Tool not found",
                "error_type": "ToolNotFoundError",
                "message": f"I don't have access to the '{tool_name}' capability."
            }

        # Inject user_id into tool arguments (security - override if present)
        tool_arguments["user_id"] = user_id

        # Get tool function
        tool_func = TOOL_REGISTRY[tool_name]

        # Execute tool
        result = await tool_func(**tool_arguments)

        # Return success response
        return {
            "status": "success",
            "result": result
        }

    except TaskNotFoundError as e:
        logger.warning(f"Task not found in {tool_name}: {e}")
        return {
            "status": "error",
            "error": "Task not found",
            "error_type": "TaskNotFoundError",
            "message": "I couldn't find that task. It may have been deleted."
        }

    except UnauthorizedTaskAccessError as e:
        logger.warning(f"Unauthorized access in {tool_name}: {e}")
        return {
            "status": "error",
            "error": "Unauthorized access",
            "error_type": "UnauthorizedTaskAccessError",
            "message": "You don't have permission to access that task."
        }

    except (TaskValidationError, ValueError) as e:
        logger.warning(f"Validation error in {tool_name}: {e}")
        return {
            "status": "error",
            "error": "Validation error",
            "error_type": "ValidationError",
            "message": f"I couldn't process that request: {str(e)}"
        }

    except TaskToolError as e:
        logger.error(f"Tool error in {tool_name}: {e}")
        return {
            "status": "error",
            "error": "Tool execution failed",
            "error_type": "TaskToolError",
            "message": f"I encountered an error while executing that action: {str(e)}"
        }

    except Exception as e:
        logger.error(f"Unexpected error in {tool_name}: {e}", exc_info=True)
        return {
            "status": "error",
            "error": "Unexpected error",
            "error_type": type(e).__name__,
            "message": "An unexpected error occurred. Please try again or contact support."
        }


def convert_message_history_to_openai(
    message_history: List[Dict[str, str]],
    user_id: str
) -> List[Dict[str, Any]]:
    """Convert message history to OpenAI chat completion format.

    Transforms user-provided message history into OpenAI's chat completion
    format by injecting the system prompt at the beginning. The system prompt
    includes user_id, current time, and agent behavior instructions.

    Validation:
    - Ensures message_history is not empty
    - Validates each message has 'role' and 'content' keys
    - Validates role is either 'user' or 'assistant'

    Args:
        message_history: List of message dicts with 'role' and 'content' keys.
                        Role must be 'user' or 'assistant'.
        user_id: Authenticated user's UUID for system prompt injection

    Returns:
        List of messages in OpenAI format with system prompt as first message

    Raises:
        ValueError: If message history is empty or has invalid structure

    Examples:
        >>> history = [{"role": "user", "content": "Hello"}]
        >>> messages = convert_message_history_to_openai(history, "user-123")
        >>> messages[0]["role"]
        'system'
        >>> messages[1]["role"]
        'user'
    """
    logger.debug(f"Converting {len(message_history)} messages to OpenAI format")

    # Validate message history
    if not message_history:
        raise ValueError("message_history cannot be empty")

    for msg in message_history:
        if "role" not in msg or "content" not in msg:
            raise ValueError("Each message must have 'role' and 'content' keys")
        if msg["role"] not in ["user", "assistant"]:
            raise ValueError(f"Invalid role: {msg['role']}. Must be 'user' or 'assistant'")

    # Start with system prompt
    current_time = datetime.utcnow().isoformat() + "Z"
    system_prompt = get_system_prompt(user_id, current_time)

    openai_messages = [
        {"role": "system", "content": system_prompt}
    ]

    # Add message history
    for msg in message_history:
        openai_messages.append({
            "role": msg["role"],
            "content": msg["content"]
        })

    logger.debug(f"Converted to {len(openai_messages)} OpenAI messages (including system prompt)")
    return openai_messages


# ============================================================================
# Main Agent Function
# ============================================================================

async def run_agent(
    message_history: List[Dict[str, str]],
    user_id: str,
    config: Optional[AgentConfiguration] = None
) -> AgentResponse:
    """Execute agent loop to process user message with conversation context.

    Implements an iterative agent loop that:
    1. Calls LLM with message history and available tools
    2. Detects and executes tool calls
    3. Feeds tool results back to LLM
    4. Continues until LLM provides final response or max iterations reached

    The agent maintains conversation context across multiple turns, resolves
    pronouns and references, handles errors gracefully, and interprets
    time-relative queries.

    Args:
        message_history: List of previous conversation messages with 'role' and 'content' keys.
                        Role must be 'user' or 'assistant'.
        user_id: Authenticated user's UUID for security and user isolation
        config: Optional agent configuration. If None, uses default from environment variables.

    Returns:
        AgentResponse object containing:
            - status: "completed", "max_iterations_reached", or "error"
            - final_response: Natural language response from agent
            - messages: Complete message history including tool calls
            - iterations: Number of agent loop iterations
            - finish_reason: Reason for completion
            - error: Error message if applicable
            - warning: Warning message if applicable
            - execution_time_ms: Total execution time in milliseconds

    Raises:
        ValueError: If message_history is empty or has invalid structure
        LLMAPIError: If LLM API communication fails (rate limits, timeouts, API errors)
        MaxIterationsExceededError: If agent loop exceeds maximum iterations without completion

    Examples:
        >>> message_history = [{"role": "user", "content": "Show me my tasks"}]
        >>> response = await run_agent(message_history, "user-123")
        >>> print(response.final_response)
        "You have 3 tasks: ..."
        >>> print(f"Completed in {response.iterations} iterations")
        Completed in 2 iterations
    """
    # T048: Start performance logging
    start_time = time.time()
    logger.info(f"run_agent called for user {user_id} with {len(message_history)} messages")

    # T010: Input validation
    if not message_history:
        error_msg = "message_history cannot be empty"
        logger.error(error_msg)
        execution_time = (time.time() - start_time) * 1000
        return AgentResponse(
            status="error",
            final_response="",
            messages=[],
            iterations=0,
            finish_reason="error",
            error=error_msg,
            execution_time_ms=execution_time
        )

    if not user_id:
        error_msg = "user_id cannot be empty"
        logger.error(error_msg)
        execution_time = (time.time() - start_time) * 1000
        return AgentResponse(
            status="error",
            final_response="",
            messages=[],
            iterations=0,
            finish_reason="error",
            error=error_msg,
            execution_time_ms=execution_time
        )

    # Validate message structure
    for msg in message_history:
        if "role" not in msg or "content" not in msg:
            error_msg = "Each message must have 'role' and 'content' keys"
            logger.error(error_msg)
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response="",
                messages=[],
                iterations=0,
                finish_reason="error",
                error=error_msg,
                execution_time_ms=execution_time
            )
        if msg["role"] not in ["user", "assistant"]:
            error_msg = f"Invalid role: {msg['role']}. Must be 'user' or 'assistant'"
            logger.error(error_msg)
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response="",
                messages=[],
                iterations=0,
                finish_reason="error",
                error=error_msg,
                execution_time_ms=execution_time
            )

    # T011: Initialize configuration and OpenAI client
    if config is None:
        try:
            config = AgentConfiguration()
        except ValueError as e:
            error_msg = f"Agent configuration failed: {e}"
            logger.error(f"Configuration error: {e}")
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response="I'm having trouble with my configuration. Please contact support.",
                messages=[],
                iterations=0,
                finish_reason="error",
                error=error_msg,
                execution_time_ms=execution_time
            )

    # Initialize Gemini client using OpenAI Agents SDK pattern
    # Debug: Log API key status
    api_key_preview = config.api_key[:10] + "..." if config.api_key else "NONE"
    logger.info(f"Initializing OpenAI client with API key: {api_key_preview}")
    logger.info(f"Using model: {config.model_name}")
    logger.info(f"Base URL: https://generativelanguage.googleapis.com/v1beta/openai/")

    client = AsyncOpenAI(
        api_key=config.api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )

    # Convert message history to OpenAI format
    try:
        messages = convert_message_history_to_openai(message_history, user_id)
    except ValueError as e:
        error_msg = f"Invalid message history: {e}"
        logger.error(f"Message conversion error: {e}")
        execution_time = (time.time() - start_time) * 1000
        return AgentResponse(
            status="error",
            final_response="",
            messages=[],
            iterations=0,
            finish_reason="error",
            error=error_msg,
            execution_time_ms=execution_time
        )

    # Bind MCP tools
    try:
        tools = bind_mcp_tools()
    except Exception as e:
        error_msg = f"Failed to bind tools: {e}"
        logger.error(f"Tool binding error: {e}")
        execution_time = (time.time() - start_time) * 1000
        return AgentResponse(
            status="error",
            final_response="I'm having trouble accessing my tools. Please try again later.",
            messages=[],
            iterations=0,
            finish_reason="error",
            error=error_msg,
            execution_time_ms=execution_time
        )

    # T012-T016: Agent loop
    iteration_count = 0
    max_iterations = config.max_iterations

    logger.info(f"Starting agent loop (max {max_iterations} iterations)")

    while iteration_count < max_iterations:
        iteration_count += 1
        logger.info(f"Agent iteration {iteration_count}/{max_iterations}")

        try:
            # T012: Call LLM with messages and tools
            response = await client.chat.completions.create(
                model=config.model_name,
                messages=messages,
                tools=tools,
                tool_choice="auto",
                temperature=config.temperature,
                max_tokens=config.max_tokens
            )

            assistant_message = response.choices[0].message

            # T015: Check termination - no tool calls means agent is done
            if not assistant_message.tool_calls:
                final_response = assistant_message.content or ""
                execution_time = (time.time() - start_time) * 1000
                logger.info(f"Agent completed in {iteration_count} iterations ({execution_time:.2f}ms)")

                return AgentResponse(
                    status="completed",
                    final_response=final_response,
                    messages=messages,
                    iterations=iteration_count,
                    finish_reason="completed",
                    error=None,
                    warning=None,
                    execution_time_ms=execution_time
                )

            # T013: Tool call detection - agent wants to use tools
            logger.info(f"Agent requested {len(assistant_message.tool_calls)} tool calls")

            # Add assistant message with tool calls to history
            messages.append({
                "role": "assistant",
                "content": assistant_message.content,
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {
                            "name": tc.function.name,
                            "arguments": tc.function.arguments
                        }
                    }
                    for tc in assistant_message.tool_calls
                ]
            })

            # T013: Execute tool calls
            for tool_call in assistant_message.tool_calls:
                tool_name = tool_call.function.name
                tool_arguments = json.loads(tool_call.function.arguments)

                logger.info(f"Executing tool: {tool_name}")

                # T008: Execute tool with user_id injection
                tool_result = await execute_tool_call(
                    tool_name=tool_name,
                    tool_arguments=tool_arguments,
                    user_id=user_id
                )

                # T014: Format tool result and append to messages
                tool_result_content = json.dumps(tool_result)

                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": tool_result_content
                })

                logger.debug(f"Tool {tool_name} result: {tool_result.get('status', 'unknown')}")

        except RateLimitError as e:
            # T017: Handle rate limit errors
            error_msg = "I'm currently experiencing high demand. Please try again in a moment."
            logger.error(f"Rate limit error: {e}")
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response=error_msg,
                messages=messages,
                iterations=iteration_count,
                finish_reason="error",
                error=str(e),
                warning=None,
                execution_time_ms=execution_time
            )

        except APITimeoutError as e:
            # T017: Handle timeout errors
            error_msg = "That request took too long. Please try a simpler query."
            logger.error(f"API timeout error: {e}")
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response=error_msg,
                messages=messages,
                iterations=iteration_count,
                finish_reason="error",
                error=str(e),
                warning=None,
                execution_time_ms=execution_time
            )

        except APIError as e:
            # T017: Handle general API errors
            error_msg = "I'm having trouble connecting to my AI service. Please try again."
            logger.error(f"LLM API error: {e}")
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response=error_msg,
                messages=messages,
                iterations=iteration_count,
                finish_reason="error",
                error=str(e),
                warning=None,
                execution_time_ms=execution_time
            )

        except Exception as e:
            # T017: Handle unexpected errors
            error_msg = "An unexpected error occurred. Please try again or contact support."
            logger.error(f"Unexpected error in agent loop: {e}", exc_info=True)
            execution_time = (time.time() - start_time) * 1000
            return AgentResponse(
                status="error",
                final_response=error_msg,
                messages=messages,
                iterations=iteration_count,
                finish_reason="error",
                error=str(e),
                warning=None,
                execution_time_ms=execution_time
            )

    # T016: Max iterations reached - graceful exit
    logger.warning(f"Max iterations ({max_iterations}) reached")

    # Request final summary from LLM
    try:
        messages.append({
            "role": "system",
            "content": (
                "You have reached the maximum number of iterations. "
                "Please provide a brief summary of what you've accomplished so far."
            )
        })

        final_response_obj = await client.chat.completions.create(
            model=config.model_name,
            messages=messages,
            temperature=config.temperature,
            max_tokens=config.max_tokens
        )

        summary = final_response_obj.choices[0].message.content or ""
        final_response = f"{summary}\n\nNote: I need more time to fully process this request. Please try breaking it into smaller steps."

        execution_time = (time.time() - start_time) * 1000
        logger.info(f"Agent reached max iterations after {execution_time:.2f}ms")

        return AgentResponse(
            status="max_iterations_reached",
            final_response=final_response,
            messages=messages,
            iterations=iteration_count,
            finish_reason="max_iterations",
            error=None,
            warning="Agent reached maximum iterations before natural completion",
            execution_time_ms=execution_time
        )

    except Exception as e:
        error_msg = "I need more time to process this request. Please try breaking it into smaller steps."
        logger.error(f"Failed to get final summary: {e}")
        execution_time = (time.time() - start_time) * 1000
        return AgentResponse(
            status="max_iterations_reached",
            final_response=error_msg,
            messages=messages,
            iterations=iteration_count,
            finish_reason="max_iterations",
            error=str(e),
            warning="Agent reached maximum iterations and failed to generate summary",
            execution_time_ms=execution_time
        )
