"""
Agent Configuration Module

This module defines the configuration for the AI agent service including:
- AgentConfiguration: LLM client settings and behavior parameters
- System prompt template with user isolation rules
"""

import os
from typing import Optional
from pydantic import BaseModel, Field, field_validator


class AgentConfiguration(BaseModel):
    """Configuration for the AI agent service.

    Encapsulates LLM client configuration and agent behavior settings.
    All settings can be overridden via environment variables.
    """

    api_key: str = Field(
        default_factory=lambda: os.getenv("GEMINI_API_KEY", ""),
        description="Gemini API authentication key"
    )

    base_url: str = Field(
        default_factory=lambda: os.getenv(
            "GEMINI_BASE_URL",
            "https://generativelanguage.googleapis.com/v1beta/openai/"
        ),
        description="Gemini API endpoint (OpenAI-compatible)"
    )

    model_name: str = Field(
        default_factory=lambda: os.getenv("GEMINI_MODEL", "gemini-2.5-flash"),
        description="Gemini model identifier"
    )

    temperature: float = Field(
        default_factory=lambda: float(os.getenv("AGENT_TEMPERATURE", "1.0")),
        description="Response randomness (0.0-1.0)"
    )

    max_tokens: int = Field(
        default_factory=lambda: int(os.getenv("AGENT_MAX_TOKENS", "1000")),
        description="Maximum response length in tokens"
    )

    timeout: int = Field(
        default_factory=lambda: int(os.getenv("AGENT_TIMEOUT", "30")),
        description="API request timeout in seconds"
    )

    max_iterations: int = Field(
        default_factory=lambda: int(os.getenv("AGENT_MAX_ITERATIONS", "15")),
        description="Maximum agent loop iterations"
    )

    @field_validator('api_key')
    @classmethod
    def api_key_not_empty(cls, v):
        """Validate API key is not empty."""
        if not v or v == "your_gemini_api_key_here":
            raise ValueError(
                "GEMINI_API_KEY must be set in environment variables. "
                "Get your API key from https://aistudio.google.com/apikey"
            )
        return v

    @field_validator('temperature')
    @classmethod
    def temperature_in_range(cls, v):
        """Validate temperature is between 0.0 and 1.0."""
        if not 0.0 <= v <= 1.0:
            raise ValueError("Temperature must be between 0.0 and 1.0")
        return v

    @field_validator('max_tokens')
    @classmethod
    def max_tokens_positive(cls, v):
        """Validate max_tokens is positive."""
        if v <= 0:
            raise ValueError("max_tokens must be positive")
        return v

    @field_validator('timeout')
    @classmethod
    def timeout_positive(cls, v):
        """Validate timeout is positive."""
        if v <= 0:
            raise ValueError("timeout must be positive")
        return v

    @field_validator('max_iterations')
    @classmethod
    def max_iterations_in_range(cls, v):
        """Validate max_iterations is between 1 and 50."""
        if not 1 <= v <= 50:
            raise ValueError("max_iterations must be between 1 and 50")
        return v


# System Prompt Template
SYSTEM_PROMPT_TEMPLATE = """You are a helpful Todo Assistant for user {user_id}.

CRITICAL RULES:
1. You can ONLY manage tasks for user {user_id}
2. NEVER access or modify tasks for other users
3. Always include user_id={user_id} in all tool calls
4. If asked about time-relative queries (today, tomorrow), check current time first
5. Keep responses concise and friendly
6. If a tool fails, explain the error in user-friendly language

CONVERSATION CONTEXT:
- Pay attention to the conversation history
- Use context to resolve pronouns like "it", "that task", "the first one"
- When user says "mark it as done", refer to the most recently mentioned task
- When user says "the first one", refer to the first task in the most recent list
- If context is ambiguous, ask clarifying questions before taking action

CLARIFYING QUESTIONS:
- If a request is ambiguous (e.g., "delete the task" when multiple tasks exist), ask which one
- If required information is missing (e.g., task title not provided), ask for it
- Keep clarifying questions brief and specific

TIME AWARENESS:
- Current time: {current_time}
- "today" means tasks with today's date ({current_date})
- "tomorrow" means tasks with tomorrow's date ({tomorrow_date})
- "this week" means tasks within the next 7 days
- Always check the current time when interpreting time-relative queries

Available tools:
- add_task: Create new tasks
- list_tasks: View user's tasks (with optional status filter: all, pending, completed)
- complete_task: Mark tasks as done
- update_task: Modify task details (title and/or description)
- delete_task: Remove tasks

Remember: You can only help with task management. For other requests, politely explain your limitations."""


def get_system_prompt(user_id: str, current_time: Optional[str] = None) -> str:
    """Generate system prompt with user_id and current time injected.

    Args:
        user_id: Authenticated user's UUID
        current_time: Current time in ISO 8601 format (optional)

    Returns:
        Formatted system prompt string
    """
    from datetime import datetime, timedelta

    if current_time is None:
        current_time = datetime.utcnow().isoformat() + "Z"

    # Parse current time to extract date information
    try:
        dt = datetime.fromisoformat(current_time.replace('Z', '+00:00'))
    except:
        dt = datetime.utcnow()

    current_date = dt.strftime("%Y-%m-%d")
    tomorrow_date = (dt + timedelta(days=1)).strftime("%Y-%m-%d")

    return SYSTEM_PROMPT_TEMPLATE.format(
        user_id=user_id,
        current_time=current_time,
        current_date=current_date,
        tomorrow_date=tomorrow_date
    )
