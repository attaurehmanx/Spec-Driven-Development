"""
Chat API Request/Response Models

This module defines Pydantic models for the chat endpoint request and response payloads.
These models provide validation, serialization, and API documentation.
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, List


class ChatRequest(BaseModel):
    """
    Request payload for the chat endpoint.

    Attributes:
        message: User's message content (required, 1-10,000 characters)
        conversation_id: Optional ID of existing conversation to continue
    """
    message: str = Field(
        ...,
        min_length=1,
        max_length=10000,
        description="User's message content"
    )
    conversation_id: Optional[int] = Field(
        None,
        gt=0,
        description="Optional conversation ID to continue existing conversation"
    )

    @validator('message')
    def message_not_empty(cls, v):
        """Validate that message is not just whitespace."""
        if not v or not v.strip():
            raise ValueError('message cannot be empty or whitespace only')
        return v.strip()

    class Config:
        schema_extra = {
            "example": {
                "message": "Create a task to buy milk",
                "conversation_id": 123
            }
        }


class ChatResponse(BaseModel):
    """
    Response payload from the chat endpoint.

    Attributes:
        conversation_id: ID of the conversation (new or existing)
        response: AI agent's natural language response
        tool_calls: List of tool names that were called by the agent
    """
    conversation_id: int = Field(
        ...,
        gt=0,
        description="ID of the conversation"
    )
    response: str = Field(
        ...,
        min_length=1,
        description="AI agent's response text"
    )
    tool_calls: List[str] = Field(
        default_factory=list,
        description="Names of tools called by the agent during processing"
    )

    class Config:
        schema_extra = {
            "example": {
                "conversation_id": 123,
                "response": "I've created a task titled 'Buy milk' for you.",
                "tool_calls": ["add_task"]
            }
        }
