"""
Conversation Service Module

This module provides business logic functions for conversation management
and message processing in the chat endpoint.

Core Functions:
- get_or_create_conversation(): Load existing or create new conversation
- load_conversation_messages(): Retrieve all messages for a conversation
- convert_messages_to_agent_format(): Convert Message objects to agent format
- extract_tool_calls_from_response(): Extract tool names from agent response
"""

import logging
from typing import Optional, List, Dict, Any
from sqlmodel import Session

from models.conversation import Conversation
from models.message import Message
from services.agent_service import AgentResponse

logger = logging.getLogger(__name__)


def get_or_create_conversation(
    session: Session,
    user_id: str,
    conversation_id: Optional[int] = None
) -> Conversation:
    """
    Get an existing conversation or create a new one.

    If conversation_id is provided, loads the existing conversation and validates
    that it belongs to the authenticated user. If conversation_id is not provided,
    creates a new conversation for the user.

    Args:
        session: Database session
        user_id: Authenticated user's ID
        conversation_id: Optional ID of existing conversation

    Returns:
        Conversation object (existing or newly created)

    Raises:
        ValueError: If conversation_id is provided but conversation not found
                   or doesn't belong to the user
    """
    if conversation_id is not None:
        # Load existing conversation with ownership validation
        logger.info(f"Loading existing conversation {conversation_id} for user {user_id}")
        conversation = Conversation.get_conversation_by_id(
            session=session,
            conversation_id=conversation_id,
            user_id=user_id
        )

        if conversation is None:
            logger.warning(f"Conversation {conversation_id} not found or not owned by user {user_id}")
            raise ValueError(f"Conversation {conversation_id} not found or access denied")

        logger.info(f"Successfully loaded conversation {conversation_id}")
        return conversation
    else:
        # Create new conversation
        logger.info(f"Creating new conversation for user {user_id}")
        conversation = Conversation.create_conversation(
            session=session,
            user_id=user_id,
            title=None  # Title can be set later based on first message
        )
        logger.info(f"Created new conversation {conversation.id}")
        return conversation


def load_conversation_messages(
    session: Session,
    conversation_id: int
) -> List[Message]:
    """
    Load all messages for a conversation in chronological order.

    Args:
        session: Database session
        conversation_id: ID of the conversation

    Returns:
        List of Message objects ordered by created_at ASC
    """
    logger.info(f"Loading messages for conversation {conversation_id}")
    messages = Message.get_conversation_messages(
        session=session,
        conversation_id=conversation_id
    )
    logger.info(f"Loaded {len(messages)} messages for conversation {conversation_id}")
    return messages


def convert_messages_to_agent_format(
    messages: List[Message]
) -> List[Dict[str, str]]:
    """
    Convert Message objects to the format expected by the agent service.

    The agent service expects a list of dictionaries with 'role' and 'content' keys.
    Only 'user' and 'assistant' roles are included (system messages are filtered out).

    Args:
        messages: List of Message objects from the database

    Returns:
        List of message dictionaries in agent format:
        [{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]
    """
    logger.debug(f"Converting {len(messages)} messages to agent format")

    agent_messages = []
    for msg in messages:
        # Only include user and assistant messages (filter out system messages)
        if msg.role in ["user", "assistant"]:
            agent_messages.append({
                "role": msg.role,
                "content": msg.content
            })

    logger.debug(f"Converted to {len(agent_messages)} agent messages")
    return agent_messages


def extract_tool_calls_from_response(
    agent_response: AgentResponse
) -> List[str]:
    """
    Extract the list of tool names that were called from the agent response.

    The agent response contains a complete message history including tool calls.
    This function extracts the names of all tools that were invoked during
    the agent's processing.

    Args:
        agent_response: AgentResponse object from agent service

    Returns:
        List of tool names (e.g., ["add_task", "list_tasks"])
        Returns empty list if no tools were called
    """
    logger.debug("Extracting tool calls from agent response")

    tool_names = []

    # Iterate through all messages in the response
    for message in agent_response.messages:
        # Check if this message has tool_calls
        if isinstance(message, dict) and "tool_calls" in message:
            tool_calls = message.get("tool_calls", [])
            if tool_calls:
                # Extract tool names from each tool call
                for tool_call in tool_calls:
                    if isinstance(tool_call, dict):
                        function_info = tool_call.get("function", {})
                        if isinstance(function_info, dict):
                            tool_name = function_info.get("name")
                            if tool_name and tool_name not in tool_names:
                                tool_names.append(tool_name)
                                logger.debug(f"Found tool call: {tool_name}")

    logger.info(f"Extracted {len(tool_names)} unique tool calls: {tool_names}")
    return tool_names
