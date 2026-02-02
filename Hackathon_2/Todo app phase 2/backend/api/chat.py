"""
Chat API Router

This module implements the REST endpoint for chat functionality, connecting
the frontend to the AI agent service with conversation persistence.

Endpoints:
- POST /api/{user_id}/chat: Send message and receive AI response
"""

import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session

from models.chat_models import ChatRequest, ChatResponse
from models.message import Message, MessageRole
from models.task_models import User
from middleware.auth import get_current_user_with_validation
from database.session import get_session
from services.conversation_service import (
    get_or_create_conversation,
    load_conversation_messages,
    convert_messages_to_agent_format,
    extract_tool_calls_from_response
)
from services.agent_service import run_agent, AgentServiceError

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/{user_id}/chat", response_model=ChatResponse)
async def chat_interaction(
    user_id: str,
    request: ChatRequest,
    current_user: User = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Send a chat message and receive an AI response with conversation persistence.

    This endpoint implements the complete stateless request/response cycle:
    1. Authenticate user via JWT token
    2. Validate user_id matches authenticated user
    3. Load or create conversation
    4. Persist user message to database
    5. Load conversation history
    6. Invoke AI agent service
    7. Persist AI response to database
    8. Return response to client

    Args:
        user_id: User ID from URL path (must match authenticated user)
        request: ChatRequest with message and optional conversation_id
        current_user: Authenticated user from JWT token (injected by dependency)
        session: Database session (injected by dependency)

    Returns:
        ChatResponse with conversation_id, response text, and tool_calls

    Raises:
        HTTPException 400: Invalid input (empty message, message too long)
        HTTPException 401: Missing or invalid JWT token (handled by dependency)
        HTTPException 403: User ID mismatch (user trying to access another user's data)
        HTTPException 404: Conversation not found
        HTTPException 500: Database or internal error
        HTTPException 503: AI agent service unavailable
    """
    logger.info(f"Chat request received for user {user_id}")

    # T014: Validate user_id matches authenticated user
    if current_user.id != user_id:
        logger.warning(f"User ID mismatch: token user {current_user.id} != URL user {user_id}")
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="User ID mismatch - you can only access your own conversations"
        )

    # T015: Input validation (additional validation beyond Pydantic)
    if not request.message or not request.message.strip():
        logger.warning("Empty message received")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Message cannot be empty"
        )

    if len(request.message) > 10000:
        logger.warning(f"Message too long: {len(request.message)} characters")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Message cannot exceed 10,000 characters"
        )

    try:
        # T016: Get or create conversation
        try:
            conversation = get_or_create_conversation(
                session=session,
                user_id=user_id,
                conversation_id=request.conversation_id
            )
        except ValueError as e:
            # Conversation not found or access denied
            logger.warning(f"Conversation error: {e}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=str(e)
            )

        logger.info(f"Using conversation {conversation.id}")

        # T018: Persist user message BEFORE calling agent
        # This ensures the message is saved even if the agent fails
        try:
            user_message = Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=MessageRole.USER,
                content=request.message
            )
            logger.info(f"Persisted user message {user_message.id}")
        except Exception as e:
            logger.error(f"Failed to persist user message: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to save message to database"
            )

        # T017: Load conversation history (including the message we just added)
        messages = load_conversation_messages(
            session=session,
            conversation_id=conversation.id
        )

        # T019: Convert messages to agent format
        message_history = convert_messages_to_agent_format(messages)

        # T020: Invoke agent service
        try:
            logger.info(f"Calling agent service with {len(message_history)} messages")
            agent_response = await run_agent(
                message_history=message_history,
                user_id=user_id
            )

            # Check agent response status
            if agent_response.status == "error":
                logger.error(f"Agent service error: {agent_response.error}")
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail="AI service encountered an error. Please try again."
                )

            if agent_response.status == "max_iterations_reached":
                logger.warning("Agent reached max iterations")
                # Still return the response, but it may be incomplete

            logger.info(f"Agent completed in {agent_response.iterations} iterations")

        except AgentServiceError as e:
            logger.error(f"Agent service error: {e}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="AI service is currently unavailable. Please try again later."
            )
        except Exception as e:
            logger.error(f"Unexpected error calling agent: {e}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Failed to process message with AI service"
            )

        # T021: Extract tool calls from agent response
        tool_calls = extract_tool_calls_from_response(agent_response)

        # T022: Persist AI response to database
        try:
            ai_message = Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=MessageRole.ASSISTANT,
                content=agent_response.final_response
            )
            logger.info(f"Persisted AI response {ai_message.id}")
        except Exception as e:
            logger.error(f"Failed to persist AI response: {e}")
            # Don't fail the request - we already have the response
            # Just log the error and continue
            logger.warning("Continuing despite database error")

        # T023: Construct and return ChatResponse
        response = ChatResponse(
            conversation_id=conversation.id,
            response=agent_response.final_response,
            tool_calls=tool_calls
        )

        logger.info(f"Chat request completed successfully for conversation {conversation.id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Catch any unexpected errors
        logger.error(f"Unexpected error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An unexpected error occurred. Please try again."
        )
