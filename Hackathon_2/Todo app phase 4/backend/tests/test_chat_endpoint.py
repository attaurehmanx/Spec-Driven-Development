"""
Automated Test Suite: Chat Endpoint

This module contains comprehensive pytest tests for the chat endpoint,
covering authentication, conversation management, multi-turn context,
and error handling scenarios.

Test Categories:
1. Authentication Tests - JWT token validation and user authorization
2. New Conversation Tests - Creating new conversations
3. Multi-turn Conversation Tests - Context preservation across messages
4. Error Handling Tests - Various failure scenarios
5. Integration Tests - Complete end-to-end flows

Usage:
    pytest backend/tests/test_chat_endpoint.py -v
    pytest backend/tests/test_chat_endpoint.py::test_chat_requires_authentication -v
"""

import pytest
from fastapi.testclient import TestClient
from sqlmodel import Session
from unittest.mock import patch, AsyncMock, MagicMock

from models.task_models import User
from models.conversation import Conversation
from models.message import Message, MessageRole
from services.agent_service import AgentResponse


# ============================================================================
# Authentication Tests
# ============================================================================

class TestAuthentication:
    """Test authentication and authorization for the chat endpoint."""

    def test_chat_requires_authentication(self, client: TestClient, test_user: User):
        """Test that chat endpoint requires JWT token."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            json={"message": "Hello"}
        )
        assert response.status_code == 401
        assert "detail" in response.json()

    def test_chat_rejects_invalid_token(self, client: TestClient, test_user: User):
        """Test that chat endpoint rejects invalid JWT tokens."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers={"Authorization": "Bearer invalid-token"},
            json={"message": "Hello"}
        )
        assert response.status_code == 401

    def test_chat_rejects_user_id_mismatch(
        self,
        client: TestClient,
        test_user: User,
        another_user: User,
        test_user_token: str
    ):
        """Test that users cannot access other users' chat endpoints."""
        # Try to access another user's chat with test_user's token
        response = client.post(
            f"/api/{another_user.id}/chat",
            headers={"Authorization": f"Bearer {test_user_token}"},
            json={"message": "Hello"}
        )
        assert response.status_code == 403
        assert "mismatch" in response.json()["detail"].lower()

    def test_chat_accepts_valid_authentication(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that chat endpoint accepts valid authentication."""
        # Mock the agent service to avoid actual API calls
        mock_response = AgentResponse(
            status="completed",
            final_response="Hello! How can I help you?",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Hello"}
            )
            assert response.status_code == 200


# ============================================================================
# New Conversation Tests
# ============================================================================

class TestNewConversation:
    """Test creating new conversations via the chat endpoint."""

    def test_create_new_conversation_without_conversation_id(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test that omitting conversation_id creates a new conversation."""
        mock_response = AgentResponse(
            status="completed",
            final_response="I've created a task for you.",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Create a task to buy milk"}
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "conversation_id" in data
            assert "response" in data
            assert "tool_calls" in data
            assert isinstance(data["conversation_id"], int)
            assert isinstance(data["response"], str)
            assert isinstance(data["tool_calls"], list)

            # Verify conversation was created in database
            conversation = session.get(Conversation, data["conversation_id"])
            assert conversation is not None
            assert conversation.user_id == test_user.id

    def test_user_message_persisted_before_agent_call(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test that user message is saved to database before calling agent."""
        mock_response = AgentResponse(
            status="completed",
            final_response="Response",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test message"}
            )

            assert response.status_code == 200
            conversation_id = response.json()["conversation_id"]

            # Verify user message was persisted
            messages = Message.get_conversation_messages(session, conversation_id)
            assert len(messages) >= 1
            assert messages[0].role == MessageRole.USER
            assert messages[0].content == "Test message"

    def test_ai_response_persisted_after_agent_call(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test that AI response is saved to database after agent completes."""
        mock_response = AgentResponse(
            status="completed",
            final_response="AI response text",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test message"}
            )

            assert response.status_code == 200
            conversation_id = response.json()["conversation_id"]

            # Verify AI response was persisted
            messages = Message.get_conversation_messages(session, conversation_id)
            assert len(messages) == 2
            assert messages[1].role == MessageRole.ASSISTANT
            assert messages[1].content == "AI response text"


# ============================================================================
# Multi-turn Conversation Tests
# ============================================================================

class TestMultiTurnConversation:
    """Test conversation context preservation across multiple messages."""

    def test_continue_existing_conversation(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        test_conversation: Conversation,
        session: Session
    ):
        """Test that providing conversation_id continues existing conversation."""
        mock_response = AgentResponse(
            status="completed",
            final_response="Follow-up response",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={
                    "message": "Follow-up message",
                    "conversation_id": test_conversation.id
                }
            )

            assert response.status_code == 200
            data = response.json()

            # Verify same conversation_id is returned
            assert data["conversation_id"] == test_conversation.id

            # Verify message was added to existing conversation
            messages = Message.get_conversation_messages(session, test_conversation.id)
            assert len(messages) == 4  # 2 original + 2 new (user + assistant)

    def test_conversation_history_passed_to_agent(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        test_conversation: Conversation
    ):
        """Test that conversation history is passed to agent service."""
        mock_response = AgentResponse(
            status="completed",
            final_response="Response with context",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response) as mock_agent:
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={
                    "message": "What was that task?",
                    "conversation_id": test_conversation.id
                }
            )

            assert response.status_code == 200

            # Verify agent was called with message history
            mock_agent.assert_called_once()
            call_args = mock_agent.call_args
            message_history = call_args[1]["message_history"]

            # Should include previous messages plus new user message
            assert len(message_history) >= 3
            assert message_history[0]["role"] == "user"
            assert message_history[0]["content"] == "Create a task to buy milk"

    def test_invalid_conversation_id_returns_404(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that invalid conversation_id returns 404."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={
                "message": "Test message",
                "conversation_id": 999999
            }
        )

        assert response.status_code == 404
        assert "not found" in response.json()["detail"].lower()

    def test_cannot_access_other_users_conversation(
        self,
        client: TestClient,
        test_user: User,
        another_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test that users cannot access conversations belonging to other users."""
        # Create conversation for another_user
        other_conversation = Conversation.create_conversation(
            session=session,
            user_id=another_user.id,
            title="Other user's conversation"
        )

        # Try to access it with test_user's token
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={
                "message": "Test message",
                "conversation_id": other_conversation.id
            }
        )

        assert response.status_code == 404  # Should not find conversation


# ============================================================================
# Error Handling Tests
# ============================================================================

class TestErrorHandling:
    """Test error handling for various failure scenarios."""

    def test_empty_message_returns_400(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that empty message returns 400 Bad Request."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={"message": ""}
        )

        assert response.status_code in [400, 422]

    def test_whitespace_only_message_returns_400(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that whitespace-only message returns 400 Bad Request."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={"message": "   \n\t   "}
        )

        assert response.status_code in [400, 422]

    def test_message_too_long_returns_400(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that message exceeding length limit returns 400."""
        long_message = "A" * 10001  # Exceeds 10,000 character limit
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={"message": long_message}
        )

        assert response.status_code in [400, 422]

    def test_missing_message_field_returns_422(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that missing message field returns 422."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={"conversation_id": 123}
        )

        assert response.status_code == 422

    def test_negative_conversation_id_returns_422(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that negative conversation_id returns 422."""
        response = client.post(
            f"/api/{test_user.id}/chat",
            headers=auth_headers,
            json={"message": "Test", "conversation_id": -1}
        )

        assert response.status_code == 422

    def test_agent_service_error_returns_503(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that agent service errors return 503 Service Unavailable."""
        from services.agent_service import AgentServiceError

        with patch('api.chat.run_agent', new_callable=AsyncMock, side_effect=AgentServiceError("Service down")):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test message"}
            )

            assert response.status_code == 503
            assert "unavailable" in response.json()["detail"].lower()

    def test_agent_error_status_returns_503(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that agent response with error status returns 503."""
        mock_response = AgentResponse(
            status="error",
            final_response="",
            messages=[],
            iterations=0,
            finish_reason="error",
            error="Agent failed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test message"}
            )

            assert response.status_code == 503


# ============================================================================
# Integration Tests
# ============================================================================

class TestIntegration:
    """Test complete end-to-end flows."""

    def test_complete_chat_flow_new_conversation(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test complete flow: authentication -> new conversation -> agent call -> response."""
        mock_response = AgentResponse(
            status="completed",
            final_response="I've created a task titled 'Buy groceries' for you.",
            messages=[
                {"role": "user", "content": "Create a task to buy groceries"},
                {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [
                        {
                            "id": "call_123",
                            "type": "function",
                            "function": {
                                "name": "add_task",
                                "arguments": '{"title": "Buy groceries"}'
                            }
                        }
                    ]
                },
                {
                    "role": "tool",
                    "tool_call_id": "call_123",
                    "content": '{"status": "success", "result": {"task_id": 1}}'
                }
            ],
            iterations=2,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Create a task to buy groceries"}
            )

            # Verify response
            assert response.status_code == 200
            data = response.json()
            assert data["conversation_id"] > 0
            assert "created a task" in data["response"].lower()
            assert "add_task" in data["tool_calls"]

            # Verify database state
            conversation = session.get(Conversation, data["conversation_id"])
            assert conversation is not None
            assert conversation.user_id == test_user.id

            messages = Message.get_conversation_messages(session, conversation.id)
            assert len(messages) == 2
            assert messages[0].role == MessageRole.USER
            assert messages[1].role == MessageRole.ASSISTANT

    def test_complete_multi_turn_flow(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict,
        session: Session
    ):
        """Test complete multi-turn conversation flow."""
        # First message - create conversation
        mock_response_1 = AgentResponse(
            status="completed",
            final_response="Task created.",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response_1):
            response_1 = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Create a task to call mom"}
            )

            assert response_1.status_code == 200
            conversation_id = response_1.json()["conversation_id"]

        # Second message - continue conversation
        mock_response_2 = AgentResponse(
            status="completed",
            final_response="Task marked as complete.",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response_2):
            response_2 = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={
                    "message": "Mark it as complete",
                    "conversation_id": conversation_id
                }
            )

            assert response_2.status_code == 200
            assert response_2.json()["conversation_id"] == conversation_id

        # Verify conversation has all messages
        messages = Message.get_conversation_messages(session, conversation_id)
        assert len(messages) == 4  # 2 user + 2 assistant


# ============================================================================
# Tool Call Extraction Tests
# ============================================================================

class TestToolCallExtraction:
    """Test extraction of tool calls from agent responses."""

    def test_extract_tool_calls_from_response(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that tool calls are correctly extracted from agent response."""
        mock_response = AgentResponse(
            status="completed",
            final_response="Done.",
            messages=[
                {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [
                        {
                            "id": "call_1",
                            "type": "function",
                            "function": {
                                "name": "add_task",
                                "arguments": '{}'
                            }
                        },
                        {
                            "id": "call_2",
                            "type": "function",
                            "function": {
                                "name": "list_tasks",
                                "arguments": '{}'
                            }
                        }
                    ]
                }
            ],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test"}
            )

            assert response.status_code == 200
            data = response.json()
            assert "add_task" in data["tool_calls"]
            assert "list_tasks" in data["tool_calls"]

    def test_no_tool_calls_returns_empty_array(
        self,
        client: TestClient,
        test_user: User,
        auth_headers: dict
    ):
        """Test that responses without tool calls return empty array."""
        mock_response = AgentResponse(
            status="completed",
            final_response="Just a response.",
            messages=[],
            iterations=1,
            finish_reason="completed"
        )

        with patch('api.chat.run_agent', new_callable=AsyncMock, return_value=mock_response):
            response = client.post(
                f"/api/{test_user.id}/chat",
                headers=auth_headers,
                json={"message": "Test"}
            )

            assert response.status_code == 200
            data = response.json()
            assert data["tool_calls"] == []
