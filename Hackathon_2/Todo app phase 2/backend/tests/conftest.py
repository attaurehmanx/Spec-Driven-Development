"""
Pytest configuration and fixtures for chat endpoint tests.

This module provides shared fixtures for testing the chat endpoint,
including test client setup, database session management, and test user creation.
"""

import pytest
import os
import sys
from typing import Generator, Dict, Any
from fastapi.testclient import TestClient
from sqlmodel import Session, create_engine, SQLModel
from sqlmodel.pool import StaticPool

# Add backend directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from main import app
from database.session import get_session
from models.task_models import User
from models.conversation import Conversation
from models.message import Message
from middleware.auth import create_access_token


# Test database setup
@pytest.fixture(name="session", scope="function")
def session_fixture() -> Generator[Session, None, None]:
    """
    Create a fresh in-memory SQLite database for each test.

    This ensures test isolation - each test gets a clean database state.
    """
    # Create in-memory SQLite database
    engine = create_engine(
        "sqlite:///:memory:",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )

    # Create all tables
    SQLModel.metadata.create_all(engine)

    # Create session
    with Session(engine) as session:
        yield session

    # Cleanup
    SQLModel.metadata.drop_all(engine)


@pytest.fixture(name="client")
def client_fixture(session: Session) -> TestClient:
    """
    Create a FastAPI test client with database session override.

    This allows tests to use the in-memory test database instead of
    the production database.
    """
    def get_session_override():
        return session

    app.dependency_overrides[get_session] = get_session_override

    client = TestClient(app)

    yield client

    app.dependency_overrides.clear()


@pytest.fixture(name="test_user")
def test_user_fixture(session: Session) -> User:
    """
    Create a test user in the database.

    Returns a User object that can be used for authentication tests.
    """
    user = User(
        id="test-user-123",
        email="test@example.com",
        name="Test User",
        emailVerified=False,
        createdAt="2024-01-01T00:00:00Z",
        updatedAt="2024-01-01T00:00:00Z"
    )
    session.add(user)
    session.commit()
    session.refresh(user)
    return user


@pytest.fixture(name="test_user_token")
def test_user_token_fixture(test_user: User) -> str:
    """
    Create a valid JWT token for the test user.

    This token can be used in Authorization headers for authenticated requests.
    """
    # Create JWT token with user_id
    token_data = {
        "sub": test_user.id,
        "email": test_user.email,
        "name": test_user.name
    }
    token = create_access_token(token_data)
    return token


@pytest.fixture(name="auth_headers")
def auth_headers_fixture(test_user_token: str) -> Dict[str, str]:
    """
    Create authentication headers with Bearer token.

    Returns a dict that can be passed to client.post() as headers parameter.
    """
    return {
        "Authorization": f"Bearer {test_user_token}",
        "Content-Type": "application/json"
    }


@pytest.fixture(name="test_conversation")
def test_conversation_fixture(session: Session, test_user: User) -> Conversation:
    """
    Create a test conversation with some messages.

    Returns a Conversation object with pre-populated messages for testing
    multi-turn conversation scenarios.
    """
    # Create conversation
    conversation = Conversation.create_conversation(
        session=session,
        user_id=test_user.id,
        title="Test Conversation"
    )

    # Add some messages
    Message.add_message(
        session=session,
        conversation_id=conversation.id,
        role="user",
        content="Create a task to buy milk"
    )

    Message.add_message(
        session=session,
        conversation_id=conversation.id,
        role="assistant",
        content="I've created a task titled 'Buy milk' for you."
    )

    return conversation


@pytest.fixture(name="another_user")
def another_user_fixture(session: Session) -> User:
    """
    Create a second test user for testing authorization scenarios.

    This user can be used to test that users cannot access each other's data.
    """
    user = User(
        id="another-user-456",
        email="another@example.com",
        name="Another User",
        emailVerified=False,
        createdAt="2024-01-01T00:00:00Z",
        updatedAt="2024-01-01T00:00:00Z"
    )
    session.add(user)
    session.commit()
    session.refresh(user)
    return user


@pytest.fixture(name="another_user_token")
def another_user_token_fixture(another_user: User) -> str:
    """
    Create a valid JWT token for the second test user.
    """
    token_data = {
        "sub": another_user.id,
        "email": another_user.email,
        "name": another_user.name
    }
    token = create_access_token(token_data)
    return token
