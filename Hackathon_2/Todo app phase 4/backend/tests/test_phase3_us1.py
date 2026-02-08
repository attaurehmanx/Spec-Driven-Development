"""
Test script for Phase 3 (User Story 1): Persistent Chat Conversations

Tests:
- T012: Conversation creation with user_id validation
- T013: Message insertion with conversation_id validation
- T014: Automatic updated_at timestamp update when message added
- T015: Foreign key constraints (conversation -> user, message -> conversation)
- T016: Creating conversation with multiple messages in chronological order
- T017: Messages persist after database connection closes and reopens
"""
import sys
import os

# Add backend directory to Python path
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from database.session import get_session
from models.conversation import Conversation
from models.message import Message, MessageRole
from models.task_models import User
from datetime import datetime
import time


def test_t012_conversation_creation_with_validation():
    """T012: Test conversation creation with user_id validation"""
    print("\n[T012] Testing conversation creation with user_id validation...")

    with next(get_session()) as session:
        # Create a test user first
        test_user = User(
            id="test-user-phase3-t012",
            email="phase3test@example.com",
            hashed_password="dummy_hash",
            name="Phase 3 Test User"
        )
        session.add(test_user)
        session.commit()

        # Test 1: Valid conversation creation
        try:
            conversation = Conversation.create_conversation(
                session=session,
                user_id="test-user-phase3-t012",
                title="Test Conversation T012"
            )
            assert conversation.id is not None
            assert conversation.user_id == "test-user-phase3-t012"
            assert conversation.title == "Test Conversation T012"
            print("  [PASS] Valid conversation created successfully")
        except Exception as e:
            print(f"  [FAIL] Failed to create valid conversation: {e}")
            return False

        # Test 2: Invalid user_id (empty)
        try:
            Conversation.create_conversation(
                session=session,
                user_id="",
                title="Should Fail"
            )
            print("  [FAIL] Empty user_id should have raised ValueError")
            return False
        except ValueError as e:
            print(f"  [PASS] Empty user_id correctly rejected: {e}")

        # Test 3: Invalid title (too long)
        try:
            Conversation.create_conversation(
                session=session,
                user_id="test-user-phase3-t012",
                title="x" * 256  # Exceeds 255 character limit
            )
            print("  [FAIL] Title exceeding 255 chars should have raised ValueError")
            return False
        except ValueError as e:
            print(f"  [PASS] Long title correctly rejected: {e}")

        # Cleanup (delete in correct order: conversations -> users)
        session.delete(conversation)
        session.commit()
        session.delete(test_user)
        session.commit()

    print("  [OK] T012 PASSED")
    return True


def test_t013_message_insertion_with_validation():
    """T013: Test message insertion with conversation_id validation"""
    print("\n[T013] Testing message insertion with conversation_id validation...")

    with next(get_session()) as session:
        # Create test user and conversation
        test_user = User(
            id="test-user-phase3-t013",
            email="phase3test13@example.com",
            hashed_password="dummy_hash",
            name="Phase 3 Test User T013"
        )
        session.add(test_user)
        session.commit()

        conversation = Conversation.create_conversation(
            session=session,
            user_id="test-user-phase3-t013",
            title="Test Conversation T013"
        )

        # Test 1: Valid message insertion
        try:
            message = Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=MessageRole.USER,
                content="Test message content"
            )
            assert message.id is not None
            assert message.conversation_id == conversation.id
            assert message.role == MessageRole.USER
            assert message.content == "Test message content"
            print("  [PASS] Valid message created successfully")
        except Exception as e:
            print(f"  [FAIL] Failed to create valid message: {e}")
            return False

        # Test 2: Invalid conversation_id
        try:
            Message.add_message(
                session=session,
                conversation_id=99999,  # Non-existent conversation
                role=MessageRole.USER,
                content="Should fail"
            )
            print("  [FAIL] Invalid conversation_id should have raised ValueError")
            return False
        except ValueError as e:
            print(f"  [PASS] Invalid conversation_id correctly rejected: {e}")

        # Test 3: Empty content
        try:
            Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=MessageRole.USER,
                content=""
            )
            print("  [FAIL] Empty content should have raised ValueError")
            return False
        except ValueError as e:
            print(f"  [PASS] Empty content correctly rejected: {e}")

        # Test 4: Content too long
        try:
            Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=MessageRole.USER,
                content="x" * 10001  # Exceeds 10,000 character limit
            )
            print("  [FAIL] Content exceeding 10,000 chars should have raised ValueError")
            return False
        except ValueError as e:
            print(f"  [PASS] Long content correctly rejected: {e}")

        # Cleanup (delete in correct order: messages -> conversations -> users)
        session.delete(message)
        session.commit()
        session.delete(conversation)
        session.commit()
        session.delete(test_user)
        session.commit()

    print("  [OK] T013 PASSED")
    return True


def test_t014_automatic_updated_at_timestamp():
    """T014: Test automatic updated_at timestamp update when message added"""
    print("\n[T014] Testing automatic updated_at timestamp update...")

    with next(get_session()) as session:
        # Create test user and conversation
        test_user = User(
            id="test-user-phase3-t014",
            email="phase3test14@example.com",
            hashed_password="dummy_hash",
            name="Phase 3 Test User T014"
        )
        session.add(test_user)
        session.commit()

        conversation = Conversation.create_conversation(
            session=session,
            user_id="test-user-phase3-t014",
            title="Test Conversation T014"
        )

        initial_updated_at = conversation.updated_at
        print(f"  Initial updated_at: {initial_updated_at}")

        # Wait a moment to ensure timestamp difference
        time.sleep(0.1)

        # Add a message
        Message.add_message(
            session=session,
            conversation_id=conversation.id,
            role=MessageRole.USER,
            content="This should update the conversation's updated_at"
        )

        # Refresh conversation to get updated timestamp
        session.refresh(conversation)
        new_updated_at = conversation.updated_at
        print(f"  New updated_at: {new_updated_at}")

        if new_updated_at > initial_updated_at:
            print("  [PASS] updated_at timestamp correctly updated when message added")
        else:
            print("  [FAIL] updated_at timestamp was not updated")
            return False

        # Cleanup (delete in correct order: conversations -> users)
        session.delete(conversation)
        session.commit()
        session.delete(test_user)
        session.commit()

    print("  [OK] T014 PASSED")
    return True


def test_t015_foreign_key_constraints():
    """T015: Test foreign key constraints work"""
    print("\n[T015] Testing foreign key constraints...")

    with next(get_session()) as session:
        # Test 1: Conversation with non-existent user_id
        print("  Testing conversation -> user foreign key...")
        try:
            conversation = Conversation(
                user_id="non-existent-user-12345",
                title="Should fail"
            )
            session.add(conversation)
            session.commit()
            print("  [FAIL] Foreign key constraint not enforced (conversation -> user)")
            session.rollback()
            return False
        except Exception as e:
            session.rollback()
            print(f"  [PASS] Foreign key constraint working (conversation -> user): {type(e).__name__}")

        # Test 2: Message with non-existent conversation_id
        print("  Testing message -> conversation foreign key...")
        try:
            message = Message(
                conversation_id=99999,
                role=MessageRole.USER,
                content="Should fail"
            )
            session.add(message)
            session.commit()
            print("  [FAIL] Foreign key constraint not enforced (message -> conversation)")
            session.rollback()
            return False
        except Exception as e:
            session.rollback()
            print(f"  [PASS] Foreign key constraint working (message -> conversation): {type(e).__name__}")

    print("  [OK] T015 PASSED")
    return True


def test_t016_multiple_messages_chronological_order():
    """T016: Test creating conversation with multiple messages in chronological order"""
    print("\n[T016] Testing multiple messages in chronological order...")

    with next(get_session()) as session:
        # Create test user and conversation
        test_user = User(
            id="test-user-phase3-t016",
            email="phase3test16@example.com",
            hashed_password="dummy_hash",
            name="Phase 3 Test User T016"
        )
        session.add(test_user)
        session.commit()

        conversation = Conversation.create_conversation(
            session=session,
            user_id="test-user-phase3-t016",
            title="Test Conversation T016"
        )

        # Add multiple messages with slight delays
        messages_data = [
            (MessageRole.USER, "First message from user"),
            (MessageRole.ASSISTANT, "First response from assistant"),
            (MessageRole.USER, "Second message from user"),
            (MessageRole.ASSISTANT, "Second response from assistant"),
            (MessageRole.SYSTEM, "System notification"),
        ]

        created_messages = []
        for role, content in messages_data:
            time.sleep(0.01)  # Small delay to ensure different timestamps
            msg = Message.add_message(
                session=session,
                conversation_id=conversation.id,
                role=role,
                content=content
            )
            created_messages.append(msg)

        # Retrieve messages and verify order
        retrieved_messages = Message.get_conversation_messages(
            session=session,
            conversation_id=conversation.id
        )

        if len(retrieved_messages) != len(messages_data):
            print(f"  [FAIL] Expected {len(messages_data)} messages, got {len(retrieved_messages)}")
            return False

        print(f"  [PASS] Created and retrieved {len(retrieved_messages)} messages")

        # Verify chronological order
        for i in range(len(retrieved_messages) - 1):
            if retrieved_messages[i].created_at > retrieved_messages[i + 1].created_at:
                print(f"  [FAIL] Messages not in chronological order")
                return False

        print("  [PASS] Messages are in correct chronological order")

        # Verify content matches
        for i, (role, content) in enumerate(messages_data):
            if retrieved_messages[i].role != role or retrieved_messages[i].content != content:
                print(f"  [FAIL] Message {i} content mismatch")
                return False

        print("  [PASS] All message content matches expected values")

        # Cleanup (delete in correct order: conversations -> users)
        session.delete(conversation)
        session.commit()
        session.delete(test_user)
        session.commit()

    print("  [OK] T016 PASSED")
    return True


def test_t017_persistence_across_connections():
    """T017: Test messages persist after database connection closes and reopens"""
    print("\n[T017] Testing persistence across database connections...")

    conversation_id = None
    user_id = "test-user-phase3-t017"

    # Connection 1: Create data
    with next(get_session()) as session:
        test_user = User(
            id=user_id,
            email="phase3test17@example.com",
            hashed_password="dummy_hash",
            name="Phase 3 Test User T017"
        )
        session.add(test_user)
        session.commit()

        conversation = Conversation.create_conversation(
            session=session,
            user_id=user_id,
            title="Persistence Test"
        )
        conversation_id = conversation.id

        Message.add_message(
            session=session,
            conversation_id=conversation_id,
            role=MessageRole.USER,
            content="Message 1: Testing persistence"
        )

        Message.add_message(
            session=session,
            conversation_id=conversation_id,
            role=MessageRole.ASSISTANT,
            content="Message 2: This should persist"
        )

        print(f"  [PASS] Created conversation {conversation_id} with 2 messages")

    # Connection closed here
    print("  [PASS] Database connection closed")

    # Connection 2: Retrieve data
    with next(get_session()) as session:
        print("  [PASS] New database connection opened")

        # Retrieve conversation
        conversation = Conversation.get_conversation_by_id(
            session=session,
            conversation_id=conversation_id,
            user_id=user_id
        )

        if not conversation:
            print("  [FAIL] Conversation not found after reconnection")
            return False

        print(f"  [PASS] Retrieved conversation: {conversation.title}")

        # Retrieve messages
        messages = Message.get_conversation_messages(
            session=session,
            conversation_id=conversation_id
        )

        if len(messages) != 2:
            print(f"  [FAIL] Expected 2 messages, got {len(messages)}")
            return False

        print(f"  [PASS] Retrieved {len(messages)} messages")

        # Verify message content
        if messages[0].content != "Message 1: Testing persistence":
            print("  [FAIL] First message content mismatch")
            return False

        if messages[1].content != "Message 2: This should persist":
            print("  [FAIL] Second message content mismatch")
            return False

        print("  [PASS] All message content persisted correctly")

        # Cleanup (delete in correct order: conversations -> users)
        session.delete(conversation)
        session.commit()
        session.delete(test_user)
        session.commit()

    print("  [OK] T017 PASSED")
    return True


def main():
    """Run all Phase 3 (User Story 1) tests"""
    print("=" * 70)
    print("Phase 3 (User Story 1): Persistent Chat Conversations - Test Suite")
    print("=" * 70)

    tests = [
        test_t012_conversation_creation_with_validation,
        test_t013_message_insertion_with_validation,
        test_t014_automatic_updated_at_timestamp,
        test_t015_foreign_key_constraints,
        test_t016_multiple_messages_chronological_order,
        test_t017_persistence_across_connections,
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  [FAIL] Test failed with exception: {e}")
            import traceback
            traceback.print_exc()
            results.append(False)

    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if all(results):
        print("\n[SUCCESS] ALL PHASE 3 (USER STORY 1) TESTS PASSED!")
        print("\nUser Story 1 is fully functional:")
        print("  [PASS] Conversations can be created with user_id validation")
        print("  [PASS] Messages can be added with conversation_id validation")
        print("  [PASS] Timestamps update automatically when messages are added")
        print("  [PASS] Foreign key constraints are enforced")
        print("  [PASS] Multiple messages maintain chronological order")
        print("  [PASS] Data persists across database connections")
        return True
    else:
        print("\n[FAILED] SOME TESTS FAILED")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
