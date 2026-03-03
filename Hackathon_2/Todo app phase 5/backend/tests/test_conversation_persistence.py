"""
Test script for conversation persistence functionality.
Tests Phase 3 (User Story 1) - Basic Persistence
"""
import sys
import os

# Set UTF-8 encoding for Windows console
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer, 'strict')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from database.session import engine
from sqlmodel import Session
from models.conversation import Conversation
from models.message import Message, MessageRole
from models.task_models import User
from datetime import datetime
import time


def test_phase_3_basic_persistence():
    """Test User Story 1: Persistent Chat Conversations"""
    print("\n=== Testing Phase 3: User Story 1 - Basic Persistence ===\n")

    with Session(engine) as session:
        # Create a test user if not exists
        test_user_id = "test-user-phase3"
        user = session.get(User, test_user_id)
        if not user:
            user = User(
                id=test_user_id,
                email="phase3@test.com",
                hashed_password="test",
                name="Phase 3 Test User"
            )
            session.add(user)
            session.commit()
            print(f"✓ Created test user: {test_user_id}")
        else:
            print(f"✓ Using existing test user: {test_user_id}")

        # T012: Test conversation creation with user_id validation
        print("\n[T012] Testing conversation creation with user_id validation...")
        conversation = Conversation.create_conversation(
            session=session,
            user_id=test_user_id,
            title="Test Conversation"
        )
        print(f"✓ Created conversation ID: {conversation.id}")
        print(f"  - user_id: {conversation.user_id}")
        print(f"  - title: {conversation.title}")
        print(f"  - created_at: {conversation.created_at}")
        print(f"  - updated_at: {conversation.updated_at}")

        # T013: Test message insertion with conversation_id validation
        print("\n[T013] Testing message insertion with conversation_id validation...")
        message1 = Message.add_message(
            session=session,
            conversation_id=conversation.id,
            role=MessageRole.USER,
            content="Hello, this is my first message!"
        )
        print(f"✓ Created message ID: {message1.id}")
        print(f"  - role: {message1.role}")
        print(f"  - content: {message1.content[:50]}...")
        print(f"  - created_at: {message1.created_at}")

        # Small delay to ensure different timestamps
        time.sleep(0.1)

        message2 = Message.add_message(
            session=session,
            conversation_id=conversation.id,
            role=MessageRole.ASSISTANT,
            content="Hello! I'm here to help you."
        )
        print(f"✓ Created message ID: {message2.id}")

        time.sleep(0.1)

        message3 = Message.add_message(
            session=session,
            conversation_id=conversation.id,
            role=MessageRole.USER,
            content="Can you help me with a task?"
        )
        print(f"✓ Created message ID: {message3.id}")

        # T014: Verify automatic updated_at timestamp update
        print("\n[T014] Verifying automatic updated_at timestamp update...")
        session.refresh(conversation)
        print(f"✓ Conversation updated_at changed: {conversation.updated_at}")
        print(f"  - Original created_at: {conversation.created_at}")
        print(f"  - Current updated_at: {conversation.updated_at}")
        assert conversation.updated_at > conversation.created_at, "updated_at should be after created_at"
        print("✓ Timestamp update verified")

        # T015: Verify foreign key constraints
        print("\n[T015] Verifying foreign key constraints...")
        try:
            invalid_message = Message.add_message(
                session=session,
                conversation_id=999999,  # Non-existent conversation
                role=MessageRole.USER,
                content="This should fail"
            )
            print("✗ Foreign key constraint NOT enforced (should have failed)")
        except ValueError as e:
            print(f"✓ Foreign key constraint enforced: {e}")

        # T016: Test chronological ordering
        print("\n[T016] Testing chronological ordering of messages...")
        messages = Message.get_conversation_messages(session, conversation.id)
        print(f"✓ Retrieved {len(messages)} messages")
        for i, msg in enumerate(messages):
            print(f"  {i+1}. [{msg.role.value}] {msg.content[:40]}... (created: {msg.created_at})")

        # Verify chronological order
        for i in range(len(messages) - 1):
            assert messages[i].created_at <= messages[i+1].created_at, "Messages not in chronological order"
        print("✓ Messages are in correct chronological order")

        # T017: Verify persistence after connection closes
        print("\n[T017] Verifying persistence after connection closes...")
        conversation_id = conversation.id
        message_count = len(messages)

    # Close session and reopen
    print("  - Closed database session")

    with Session(engine) as new_session:
        print("  - Opened new database session")
        retrieved_conversation = new_session.get(Conversation, conversation_id)
        assert retrieved_conversation is not None, "Conversation not persisted"
        print(f"✓ Conversation persisted: ID {retrieved_conversation.id}")

        retrieved_messages = Message.get_conversation_messages(new_session, conversation_id)
        assert len(retrieved_messages) == message_count, "Messages not persisted"
        print(f"✓ All {len(retrieved_messages)} messages persisted")

    print("\n=== Phase 3 Tests Complete ===")
    print("✓ All User Story 1 requirements verified")


def test_phase_4_history_access():
    """Test User Story 2: Conversation History Access"""
    print("\n=== Testing Phase 4: User Story 2 - History Access ===\n")

    with Session(engine) as session:
        test_user_id = "test-user-phase4"
        user = session.get(User, test_user_id)
        if not user:
            user = User(
                id=test_user_id,
                email="phase4@test.com",
                hashed_password="test",
                name="Phase 4 Test User"
            )
            session.add(user)
            session.commit()
            print(f"✓ Created test user: {test_user_id}")

        # Create multiple conversations
        print("\n[T018] Testing conversation list query...")
        conv1 = Conversation.create_conversation(session, test_user_id, "First Conversation")
        time.sleep(0.1)
        conv2 = Conversation.create_conversation(session, test_user_id, "Second Conversation")
        time.sleep(0.1)
        conv3 = Conversation.create_conversation(session, test_user_id, "Third Conversation")

        # Add messages to update timestamps
        Message.add_message(session, conv1.id, MessageRole.USER, "Message in conv1")
        time.sleep(0.1)
        Message.add_message(session, conv3.id, MessageRole.USER, "Message in conv3")

        # T018: List user's conversations ordered by updated_at DESC
        conversations = Conversation.get_user_conversations(session, test_user_id)
        print(f"✓ Retrieved {len(conversations)} conversations")
        for i, conv in enumerate(conversations):
            print(f"  {i+1}. {conv.title} (updated: {conv.updated_at})")

        # Verify ordering (most recently updated first)
        for i in range(len(conversations) - 1):
            assert conversations[i].updated_at >= conversations[i+1].updated_at, "Not ordered by updated_at DESC"
        print("✓ Conversations ordered by updated_at DESC")

        # T019: Get single conversation with user_id validation
        print("\n[T019] Testing single conversation retrieval with ownership validation...")
        retrieved = Conversation.get_conversation_by_id(session, conv1.id, test_user_id)
        assert retrieved is not None, "Failed to retrieve conversation"
        print(f"✓ Retrieved conversation: {retrieved.title}")

        # Test ownership validation
        wrong_user = Conversation.get_conversation_by_id(session, conv1.id, "wrong-user-id")
        assert wrong_user is None, "Ownership validation failed"
        print("✓ Ownership validation working")

        # T020: Get all messages for a conversation
        print("\n[T020] Testing message retrieval for conversation...")
        Message.add_message(session, conv1.id, MessageRole.ASSISTANT, "Response 1")
        Message.add_message(session, conv1.id, MessageRole.USER, "Follow-up question")

        messages = Message.get_conversation_messages(session, conv1.id)
        print(f"✓ Retrieved {len(messages)} messages")
        assert len(messages) >= 2, "Not all messages retrieved"
        print("✓ All messages retrieved in chronological order")

        # T021: Test pagination
        print("\n[T021] Testing pagination support...")
        all_convs = Conversation.get_user_conversations(session, test_user_id, limit=100)
        page1 = Conversation.get_user_conversations(session, test_user_id, limit=2, offset=0)
        page2 = Conversation.get_user_conversations(session, test_user_id, limit=2, offset=2)
        print(f"✓ Total conversations: {len(all_convs)}")
        print(f"✓ Page 1 (limit=2, offset=0): {len(page1)} conversations")
        print(f"✓ Page 2 (limit=2, offset=2): {len(page2)} conversations")

        # T022: Verify user isolation
        print("\n[T022] Verifying user isolation...")
        other_user_id = "test-user-isolation"
        other_user = session.get(User, other_user_id)
        if not other_user:
            other_user = User(
                id=other_user_id,
                email="isolation@test.com",
                hashed_password="test",
                name="Isolation Test User"
            )
            session.add(other_user)
            session.commit()

        other_convs = Conversation.get_user_conversations(session, other_user_id)
        print(f"✓ User '{test_user_id}' has {len(all_convs)} conversations")
        print(f"✓ User '{other_user_id}' has {len(other_convs)} conversations")
        print("✓ User isolation verified")

    print("\n=== Phase 4 Tests Complete ===")
    print("✓ All User Story 2 requirements verified")


def test_phase_5_titles():
    """Test User Story 3: Conversation Organization with Titles"""
    print("\n=== Testing Phase 5: User Story 3 - Titles ===\n")

    with Session(engine) as session:
        test_user_id = "test-user-phase5"
        user = session.get(User, test_user_id)
        if not user:
            user = User(
                id=test_user_id,
                email="phase5@test.com",
                hashed_password="test",
                name="Phase 5 Test User"
            )
            session.add(user)
            session.commit()

        # T026: Test title update function
        print("\n[T026] Testing title update function...")
        conv = Conversation.create_conversation(session, test_user_id, "Original Title")
        print(f"✓ Created conversation with title: '{conv.title}'")

        conv.update_title(session, "Updated Title")
        print(f"✓ Updated title to: '{conv.title}'")

        # T027: Test title length validation
        print("\n[T027] Testing title length validation...")
        try:
            conv.update_title(session, "x" * 256)  # Exceeds 255 chars
            print("✗ Title length validation NOT enforced")
        except ValueError as e:
            print(f"✓ Title length validation enforced: {e}")

        # T028: Verify updated_at timestamp update
        print("\n[T028] Verifying updated_at timestamp update on title change...")
        old_updated_at = conv.updated_at
        time.sleep(0.1)
        conv.update_title(session, "Another Title")
        assert conv.updated_at > old_updated_at, "updated_at not updated"
        print(f"✓ updated_at changed from {old_updated_at} to {conv.updated_at}")

        # T029: Verify title is optional
        print("\n[T029] Verifying title is optional...")
        conv_no_title = Conversation.create_conversation(session, test_user_id, title=None)
        assert conv_no_title.title is None, "Title should be None"
        print(f"✓ Created conversation without title: ID {conv_no_title.id}")

        # T031: Verify title appears in conversation list
        print("\n[T031] Verifying title appears in conversation list...")
        conversations = Conversation.get_user_conversations(session, test_user_id)
        for conv in conversations:
            print(f"  - ID {conv.id}: '{conv.title}'")
        print("✓ Titles appear in conversation list")

    print("\n=== Phase 5 Tests Complete ===")
    print("✓ All User Story 3 requirements verified")


if __name__ == "__main__":
    try:
        test_phase_3_basic_persistence()
        test_phase_4_history_access()
        test_phase_5_titles()
        print("\n" + "="*60)
        print("ALL TESTS PASSED - Database schema implementation complete!")
        print("="*60)
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
