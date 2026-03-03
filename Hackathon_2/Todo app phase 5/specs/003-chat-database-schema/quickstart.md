# Quickstart: Chat Conversation Persistence

**Feature**: 003-chat-database-schema
**Date**: 2026-01-29
**Purpose**: Instructions for testing the database schema implementation

## Prerequisites

- Python 3.10+ installed
- PostgreSQL database running (Neon Serverless or local)
- Backend dependencies installed: `pip install -r backend/requirements.txt`
- Database connection configured in `backend/.env`

## Environment Setup

Ensure your `backend/.env` file contains:

```env
DATABASE_URL=postgresql://username:password@host:port/database
JWT_SECRET_KEY=your-secret-key
JWT_ALGORITHM=HS256
```

## Step 1: Verify Existing Backend

Before adding chat models, verify the existing backend works:

```bash
# Navigate to backend directory
cd backend

# Test database connection
python -c "from database.session import engine; print('Database connected:', engine.url)"

# Verify existing models
python -c "from models.task_models import User, Task; print('Models imported successfully')"
```

**Expected Output**: No errors, confirmation messages printed

## Step 2: Add Chat Models

The new models should be created at:
- `backend/models/conversation.py` - Conversation model
- `backend/models/message.py` - Message model

Verify models are importable:

```bash
python -c "from models.conversation import Conversation; from models.message import Message, MessageRole; print('Chat models imported successfully')"
```

## Step 3: Update Models __init__.py

Ensure `backend/models/__init__.py` imports the new models:

```python
from .task_models import User, Task, TaskCreate, TaskUpdate, TaskResponse
from .conversation import Conversation
from .message import Message, MessageRole

__all__ = [
    "User",
    "Task",
    "TaskCreate",
    "TaskUpdate",
    "TaskResponse",
    "Conversation",
    "Message",
    "MessageRole",
]
```

## Step 4: Create Database Tables

Run the migrations script to create tables:

```bash
# From backend directory
python database/migrations.py
```

**Expected Output**:
```
Database tables created successfully!
```

## Step 5: Verify Tables Created

Connect to your database and verify the new tables exist:

```bash
# Using psql (PostgreSQL command-line)
psql $DATABASE_URL -c "\dt"
```

**Expected Tables**:
- `user` (existing)
- `task` (existing)
- `conversation` (NEW)
- `message` (NEW)

## Step 6: Verify Table Schema

Check the conversation table structure:

```bash
psql $DATABASE_URL -c "\d conversation"
```

**Expected Columns**:
- `id` (integer, primary key)
- `user_id` (character varying, foreign key to user.id)
- `title` (character varying(255), nullable)
- `created_at` (timestamp)
- `updated_at` (timestamp)

**Expected Indexes**:
- Primary key on `id`
- Index on `user_id`
- Index on `updated_at`

Check the message table structure:

```bash
psql $DATABASE_URL -c "\d message"
```

**Expected Columns**:
- `id` (integer, primary key)
- `conversation_id` (integer, foreign key to conversation.id)
- `role` (character varying(20))
- `content` (text)
- `created_at` (timestamp)

**Expected Indexes**:
- Primary key on `id`
- Index on `conversation_id`
- Index on `created_at`

## Step 7: Test Data Insertion

Create a test script to verify CRUD operations:

```python
# test_chat_models.py
from database.session import get_session
from models.conversation import Conversation
from models.message import Message, MessageRole
from datetime import datetime

def test_conversation_creation():
    """Test creating a conversation"""
    with next(get_session()) as session:
        # Create conversation
        conversation = Conversation(
            user_id="test-user-123",
            title="Test Conversation"
        )
        session.add(conversation)
        session.commit()
        session.refresh(conversation)

        print(f"✓ Created conversation: {conversation.id}")
        return conversation.id

def test_message_creation(conversation_id):
    """Test creating messages"""
    with next(get_session()) as session:
        # Create user message
        user_msg = Message(
            conversation_id=conversation_id,
            role=MessageRole.USER,
            content="Hello, AI assistant!"
        )
        session.add(user_msg)

        # Create assistant message
        assistant_msg = Message(
            conversation_id=conversation_id,
            role=MessageRole.ASSISTANT,
            content="Hello! How can I help you today?"
        )
        session.add(assistant_msg)

        session.commit()
        print(f"✓ Created 2 messages in conversation {conversation_id}")

def test_conversation_retrieval(conversation_id):
    """Test retrieving conversation with messages"""
    with next(get_session()) as session:
        from sqlmodel import select

        # Get conversation
        conversation = session.get(Conversation, conversation_id)
        print(f"✓ Retrieved conversation: {conversation.title}")

        # Get messages
        messages = session.exec(
            select(Message)
            .where(Message.conversation_id == conversation_id)
            .order_by(Message.created_at)
        ).all()

        print(f"✓ Retrieved {len(messages)} messages")
        for msg in messages:
            print(f"  - {msg.role}: {msg.content[:50]}...")

if __name__ == "__main__":
    print("Testing Chat Database Schema...")
    print("-" * 50)

    # Test 1: Create conversation
    conv_id = test_conversation_creation()

    # Test 2: Create messages
    test_message_creation(conv_id)

    # Test 3: Retrieve data
    test_conversation_retrieval(conv_id)

    print("-" * 50)
    print("✓ All tests passed!")
```

Run the test:

```bash
python test_chat_models.py
```

**Expected Output**:
```
Testing Chat Database Schema...
--------------------------------------------------
✓ Created conversation: 1
✓ Created 2 messages in conversation 1
✓ Retrieved conversation: Test Conversation
✓ Retrieved 2 messages
  - user: Hello, AI assistant!...
  - assistant: Hello! How can I help you today?...
--------------------------------------------------
✓ All tests passed!
```

## Step 8: Verify Foreign Key Constraints

Test that foreign key constraints work:

```python
# test_constraints.py
from database.session import get_session
from models.message import Message, MessageRole

def test_invalid_conversation_id():
    """Test that invalid conversation_id is rejected"""
    with next(get_session()) as session:
        try:
            # Try to create message with non-existent conversation
            msg = Message(
                conversation_id=99999,
                role=MessageRole.USER,
                content="This should fail"
            )
            session.add(msg)
            session.commit()
            print("✗ Foreign key constraint NOT working!")
        except Exception as e:
            print(f"✓ Foreign key constraint working: {type(e).__name__}")

if __name__ == "__main__":
    test_invalid_conversation_id()
```

**Expected Output**: Foreign key constraint error (IntegrityError)

## Step 9: Verify User Isolation

Test that user isolation works at query level:

```python
# test_user_isolation.py
from database.session import get_session
from models.conversation import Conversation
from sqlmodel import select

def test_user_isolation():
    """Test that users can only see their own conversations"""
    with next(get_session()) as session:
        # Create conversations for two users
        conv1 = Conversation(user_id="user-1", title="User 1 Conversation")
        conv2 = Conversation(user_id="user-2", title="User 2 Conversation")
        session.add(conv1)
        session.add(conv2)
        session.commit()

        # Query as user-1
        user1_convs = session.exec(
            select(Conversation).where(Conversation.user_id == "user-1")
        ).all()

        print(f"✓ User 1 sees {len(user1_convs)} conversation(s)")
        assert len(user1_convs) == 1
        assert user1_convs[0].title == "User 1 Conversation"

        # Query as user-2
        user2_convs = session.exec(
            select(Conversation).where(Conversation.user_id == "user-2")
        ).all()

        print(f"✓ User 2 sees {len(user2_convs)} conversation(s)")
        assert len(user2_convs) == 1
        assert user2_convs[0].title == "User 2 Conversation"

        print("✓ User isolation verified!")

if __name__ == "__main__":
    test_user_isolation()
```

## Step 10: Performance Testing

Test with larger datasets:

```python
# test_performance.py
from database.session import get_session
from models.conversation import Conversation
from models.message import Message, MessageRole
import time

def test_large_conversation():
    """Test conversation with 1000+ messages"""
    with next(get_session()) as session:
        # Create conversation
        conv = Conversation(user_id="perf-test-user", title="Performance Test")
        session.add(conv)
        session.commit()
        session.refresh(conv)

        # Add 1000 messages
        start = time.time()
        for i in range(1000):
            msg = Message(
                conversation_id=conv.id,
                role=MessageRole.USER if i % 2 == 0 else MessageRole.ASSISTANT,
                content=f"Message {i}: " + ("x" * 100)
            )
            session.add(msg)
        session.commit()
        insert_time = time.time() - start

        print(f"✓ Inserted 1000 messages in {insert_time:.2f}s")

        # Retrieve messages
        from sqlmodel import select
        start = time.time()
        messages = session.exec(
            select(Message)
            .where(Message.conversation_id == conv.id)
            .order_by(Message.created_at)
        ).all()
        retrieve_time = time.time() - start

        print(f"✓ Retrieved {len(messages)} messages in {retrieve_time:.2f}s")
        assert retrieve_time < 2.0, "Retrieval should be under 2 seconds (SC-002)"

if __name__ == "__main__":
    test_large_conversation()
```

## Troubleshooting

### Issue: Tables not created

**Solution**: Check that models are imported in `__init__.py` and run migrations again

### Issue: Foreign key constraint errors

**Solution**: Ensure `user` table exists before creating conversations

### Issue: Import errors

**Solution**: Verify Python path includes backend directory:
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
```

### Issue: Database connection errors

**Solution**: Verify DATABASE_URL in `.env` and database is running

## Cleanup

To remove test data:

```bash
psql $DATABASE_URL -c "DELETE FROM message WHERE conversation_id IN (SELECT id FROM conversation WHERE user_id LIKE 'test-%' OR user_id LIKE 'perf-%');"
psql $DATABASE_URL -c "DELETE FROM conversation WHERE user_id LIKE 'test-%' OR user_id LIKE 'perf-%';"
```

## Next Steps

After verifying the database schema:
1. Proceed to `/sp.tasks` to generate implementation tasks
2. Implement API endpoints for conversation management
3. Integrate with AI agent service
4. Add frontend chat interface

## Success Criteria Verification

- ✅ Tables created with correct schema
- ✅ Foreign key constraints enforced
- ✅ Indexes created on key columns
- ✅ User isolation works at query level
- ✅ Can insert and retrieve conversations and messages
- ✅ Performance meets targets (<2s retrieval, 1000+ messages supported)
