"""
Manual Test Script: Agent Service - Multi-Turn Conversation

Tests User Story 2: Multi-Turn Conversation with Context
Scenario: Agent maintains context and resolves pronouns across multiple turns

Usage:
    python test_agent_multiturn.py
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


async def test_pronoun_resolution():
    """Test agent resolving 'it' pronoun from previous context."""
    print("=" * 70)
    print("TEST: Pronoun Resolution - 'Mark it as done'")
    print("=" * 70)

    # Simulate conversation where user creates a task, then refers to "it"
    message_history = [
        {"role": "user", "content": "Create a task to call mom"},
        {"role": "assistant", "content": "I've created a task titled 'Call mom' for you."},
        {"role": "user", "content": "Mark it as complete"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print("\nConversation History:")
    for msg in message_history:
        print(f"  {msg['role']}: {msg['content']}")
    print("\nCalling agent...\n")

    try:
        response = await run_agent(message_history, user_id)

        print("-" * 70)
        print("AGENT RESPONSE:")
        print("-" * 70)
        print(f"Status: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Execution Time: {response.execution_time_ms:.2f}ms")
        print(f"Response: {response.final_response}")
        print("-" * 70)

        if "complete" in response.lower() or "done" in response.lower():
            print("\n✓ SUCCESS: Agent understood 'it' refers to 'Call mom' task")
        else:
            print("\n✗ WARNING: Agent may not have resolved pronoun correctly")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


async def test_first_one_reference():
    """Test agent identifying 'the first one' from previous list."""
    print("\n" + "=" * 70)
    print("TEST: List Reference - 'Delete the first one'")
    print("=" * 70)

    # Simulate conversation where user lists tasks, then refers to "the first one"
    message_history = [
        {"role": "user", "content": "Show me my tasks"},
        {"role": "assistant", "content": "You have 3 tasks:\n1. Buy groceries (pending)\n2. Call dentist (pending)\n3. Finish report (completed)"},
        {"role": "user", "content": "Delete the first one"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print("\nConversation History:")
    for msg in message_history:
        print(f"  {msg['role']}: {msg['content'][:60]}...")
    print("\nCalling agent...\n")

    try:
        response = await run_agent(message_history, user_id)

        print("-" * 70)
        print("AGENT RESPONSE:")
        print("-" * 70)
        print(f"Status: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Execution Time: {response.execution_time_ms:.2f}ms")
        print(f"Response: {response.final_response}")
        print("-" * 70)

        if "groceries" in response.lower() or "deleted" in response.lower():
            print("\n✓ SUCCESS: Agent identified 'the first one' as 'Buy groceries'")
        else:
            print("\n✗ WARNING: Agent may not have identified the correct task")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_that_task_reference():
    """Test agent resolving 'that task' from previous context."""
    print("\n" + "=" * 70)
    print("TEST: Task Reference - 'Update that task'")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Create a task to buy milk"},
        {"role": "assistant", "content": "I've created a task titled 'Buy milk' for you."},
        {"role": "user", "content": "Update that task to say 'Buy milk and eggs'"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print("\nConversation History:")
    for msg in message_history:
        print(f"  {msg['role']}: {msg['content']}")
    print("\nCalling agent...\n")

    try:
        response = await run_agent(message_history, user_id)

        print("-" * 70)
        print("AGENT RESPONSE:")
        print("-" * 70)
        print(f"Status: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Execution Time: {response.execution_time_ms:.2f}ms")
        print(f"Response: {response.final_response}")
        print("-" * 70)

        if "eggs" in response.lower() or "updated" in response.lower():
            print("\n✓ SUCCESS: Agent understood 'that task' refers to 'Buy milk'")
        else:
            print("\n✗ WARNING: Agent may not have resolved reference correctly")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_multi_turn_context():
    """Test agent maintaining context across multiple turns."""
    print("\n" + "=" * 70)
    print("TEST: Multi-Turn Context - Complex Conversation")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "What tasks do I have?"},
        {"role": "assistant", "content": "You have 2 pending tasks: 'Call dentist' and 'Buy groceries'."},
        {"role": "user", "content": "Add a new task to finish the report"},
        {"role": "assistant", "content": "I've added 'Finish the report' to your task list."},
        {"role": "user", "content": "Now show me all my tasks"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print("\nConversation History:")
    for i, msg in enumerate(message_history, 1):
        print(f"  Turn {i} - {msg['role']}: {msg['content'][:50]}...")
    print("\nCalling agent...\n")

    try:
        response = await run_agent(message_history, user_id)

        print("-" * 70)
        print("AGENT RESPONSE:")
        print("-" * 70)
        print(f"Status: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Execution Time: {response.execution_time_ms:.2f}ms")
        print(f"Response: {response.final_response}")
        print("-" * 70)

        if "task" in response.lower():
            print("\n✓ SUCCESS: Agent maintained context across multiple turns")
        else:
            print("\n✗ WARNING: Agent may have lost context")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_ambiguous_reference():
    """Test agent asking for clarification when reference is ambiguous."""
    print("\n" + "=" * 70)
    print("TEST: Ambiguous Reference - Should Ask for Clarification")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Show me my tasks"},
        {"role": "assistant", "content": "You have 3 tasks:\n1. Buy groceries\n2. Call dentist\n3. Finish report"},
        {"role": "user", "content": "Delete it"}  # Ambiguous - which one?
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print("\nConversation History:")
    for msg in message_history:
        print(f"  {msg['role']}: {msg['content'][:60]}...")
    print("\nCalling agent...\n")

    try:
        response = await run_agent(message_history, user_id)

        print("-" * 70)
        print("AGENT RESPONSE:")
        print("-" * 70)
        print(f"Status: {response.status}")
        print(f"Iterations: {response.iterations}")
        print(f"Execution Time: {response.execution_time_ms:.2f}ms")
        print(f"Response: {response.final_response}")
        print("-" * 70)

        if "which" in response.lower() or "clarify" in response.lower() or "?" in response:
            print("\n✓ SUCCESS: Agent asked for clarification")
        else:
            print("\n✗ WARNING: Agent should ask which task to delete")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def main():
    """Run all multi-turn conversation tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 10 + "AGENT SERVICE - MULTI-TURN CONVERSATION TESTS" + " " * 13 + "║")
    print("╚" + "=" * 68 + "╝")
    print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
    print("NOTE: Ensure test user exists in database or update user_id\n")

    await test_pronoun_resolution()
    await test_first_one_reference()
    await test_that_task_reference()
    await test_multi_turn_context()
    await test_ambiguous_reference()

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
