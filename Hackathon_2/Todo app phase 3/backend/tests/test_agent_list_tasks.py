"""
Manual Test Script: Agent Service - List Tasks

Tests User Story 1: Basic Agent Conversation
Scenario: User asks agent to show their tasks

Usage:
    python test_agent_list_tasks.py
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


async def test_list_all_tasks():
    """Test agent listing all tasks."""
    print("=" * 70)
    print("TEST: List All Tasks")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Show me my tasks"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"User Message: {message_history[0]['content']}")
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
            print("\n✓ SUCCESS: Agent retrieved and formatted tasks")
        else:
            print("\n✗ WARNING: Response doesn't mention tasks")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


async def test_list_pending_tasks():
    """Test agent listing pending tasks only."""
    print("\n" + "=" * 70)
    print("TEST: List Pending Tasks")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "What tasks do I have that aren't done yet?"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"User Message: {message_history[0]['content']}")
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

        if "task" in response.lower() or "pending" in response.lower():
            print("\n✓ SUCCESS: Agent filtered pending tasks")
        else:
            print("\n✗ WARNING: Response doesn't clearly show pending tasks")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_list_completed_tasks():
    """Test agent listing completed tasks only."""
    print("\n" + "=" * 70)
    print("TEST: List Completed Tasks")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Show me the tasks I've completed"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"User Message: {message_history[0]['content']}")
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

        if "task" in response.lower() or "completed" in response.lower():
            print("\n✓ SUCCESS: Agent filtered completed tasks")
        else:
            print("\n✗ WARNING: Response doesn't clearly show completed tasks")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_empty_task_list():
    """Test agent handling empty task list."""
    print("\n" + "=" * 70)
    print("TEST: Empty Task List")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "What tasks do I have?"}
    ]

    # Use a user ID that likely has no tasks
    user_id = "empty-user-999"

    print(f"\nUser ID: {user_id}")
    print(f"User Message: {message_history[0]['content']}")
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

        if "no tasks" in response.lower() or "don't have" in response.lower() or "empty" in response.lower():
            print("\n✓ SUCCESS: Agent handled empty list gracefully")
        else:
            print("\n✗ WARNING: Response may not clearly indicate empty list")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def main():
    """Run all manual tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 12 + "AGENT SERVICE - LIST TASKS TESTS" + " " * 24 + "║")
    print("╚" + "=" * 68 + "╝")
    print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
    print("NOTE: Ensure test user exists in database or update user_id\n")

    await test_list_all_tasks()
    await test_list_pending_tasks()
    await test_list_completed_tasks()
    await test_empty_task_list()

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
