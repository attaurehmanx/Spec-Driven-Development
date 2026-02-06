"""
Manual Test Script: Agent Service - Create Task

Tests User Story 1: Basic Agent Conversation
Scenario: User asks agent to create a task

Usage:
    python test_agent_manual.py
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


async def test_create_task():
    """Test agent creating a task via natural language."""
    print("=" * 70)
    print("TEST: Create Task via Agent")
    print("=" * 70)

    # Test message
    message_history = [
        {"role": "user", "content": "Create a task to buy groceries"}
    ]

    # Test user ID (replace with actual user ID from your database)
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

        # Verify response mentions task creation
        if response.status == "completed" and "task" in response.final_response.lower() and ("created" in response.final_response.lower() or "added" in response.final_response.lower()):
            print("\n✓ SUCCESS: Agent appears to have created the task")
        else:
            print("\n✗ WARNING: Response doesn't clearly confirm task creation")
            if response.error:
                print(f"Error: {response.error}")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


async def test_create_task_with_description():
    """Test agent creating a task with description."""
    print("\n" + "=" * 70)
    print("TEST: Create Task with Description")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Create a task called 'Call dentist' with description 'Schedule annual checkup'"}
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

        if response.status == "completed" and "task" in response.final_response.lower() and "dentist" in response.final_response.lower():
            print("\n✓ SUCCESS: Agent created task with correct details")
        else:
            print("\n✗ WARNING: Response doesn't clearly confirm task creation")
            if response.error:
                print(f"Error: {response.error}")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_out_of_scope_request():
    """Test agent handling out-of-scope requests."""
    print("\n" + "=" * 70)
    print("TEST: Out-of-Scope Request")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "What's the weather today?"}
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

        if response.status == "completed" and ("task" in response.final_response.lower() or "help" in response.final_response.lower()):
            print("\n✓ SUCCESS: Agent explained its limitations")
        else:
            print("\n✗ WARNING: Agent may not have explained limitations clearly")
            if response.error:
                print(f"Error: {response.error}")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def main():
    """Run all manual tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 15 + "AGENT SERVICE MANUAL TESTS" + " " * 27 + "║")
    print("╚" + "=" * 68 + "╝")
    print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
    print("NOTE: Ensure test user exists in database or update user_id\n")

    await test_create_task()
    await test_create_task_with_description()
    await test_out_of_scope_request()

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
