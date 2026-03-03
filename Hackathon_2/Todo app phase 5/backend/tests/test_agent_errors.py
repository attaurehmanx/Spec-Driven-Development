"""
Manual Test Script: Agent Service - Error Handling

Tests User Story 3: Error Handling and User Guidance
Scenario: Agent provides clear, helpful error messages when operations fail

Usage:
    python test_agent_errors.py
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


async def test_task_not_found():
    """Test agent handling TaskNotFoundError gracefully."""
    print("=" * 70)
    print("TEST: Task Not Found Error")
    print("=" * 70)

    # Try to complete a task that doesn't exist
    message_history = [
        {"role": "user", "content": "Mark task 99999 as complete"}
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

        if "not found" in response.lower() or "couldn't find" in response.lower() or "doesn't exist" in response.lower():
            print("\n✓ SUCCESS: Agent explained task not found in user-friendly language")
        else:
            print("\n✗ WARNING: Agent should explain task not found error")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


async def test_unauthorized_access():
    """Test agent handling UnauthorizedTaskAccessError."""
    print("\n" + "=" * 70)
    print("TEST: Unauthorized Access Error")
    print("=" * 70)

    # Try to access a task that belongs to another user
    # Note: This requires a task that exists but belongs to a different user
    message_history = [
        {"role": "user", "content": "Delete task 1"}  # Assuming task 1 belongs to another user
    ]

    user_id = "different-user-456"

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

        if "permission" in response.lower() or "not found" in response.lower() or "access" in response.lower():
            print("\n✓ SUCCESS: Agent handled unauthorized access gracefully")
        else:
            print("\n✗ WARNING: Agent should explain permission error")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_validation_error():
    """Test agent handling ValidationError for invalid input."""
    print("\n" + "=" * 70)
    print("TEST: Validation Error - Empty Task Title")
    print("=" * 70)

    # Try to create a task with empty title
    message_history = [
        {"role": "user", "content": "Create a task with no title"}
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

        if "title" in response.lower() or "name" in response.lower() or "what" in response.lower():
            print("\n✓ SUCCESS: Agent asked for missing information")
        else:
            print("\n✗ WARNING: Agent should ask for task title")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_ambiguous_request():
    """Test agent asking clarifying questions for ambiguous requests."""
    print("\n" + "=" * 70)
    print("TEST: Ambiguous Request - Clarifying Questions")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Delete the task"}  # Which task?
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

        if "which" in response.lower() or "?" in response or "specify" in response.lower():
            print("\n✓ SUCCESS: Agent asked clarifying question")
        else:
            print("\n✗ WARNING: Agent should ask which task to delete")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_out_of_scope_request():
    """Test agent explaining limitations for out-of-scope requests."""
    print("\n" + "=" * 70)
    print("TEST: Out-of-Scope Request")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "What's the weather like today?"}
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

        if "task" in response.lower() or "can't" in response.lower() or "only" in response.lower():
            print("\n✓ SUCCESS: Agent explained its limitations politely")
        else:
            print("\n✗ WARNING: Agent should explain it can only help with tasks")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_missing_information():
    """Test agent asking for missing required information."""
    print("\n" + "=" * 70)
    print("TEST: Missing Information - Task Details")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Create a task"}  # No title provided
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

        if "what" in response.lower() or "title" in response.lower() or "?" in response:
            print("\n✓ SUCCESS: Agent asked for missing task title")
        else:
            print("\n✗ WARNING: Agent should ask what the task should be")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_tool_error_explanation():
    """Test agent explaining tool errors in natural language."""
    print("\n" + "=" * 70)
    print("TEST: Tool Error Explanation")
    print("=" * 70)

    # Try to update a non-existent task
    message_history = [
        {"role": "user", "content": "Update task 88888 to say 'New title'"}
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

        if "not found" in response.lower() or "couldn't" in response.lower() or "doesn't exist" in response.lower():
            print("\n✓ SUCCESS: Agent explained error in user-friendly language")
        else:
            print("\n✗ WARNING: Agent should explain the error clearly")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def main():
    """Run all error handling tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 14 + "AGENT SERVICE - ERROR HANDLING TESTS" + " " * 18 + "║")
    print("╚" + "=" * 68 + "╝")
    print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
    print("NOTE: Ensure test user exists in database or update user_id\n")

    await test_task_not_found()
    await test_unauthorized_access()
    await test_validation_error()
    await test_ambiguous_request()
    await test_out_of_scope_request()
    await test_missing_information()
    await test_tool_error_explanation()

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
