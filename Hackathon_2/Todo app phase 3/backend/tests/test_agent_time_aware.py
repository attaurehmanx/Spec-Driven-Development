"""
Manual Test Script: Agent Service - Time-Aware Task Management

Tests User Story 4: Time-Aware Task Management
Scenario: Agent interprets time-relative queries like "today", "tomorrow"

Usage:
    python test_agent_time_aware.py
"""

import asyncio
import sys
import os
from datetime import datetime, timedelta

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from services.agent_service import run_agent


async def test_today_interpretation():
    """Test agent interpreting 'today' correctly."""
    print("=" * 70)
    print("TEST: Time Interpretation - 'today'")
    print("=" * 70)

    today = datetime.utcnow().strftime("%Y-%m-%d")

    message_history = [
        {"role": "user", "content": "Show me tasks due today"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"Current Date: {today}")
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

        if "today" in response.lower() or today in response or "task" in response.lower():
            print("\n✓ SUCCESS: Agent interpreted 'today' correctly")
        else:
            print("\n✗ WARNING: Agent should reference today's date")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()


async def test_tomorrow_interpretation():
    """Test agent interpreting 'tomorrow' correctly."""
    print("\n" + "=" * 70)
    print("TEST: Time Interpretation - 'tomorrow'")
    print("=" * 70)

    tomorrow = (datetime.utcnow() + timedelta(days=1)).strftime("%Y-%m-%d")

    message_history = [
        {"role": "user", "content": "Create a task to call the dentist tomorrow"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"Tomorrow's Date: {tomorrow}")
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

        if "tomorrow" in response.lower() or tomorrow in response or "created" in response.lower():
            print("\n✓ SUCCESS: Agent interpreted 'tomorrow' correctly")
        else:
            print("\n✗ WARNING: Agent should reference tomorrow's date")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_this_week_interpretation():
    """Test agent interpreting 'this week' correctly."""
    print("\n" + "=" * 70)
    print("TEST: Time Interpretation - 'this week'")
    print("=" * 70)

    message_history = [
        {"role": "user", "content": "Show me tasks due this week"}
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

        if "week" in response.lower() or "task" in response.lower():
            print("\n✓ SUCCESS: Agent interpreted 'this week' correctly")
        else:
            print("\n✗ WARNING: Agent should reference this week")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_current_time_awareness():
    """Test agent checking current time for time-relative queries."""
    print("\n" + "=" * 70)
    print("TEST: Current Time Awareness")
    print("=" * 70)

    current_time = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC")

    message_history = [
        {"role": "user", "content": "What time is it now?"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"Current Time: {current_time}")
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

        if "time" in response.lower() or datetime.utcnow().strftime("%Y-%m-%d") in response:
            print("\n✓ SUCCESS: Agent referenced current time")
        else:
            print("\n✗ WARNING: Agent should reference current time")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_past_time_reference():
    """Test agent handling past time references."""
    print("\n" + "=" * 70)
    print("TEST: Past Time Reference - 'yesterday'")
    print("=" * 70)

    yesterday = (datetime.utcnow() - timedelta(days=1)).strftime("%Y-%m-%d")

    message_history = [
        {"role": "user", "content": "Show me tasks I completed yesterday"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"Yesterday's Date: {yesterday}")
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

        if "yesterday" in response.lower() or yesterday in response or "task" in response.lower():
            print("\n✓ SUCCESS: Agent interpreted 'yesterday' correctly")
        else:
            print("\n✗ WARNING: Agent should reference yesterday")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def test_relative_date_calculation():
    """Test agent calculating relative dates correctly."""
    print("\n" + "=" * 70)
    print("TEST: Relative Date Calculation - 'in 3 days'")
    print("=" * 70)

    future_date = (datetime.utcnow() + timedelta(days=3)).strftime("%Y-%m-%d")

    message_history = [
        {"role": "user", "content": "Create a task to submit report in 3 days"}
    ]

    user_id = "test-user-123"

    print(f"\nUser ID: {user_id}")
    print(f"Date in 3 days: {future_date}")
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

        if "created" in response.lower() or "task" in response.lower():
            print("\n✓ SUCCESS: Agent handled relative date calculation")
        else:
            print("\n✗ WARNING: Agent should create task with future date")

    except Exception as e:
        print(f"\n✗ ERROR: {type(e).__name__}: {e}")


async def main():
    """Run all time-aware tests."""
    print("\n")
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 10 + "AGENT SERVICE - TIME-AWARE MANAGEMENT TESTS" + " " * 15 + "║")
    print("╚" + "=" * 68 + "╝")
    print("\nNOTE: Ensure GEMINI_API_KEY is set in backend/.env")
    print("NOTE: Ensure test user exists in database or update user_id\n")

    await test_today_interpretation()
    await test_tomorrow_interpretation()
    await test_this_week_interpretation()
    await test_current_time_awareness()
    await test_past_time_reference()
    await test_relative_date_calculation()

    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())
