"""
Test script for complete_task tool

This script tests the complete_task tool by directly calling it.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import complete_task, add_task, list_tasks
from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_complete_task():
    """Test complete_task tool functionality."""
    print("Testing complete_task tool...")

    try:
        # Get a valid user_id from database
        async for session in get_async_session():
            stmt = select(User).limit(1)
            result = await session.execute(stmt)
            users = result.scalars().all()

            if not users:
                print("[ERROR] No users found in database. Please create a user first.")
                return False

            test_user_id = users[0].id
            print(f"[OK] Using test user: {users[0].email} (ID: {test_user_id})")

        # Create a test task
        print("\n--- Creating test task ---")
        new_task = await add_task(test_user_id, "Task to Complete", "This will be completed")
        task_id = new_task['task_id']
        print(f"[OK] Created task {task_id}: {new_task['title']}")
        print(f"     Completed: {new_task['completed']}")

        # Test 1: Complete the task
        print("\n--- Test 1: Complete own task ---")
        result1 = await complete_task(user_id=test_user_id, task_id=task_id)
        print(f"[OK] Task completed: ID={result1['task_id']}")
        print(f"     Status: {result1['status']}")
        print(f"     Completed: {result1['completed']}")
        print(f"     Updated at: {result1['updated_at']}")

        # Test 2: Complete already completed task (idempotent)
        print("\n--- Test 2: Complete already completed task (idempotent) ---")
        result2 = await complete_task(user_id=test_user_id, task_id=task_id)
        print(f"[OK] Task remains completed: {result2['completed']}")

        # Test 3: Try to complete non-existent task (should fail)
        print("\n--- Test 3: Complete non-existent task (should fail) ---")
        try:
            result3 = await complete_task(user_id=test_user_id, task_id=99999)
            print("[ERROR] Should have failed with TaskNotFoundError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)}")

        # Test 4: Try to complete another user's task (should fail)
        print("\n--- Test 4: Complete another user's task (should fail) ---")
        try:
            result4 = await complete_task(user_id="invalid-user-id", task_id=task_id)
            print("[ERROR] Should have failed with UnauthorizedTaskAccessError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        # Verify task is completed in list
        print("\n--- Verifying task in completed list ---")
        completed_tasks = await list_tasks(user_id=test_user_id, status="completed")
        print(f"[OK] Found {completed_tasks['count']} completed task(s)")
        task_found = any(t['task_id'] == task_id for t in completed_tasks['tasks'])
        if task_found:
            print(f"[OK] Task {task_id} appears in completed list")
        else:
            print(f"[ERROR] Task {task_id} not found in completed list!")
            return False

        print("\n[SUCCESS] All complete_task tests passed!")
        return True

    except Exception as e:
        print(f"\n[ERROR] Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_complete_task())
    sys.exit(0 if success else 1)
