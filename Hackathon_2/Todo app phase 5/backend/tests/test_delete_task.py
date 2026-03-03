"""
Test script for delete_task tool

This script tests the delete_task tool by directly calling it.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import delete_task, add_task, list_tasks
from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_delete_task():
    """Test delete_task tool functionality."""
    print("Testing delete_task tool...")

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

        # Create test tasks
        print("\n--- Creating test tasks ---")
        task1 = await add_task(test_user_id, "Task to Delete 1", "This will be deleted")
        task2 = await add_task(test_user_id, "Task to Delete 2", "This will also be deleted")
        task1_id = task1['task_id']
        task2_id = task2['task_id']
        print(f"[OK] Created task {task1_id}: {task1['title']}")
        print(f"[OK] Created task {task2_id}: {task2['title']}")

        # Test 1: Delete own task
        print("\n--- Test 1: Delete own task ---")
        result1 = await delete_task(user_id=test_user_id, task_id=task1_id)
        print(f"[OK] Task deleted: ID={result1['task_id']}")
        print(f"     Status: {result1['status']}")
        print(f"     Title: {result1['title']}")

        # Test 2: Verify task is gone from list
        print("\n--- Test 2: Verify task is gone from list ---")
        all_tasks = await list_tasks(user_id=test_user_id, status="all")
        task_found = any(t['task_id'] == task1_id for t in all_tasks['tasks'])
        if not task_found:
            print(f"[OK] Task {task1_id} not found in list (correctly deleted)")
        else:
            print(f"[ERROR] Task {task1_id} still appears in list!")
            return False

        # Test 3: Try to delete already deleted task (should fail)
        print("\n--- Test 3: Delete already deleted task (should fail) ---")
        try:
            result3 = await delete_task(user_id=test_user_id, task_id=task1_id)
            print("[ERROR] Should have failed with TaskNotFoundError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)}")

        # Test 4: Try to delete non-existent task (should fail)
        print("\n--- Test 4: Delete non-existent task (should fail) ---")
        try:
            result4 = await delete_task(user_id=test_user_id, task_id=99999)
            print("[ERROR] Should have failed with TaskNotFoundError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)}")

        # Test 5: Try to delete another user's task (should fail)
        print("\n--- Test 5: Delete another user's task (should fail) ---")
        try:
            result5 = await delete_task(user_id="invalid-user-id", task_id=task2_id)
            print("[ERROR] Should have failed with UnauthorizedTaskAccessError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        # Clean up remaining test task
        print("\n--- Cleaning up remaining test task ---")
        await delete_task(user_id=test_user_id, task_id=task2_id)
        print(f"[OK] Cleaned up task {task2_id}")

        print("\n[SUCCESS] All delete_task tests passed!")
        return True

    except Exception as e:
        print(f"\n[ERROR] Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_delete_task())
    sys.exit(0 if success else 1)
