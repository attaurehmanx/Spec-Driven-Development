"""
Test script for list_tasks tool

This script tests the list_tasks tool by directly calling it.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import list_tasks, add_task
from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_list_tasks():
    """Test list_tasks tool functionality."""
    print("Testing list_tasks tool...")

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

        # Create some test tasks first
        print("\n--- Creating test tasks ---")
        await add_task(test_user_id, "Pending Task 1", "This is pending")
        await add_task(test_user_id, "Pending Task 2", "This is also pending")
        print("[OK] Created 2 pending tasks")

        # Test 1: List all tasks
        print("\n--- Test 1: List all tasks ---")
        result1 = await list_tasks(user_id=test_user_id, status="all")
        print(f"[OK] Found {result1['count']} total tasks")
        print(f"     Filter: {result1['filter']}")
        for task in result1['tasks'][:3]:  # Show first 3
            print(f"     - Task {task['task_id']}: {task['title']} (completed={task['completed']})")

        # Test 2: List pending tasks only
        print("\n--- Test 2: List pending tasks only ---")
        result2 = await list_tasks(user_id=test_user_id, status="pending")
        print(f"[OK] Found {result2['count']} pending tasks")
        print(f"     Filter: {result2['filter']}")
        for task in result2['tasks'][:3]:
            print(f"     - Task {task['task_id']}: {task['title']} (completed={task['completed']})")

        # Test 3: List completed tasks only
        print("\n--- Test 3: List completed tasks only ---")
        result3 = await list_tasks(user_id=test_user_id, status="completed")
        print(f"[OK] Found {result3['count']} completed tasks")
        print(f"     Filter: {result3['filter']}")

        # Test 4: Invalid status filter (should fail)
        print("\n--- Test 4: Invalid status filter (should fail) ---")
        try:
            result4 = await list_tasks(user_id=test_user_id, status="invalid")
            print("[ERROR] Should have failed with invalid status!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        # Test 5: Invalid user_id (should fail)
        print("\n--- Test 5: Invalid user_id (should fail) ---")
        try:
            result5 = await list_tasks(user_id="invalid-user-id", status="all")
            print("[ERROR] Should have failed with invalid user!")
            return False
        except ValueError as e:
            print(f"[OK] Failed as expected: {str(e)}")

        print("\n[SUCCESS] All list_tasks tests passed!")
        return True

    except Exception as e:
        print(f"\n[ERROR] Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_list_tasks())
    sys.exit(0 if success else 1)
