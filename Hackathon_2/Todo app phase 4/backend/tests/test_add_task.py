"""
Test script for add_task tool

This script tests the add_task tool by directly calling it.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import add_task
from database.session import get_async_session
from models.task_models import User, Task
from sqlmodel import select


async def test_add_task():
    """Test add_task tool functionality."""
    print("Testing add_task tool...")

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

        # Test 1: Create task with title only
        print("\n--- Test 1: Create task with title only ---")
        result1 = await add_task(
            user_id=test_user_id,
            title="Test Task 1 - Title Only"
        )
        print(f"[OK] Task created: ID={result1['task_id']}, Title={result1['title']}")
        print(f"     Description: {result1['description']}")
        print(f"     Completed: {result1['completed']}")

        # Test 2: Create task with title and description
        print("\n--- Test 2: Create task with title and description ---")
        result2 = await add_task(
            user_id=test_user_id,
            title="Test Task 2 - With Description",
            description="This is a test task with a description"
        )
        print(f"[OK] Task created: ID={result2['task_id']}, Title={result2['title']}")
        print(f"     Description: {result2['description']}")

        # Test 3: Try to create task with invalid user_id (should fail)
        print("\n--- Test 3: Create task with invalid user_id (should fail) ---")
        try:
            result3 = await add_task(
                user_id="invalid-user-id-12345",
                title="This should fail"
            )
            print("[ERROR] Task creation should have failed but didn't!")
            return False
        except ValueError as e:
            print(f"[OK] Task creation failed as expected: {str(e)}")

        # Test 4: Try to create task with empty title (should fail)
        print("\n--- Test 4: Create task with empty title (should fail) ---")
        try:
            result4 = await add_task(
                user_id=test_user_id,
                title="   "
            )
            print("[ERROR] Task creation should have failed but didn't!")
            return False
        except ValueError as e:
            print(f"[OK] Task creation failed as expected: {str(e)}")

        # Verify tasks were created in database
        print("\n--- Verifying tasks in database ---")
        async for session in get_async_session():
            stmt = select(Task).where(Task.user_id == test_user_id)
            result = await session.execute(stmt)
            tasks = result.scalars().all()
            print(f"[OK] Found {len(tasks)} task(s) for user {test_user_id}")
            for task in tasks[-2:]:  # Show last 2 tasks
                print(f"     - Task {task.id}: {task.title}")

        print("\n[SUCCESS] All add_task tests passed!")
        return True

    except Exception as e:
        print(f"\n[ERROR] Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_add_task())
    sys.exit(0 if success else 1)
