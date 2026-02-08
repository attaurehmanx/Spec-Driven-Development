"""
Test script for update_task tool

This script tests the update_task tool by directly calling it.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import update_task, add_task
from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_update_task():
    """Test update_task tool functionality."""
    print("Testing update_task tool...")

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
        new_task = await add_task(test_user_id, "Original Title", "Original Description")
        task_id = new_task['task_id']
        print(f"[OK] Created task {task_id}: {new_task['title']}")
        print(f"     Description: {new_task['description']}")

        # Test 1: Update title only
        print("\n--- Test 1: Update title only ---")
        result1 = await update_task(user_id=test_user_id, task_id=task_id, title="Updated Title")
        print(f"[OK] Task updated: ID={result1['task_id']}")
        print(f"     New title: {result1['title']}")
        print(f"     Description (should be unchanged): {result1['description']}")
        if result1['description'] == "Original Description":
            print("[OK] Description unchanged as expected")
        else:
            print("[ERROR] Description was changed unexpectedly!")
            return False

        # Test 2: Update description only
        print("\n--- Test 2: Update description only ---")
        result2 = await update_task(user_id=test_user_id, task_id=task_id, description="Updated Description")
        print(f"[OK] Task updated: ID={result2['task_id']}")
        print(f"     Title (should be unchanged): {result2['title']}")
        print(f"     New description: {result2['description']}")
        if result2['title'] == "Updated Title":
            print("[OK] Title unchanged as expected")
        else:
            print("[ERROR] Title was changed unexpectedly!")
            return False

        # Test 3: Update both fields
        print("\n--- Test 3: Update both title and description ---")
        result3 = await update_task(
            user_id=test_user_id,
            task_id=task_id,
            title="Final Title",
            description="Final Description"
        )
        print(f"[OK] Task updated: ID={result3['task_id']}")
        print(f"     New title: {result3['title']}")
        print(f"     New description: {result3['description']}")

        # Test 4: Try to update with no fields (should fail)
        print("\n--- Test 4: Update with no fields (should fail) ---")
        try:
            result4 = await update_task(user_id=test_user_id, task_id=task_id)
            print("[ERROR] Should have failed with ValidationError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        # Test 5: Try to update with empty title (should fail)
        print("\n--- Test 5: Update with empty title (should fail) ---")
        try:
            result5 = await update_task(user_id=test_user_id, task_id=task_id, title="   ")
            print("[ERROR] Should have failed with ValidationError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        # Test 6: Try to update non-existent task (should fail)
        print("\n--- Test 6: Update non-existent task (should fail) ---")
        try:
            result6 = await update_task(user_id=test_user_id, task_id=99999, title="Test")
            print("[ERROR] Should have failed with TaskNotFoundError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)}")

        # Test 7: Try to update another user's task (should fail)
        print("\n--- Test 7: Update another user's task (should fail) ---")
        try:
            result7 = await update_task(user_id="invalid-user-id", task_id=task_id, title="Hacked")
            print("[ERROR] Should have failed with UnauthorizedTaskAccessError!")
            return False
        except Exception as e:
            print(f"[OK] Failed as expected: {str(e)[:100]}")

        print("\n[SUCCESS] All update_task tests passed!")
        return True

    except Exception as e:
        print(f"\n[ERROR] Test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_update_task())
    sys.exit(0 if success else 1)
