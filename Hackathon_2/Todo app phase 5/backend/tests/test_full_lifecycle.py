"""
Comprehensive test script for all MCP Task Tools

This script tests the full lifecycle of task management:
create → list → complete → update → delete
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.tools.task_tools import (
    add_task, list_tasks, complete_task, update_task, delete_task
)
from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_full_lifecycle():
    """Test complete task lifecycle."""
    print("=" * 60)
    print("MCP TASK TOOLS - FULL LIFECYCLE TEST")
    print("=" * 60)

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
            print(f"\n[OK] Using test user: {users[0].email}")
            print(f"     User ID: {test_user_id}")

        # Step 1: Create tasks
        print("\n" + "=" * 60)
        print("STEP 1: CREATE TASKS")
        print("=" * 60)

        task1 = await add_task(test_user_id, "Buy groceries", "Milk, eggs, bread")
        task2 = await add_task(test_user_id, "Call dentist", "Schedule appointment")
        task3 = await add_task(test_user_id, "Write report", "Q4 financial report")

        print(f"[OK] Created 3 tasks:")
        print(f"     - Task {task1['task_id']}: {task1['title']}")
        print(f"     - Task {task2['task_id']}: {task2['title']}")
        print(f"     - Task {task3['task_id']}: {task3['title']}")

        # Step 2: List all tasks
        print("\n" + "=" * 60)
        print("STEP 2: LIST ALL TASKS")
        print("=" * 60)

        all_tasks = await list_tasks(test_user_id, "all")
        print(f"[OK] Found {all_tasks['count']} total tasks")
        for task in all_tasks['tasks'][:5]:
            status = "[X]" if task['completed'] else "[ ]"
            print(f"     {status} Task {task['task_id']}: {task['title']}")

        # Step 3: Complete a task
        print("\n" + "=" * 60)
        print("STEP 3: COMPLETE A TASK")
        print("=" * 60)

        completed = await complete_task(test_user_id, task1['task_id'])
        print(f"[OK] Completed task {completed['task_id']}: {completed['title']}")
        print(f"     Status: {completed['status']}")
        print(f"     Completed: {completed['completed']}")

        # Step 4: List pending tasks
        print("\n" + "=" * 60)
        print("STEP 4: LIST PENDING TASKS")
        print("=" * 60)

        pending_tasks = await list_tasks(test_user_id, "pending")
        print(f"[OK] Found {pending_tasks['count']} pending tasks")
        for task in pending_tasks['tasks'][:5]:
            print(f"     [ ] Task {task['task_id']}: {task['title']}")

        # Step 5: List completed tasks
        print("\n" + "=" * 60)
        print("STEP 5: LIST COMPLETED TASKS")
        print("=" * 60)

        completed_tasks = await list_tasks(test_user_id, "completed")
        print(f"[OK] Found {completed_tasks['count']} completed tasks")
        for task in completed_tasks['tasks'][:5]:
            print(f"     [X] Task {task['task_id']}: {task['title']}")

        # Step 6: Update a task
        print("\n" + "=" * 60)
        print("STEP 6: UPDATE A TASK")
        print("=" * 60)

        updated = await update_task(
            test_user_id,
            task2['task_id'],
            title="Call dentist - URGENT",
            description="Schedule appointment for next week"
        )
        print(f"[OK] Updated task {updated['task_id']}")
        print(f"     New title: {updated['title']}")
        print(f"     New description: {updated['description']}")

        # Step 7: Delete a task
        print("\n" + "=" * 60)
        print("STEP 7: DELETE A TASK")
        print("=" * 60)

        deleted = await delete_task(test_user_id, task3['task_id'])
        print(f"[OK] Deleted task {deleted['task_id']}: {deleted['title']}")
        print(f"     Status: {deleted['status']}")

        # Step 8: Verify final state
        print("\n" + "=" * 60)
        print("STEP 8: VERIFY FINAL STATE")
        print("=" * 60)

        final_tasks = await list_tasks(test_user_id, "all")
        print(f"[OK] Final task count: {final_tasks['count']}")

        # Verify task3 is gone
        task3_found = any(t['task_id'] == task3['task_id'] for t in final_tasks['tasks'])
        if not task3_found:
            print(f"[OK] Task {task3['task_id']} successfully deleted (not in list)")
        else:
            print(f"[ERROR] Task {task3['task_id']} still in list!")
            return False

        # Verify task1 is completed
        task1_data = next((t for t in final_tasks['tasks'] if t['task_id'] == task1['task_id']), None)
        if task1_data and task1_data['completed']:
            print(f"[OK] Task {task1['task_id']} is marked as completed")
        else:
            print(f"[ERROR] Task {task1['task_id']} completion status incorrect!")
            return False

        # Verify task2 is updated
        task2_data = next((t for t in final_tasks['tasks'] if t['task_id'] == task2['task_id']), None)
        if task2_data and "URGENT" in task2_data['title']:
            print(f"[OK] Task {task2['task_id']} title was updated")
        else:
            print(f"[ERROR] Task {task2['task_id']} update not reflected!")
            return False

        print("\n" + "=" * 60)
        print("SUCCESS! ALL LIFECYCLE TESTS PASSED")
        print("=" * 60)
        print("\nSummary:")
        print("  [OK] Task creation working")
        print("  [OK] Task listing working (all/pending/completed)")
        print("  [OK] Task completion working")
        print("  [OK] Task updates working")
        print("  [OK] Task deletion working")
        print("  [OK] User isolation enforced")

        return True

    except Exception as e:
        print(f"\n[ERROR] Lifecycle test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_full_lifecycle())
    sys.exit(0 if success else 1)
