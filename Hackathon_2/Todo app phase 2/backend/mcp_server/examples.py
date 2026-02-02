"""
Example usage scripts for MCP Task Tools

This file demonstrates how to use each of the 5 task management tools.
"""

import asyncio
from mcp_server.tools.task_tools import (
    add_task, list_tasks, complete_task, update_task, delete_task
)


async def example_usage():
    """Demonstrate usage of all 5 task management tools."""

    # Replace with a valid user_id from your database
    user_id = "550e8400-e29b-41d4-a716-446655440000"

    print("=" * 60)
    print("MCP TASK TOOLS - EXAMPLE USAGE")
    print("=" * 60)

    # Example 1: Create a task
    print("\n1. Creating a task...")
    task = await add_task(
        user_id=user_id,
        title="Buy groceries",
        description="Milk, eggs, bread, vegetables"
    )
    print(f"   Created task #{task['task_id']}: {task['title']}")
    task_id = task['task_id']

    # Example 2: List all tasks
    print("\n2. Listing all tasks...")
    all_tasks = await list_tasks(user_id=user_id, status="all")
    print(f"   Found {all_tasks['count']} total tasks")

    # Example 3: List pending tasks only
    print("\n3. Listing pending tasks...")
    pending = await list_tasks(user_id=user_id, status="pending")
    print(f"   Found {pending['count']} pending tasks")

    # Example 4: Update a task
    print("\n4. Updating task title...")
    updated = await update_task(
        user_id=user_id,
        task_id=task_id,
        title="Buy organic groceries"
    )
    print(f"   Updated task #{updated['task_id']}: {updated['title']}")

    # Example 5: Complete a task
    print("\n5. Marking task as complete...")
    completed = await complete_task(user_id=user_id, task_id=task_id)
    print(f"   Completed task #{completed['task_id']}")

    # Example 6: List completed tasks
    print("\n6. Listing completed tasks...")
    completed_tasks = await list_tasks(user_id=user_id, status="completed")
    print(f"   Found {completed_tasks['count']} completed tasks")

    # Example 7: Delete a task
    print("\n7. Deleting task...")
    deleted = await delete_task(user_id=user_id, task_id=task_id)
    print(f"   Deleted task #{deleted['task_id']}: {deleted['title']}")

    print("\n" + "=" * 60)
    print("All examples completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(example_usage())
